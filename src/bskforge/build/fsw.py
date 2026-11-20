#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""FSW build phase: guidance, estimator, tracking error, control, allocation."""

from __future__ import annotations

from typing import Any

from Basilisk.fswAlgorithms import (
    attTrackingError,
    inertial3D,
    mrpFeedback,
    rwMotorTorque,
)
from Basilisk.simulation import simpleNav

from bskforge.build.context import SimContext, SpacecraftBundle
from bskforge.build.registry import apply_params
from bskforge.errors import BuildError
from bskforge.schema.fsw import (
    AllocationConfig,
    ControlConfig,
    FswSection,
)


def build_fsw(ctx: SimContext, bundle: SpacecraftBundle) -> None:
    """Assemble the vehicle's flight-software stack and message graph.

    Order: estimator (navigation) → guidance → attitude tracking error →
    control → allocation. The commanded torque is also routed to the
    vehicle's ``extForceTorque`` effector.

    Args:
        ctx: Simulation build context.
        bundle: Vehicle whose ``fsw`` section is built.
    """
    cfg = bundle.config.fsw
    if cfg is None:
        return

    nav_msg = _build_estimator(ctx, bundle, cfg)
    ref_msg = _build_guidance(ctx, bundle, cfg)
    if cfg.control is None:
        return

    err_msg = _build_tracking_error(ctx, bundle, nav_msg, ref_msg)
    ctrl = _build_control(ctx, bundle, cfg.control, err_msg)
    bundle.fsw["control"] = ctrl
    bundle.ext_ft.cmdTorqueInMsg.subscribeTo(ctrl.cmdTorqueOutMsg)

    if cfg.allocation is not None:
        _build_allocation(ctx, bundle, cfg.allocation, ctrl)


def _build_estimator(ctx: SimContext, bundle: SpacecraftBundle, cfg: FswSection) -> Any:
    """Build the navigation/estimator module and return its attitude msg."""
    if cfg.estimator is None:
        return bundle.sc.scStateOutMsg
    est = cfg.estimator
    if est.type == "simpleNav":
        nav = simpleNav.SimpleNav()
        nav.ModelTag = f"{bundle.config.name}_nav"
        nav.scStateInMsg.subscribeTo(bundle.sc.scStateOutMsg)
        apply_params(nav, est.params, f"estimator {nav.ModelTag!r}")
        ctx.add_fsw(nav)
        ctx.register_tag(nav.ModelTag, nav)
        bundle.fsw["estimator"] = nav
        return nav.attOutMsg
    raise BuildError(f"unsupported estimator type {est.type!r}")


def _build_guidance(ctx: SimContext, bundle: SpacecraftBundle, cfg: FswSection) -> Any:
    """Build the attitude-reference (guidance) module and return its msg."""
    if cfg.guidance is None:
        return None
    guid = cfg.guidance
    if guid.type == "inertial3D":
        ref = inertial3D.inertial3D()
        ref.ModelTag = f"{bundle.config.name}_inertial3D"
        ref.sigma_R0N = list(guid.sigma_R0N)
        apply_params(ref, guid.params, f"guidance {ref.ModelTag!r}")
        ctx.add_fsw(ref)
        ctx.register_tag(ref.ModelTag, ref)
        bundle.fsw["guidance"] = ref
        return ref.attRefOutMsg
    raise BuildError(f"unsupported guidance type {guid.type!r}")


def _build_tracking_error(
    ctx: SimContext, bundle: SpacecraftBundle, nav_msg: Any, ref_msg: Any
) -> Any:
    """Build attitude tracking error combining nav attitude and reference."""
    err = attTrackingError.attTrackingError()
    err.ModelTag = f"{bundle.config.name}_attErr"
    if nav_msg is not None:
        err.attNavInMsg.subscribeTo(nav_msg)
    if ref_msg is not None:
        err.attRefInMsg.subscribeTo(ref_msg)
    ctx.add_fsw(err)
    ctx.register_tag(err.ModelTag, err)
    bundle.fsw["tracking_error"] = err
    return err.attGuidOutMsg


def _build_control(
    ctx: SimContext,
    bundle: SpacecraftBundle,
    control: ControlConfig,
    err_msg: Any,
) -> Any:
    """Build the attitude control law and subscribe it to error/config msgs."""
    if control.type == "mrpFeedback":
        ctrl = mrpFeedback.mrpFeedback()
        ctrl.ModelTag = f"{bundle.config.name}_mrpFeedback"
        ctrl.K = control.k
        ctrl.P = control.p
        ctrl.Ki = control.ki
        ctrl.integralLimit = control.resolved_integral_limit()
        ctrl.knownTorquePntB_B = list(control.known_torque)
        ctrl.guidInMsg.subscribeTo(err_msg)
        ctrl.vehConfigInMsg.subscribeTo(bundle.veh_config_msg)
        if bundle.rw is not None:
            ctrl.rwParamsInMsg.subscribeTo(bundle.rw.cfg_msg)
            ctrl.rwSpeedsInMsg.subscribeTo(bundle.rw.effector.rwSpeedOutMsg)
        apply_params(ctrl, control.params, f"control {ctrl.ModelTag!r}")
        ctx.add_fsw(ctrl)
        ctx.register_tag(ctrl.ModelTag, ctrl)
        return ctrl
    raise BuildError(f"unsupported control type {control.type!r}")


def _build_allocation(
    ctx: SimContext,
    bundle: SpacecraftBundle,
    allocation: AllocationConfig,
    ctrl: Any,
) -> None:
    """Build torque→device allocation (RW motor torque) if configured."""
    if allocation.type == "rwMotorTorque":
        if bundle.rw is None:
            raise BuildError(
                f"spacecraft {bundle.config.name!r}: rwMotorTorque allocation "
                "requires a reactionWheels effector"
            )
        alloc = rwMotorTorque.rwMotorTorque()
        alloc.ModelTag = f"{bundle.config.name}_rwMotorTorque"
        rw_cfg = bundle.config.reaction_wheels()
        if rw_cfg is None:  # guarded by ScenarioConfig cross-check too
            raise BuildError(
                f"spacecraft {bundle.config.name!r}: rwMotorTorque allocation "
                "requires a reactionWheels effector"
            )
        alloc.numControlAxes = len(rw_cfg.wheels)
        alloc.controlAxes_B = [
            float(component)
            for wheel in rw_cfg.wheels
            for component in wheel.direction_B
        ]
        alloc.rwParamsInMsg.subscribeTo(bundle.rw.cfg_msg)
        alloc.vehControlInMsg.subscribeTo(ctrl.cmdTorqueOutMsg)
        bundle.rw.effector.rwMotorCmdInMsg.subscribeTo(alloc.rwMotorTorqueOutMsg)
        apply_params(alloc, allocation.params, f"allocation {alloc.ModelTag!r}")
        ctx.add_fsw(alloc)
        ctx.register_tag(alloc.ModelTag, alloc)
        bundle.fsw["allocation"] = alloc
        return
    raise BuildError(f"unsupported allocation type {allocation.type!r}")
