#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Effector build phase: reaction wheels, thrusters and magnetic torque bars."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from Basilisk.architecture import messaging
from Basilisk.simulation import (
    MtbEffector,
    reactionWheelStateEffector,
    thrusterDynamicEffector,
)
from Basilisk.utilities import simIncludeRW, simIncludeThruster

from bskforge.build.context import (
    MtbBundle,
    RwBundle,
    SimContext,
    SpacecraftBundle,
    ThrusterBundle,
)
from bskforge.build.environment import MAGNETIC_FIELD
from bskforge.errors import BuildError
from bskforge.schema.effectors import (
    MtbConfig,
    ReactionWheelsConfig,
    ThrustersConfig,
    WheelConfig,
)

#: Jitter model label -> Basilisk RWModel enum.
JITTER_MODELS = {
    "none": reactionWheelStateEffector.BalancedWheels,
    "constant": reactionWheelStateEffector.JitterSimple,
    "speedDependent": reactionWheelStateEffector.JitterFullyCoupled,
}

EFFECTOR_BUILDERS: dict[str, Callable[..., Any]] = {}


def effector_builder(type_name: str) -> Callable[[Callable], Callable]:
    """Register a builder function for an effector ``type``."""

    def _decorator(fn: Callable) -> Callable:
        EFFECTOR_BUILDERS[type_name] = fn
        return fn

    return _decorator


@effector_builder("reactionWheels")
def build_reaction_wheels(
    ctx: SimContext, bundle: SpacecraftBundle, cfg: ReactionWheelsConfig
) -> None:
    """Attach a reaction-wheel state effector built from ``simIncludeRW``.

    Args:
        ctx: Simulation build context.
        bundle: The vehicle this RW set belongs to.
        cfg: Reaction-wheel set configuration.
    """
    if not cfg.wheels:
        raise BuildError(
            f"spacecraft {bundle.config.name!r}: reactionWheels effector has no wheels"
        )

    factory = simIncludeRW.rwFactory()
    for index, wheel in enumerate(cfg.wheels):
        label = _wheel_label(wheel, index)
        factory.create(
            "custom",
            list(wheel.direction_B),
            Js=cfg.inertia,
            maxMomentum=cfg.inertia * wheel.max_speed,
            Omega=wheel.initial_speed,  # [rpm]
            # with no explicit limit, command torques verbatim; the
            # factory otherwise clamps to an enormous u_max and pins the
            # wheel at its speed limit within milliseconds
            useMaxTorque=wheel.max_torque is not None,
            u_max=wheel.max_torque if wheel.max_torque is not None else 1e10,
            RWModel=JITTER_MODELS[wheel.jitter_model],
            label=label,
            **wheel.params,
        )

    rw_effector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rw_effector.ModelTag = f"{bundle.config.name}_rwEff"
    factory.addToSpacecraft(rw_effector.ModelTag, rw_effector, bundle.sc)
    ctx.add_dyn(rw_effector)
    ctx.register_tag(rw_effector.ModelTag, rw_effector)

    bundle.rw = RwBundle(
        effector=rw_effector,
        factory=factory,
        cfg_msg=factory.getConfigMessage(),
        recorder_cfg=cfg.recorder,
    )


def _wheel_label(wheel: WheelConfig, index: int) -> str:
    """Return a Basilisk-valid (<=5 char) RW device label."""
    return (wheel.name or f"RW{index}")[:5]


@effector_builder("thrusters")
def build_thrusters(
    ctx: SimContext, bundle: SpacecraftBundle, cfg: ThrustersConfig
) -> None:
    """Attach a thruster dynamic effector built from ``simIncludeThruster``.

    Args:
        ctx: Simulation build context.
        bundle: The vehicle this thruster set belongs to.
        cfg: Thruster set configuration.
    """
    factory = simIncludeThruster.thrusterFactory()
    for index, device in enumerate(cfg.thrusters):
        kwargs: dict[str, Any] = dict(device.params)
        if device.max_thrust is not None:
            kwargs["MaxThrust"] = device.max_thrust
        factory.create(
            device.model,
            list(device.position_B),
            list(device.direction_B),
            label=(device.name or f"TH{index}")[:5],
            **kwargs,
        )
    effector = thrusterDynamicEffector.ThrusterDynamicEffector()
    factory.addToSpacecraft(f"{bundle.config.name}_thrEff", effector, bundle.sc)
    ctx.add_dyn(effector)
    ctx.register_tag(f"{bundle.config.name}_thrEff", effector)
    bundle.thruster = ThrusterBundle(
        effector=effector,
        factory=factory,
        cfg_msg=factory.getConfigMessage(),
    )


@effector_builder("mtb")
def build_mtb(ctx: SimContext, bundle: SpacecraftBundle, cfg: MtbConfig) -> None:
    """Attach a magnetic torque-bar effector with its array configuration.

    Args:
        ctx: Simulation build context.
        bundle: The vehicle this MTB set belongs to.
        cfg: Torque-bar configuration.

    Raises:
        BuildError: No magnetic field model is configured for the vehicle.
    """
    field_msg = ctx.env_msgs.get((bundle.config.name, MAGNETIC_FIELD))
    if field_msg is None:
        raise BuildError(
            f"spacecraft {bundle.config.name!r}: an mtb effector needs an "
            "[environment.magnetic_field] model to generate torque against"
        )
    payload = messaging.MTBArrayConfigMsgPayload()
    payload.numMTB = len(cfg.bars)
    # component-major storage: axis components of every bar, x-row first
    payload.GtMatrix_B = [
        float(bar.direction_B[component]) for component in range(3) for bar in cfg.bars
    ]
    payload.maxMtbDipoles = [bar.max_dipole for bar in cfg.bars]
    params_msg = messaging.MTBArrayConfigMsg().write(payload)

    effector = MtbEffector.MtbEffector()
    effector.ModelTag = f"{bundle.config.name}_mtbEff"
    bundle.sc.addDynamicEffector(effector)
    effector.magInMsg.subscribeTo(field_msg)
    effector.mtbParamsInMsg.subscribeTo(params_msg)
    ctx.add_dyn(effector)
    ctx.register_tag(effector.ModelTag, effector)
    bundle.mtb = MtbBundle(effector=effector, params_msg=params_msg)
