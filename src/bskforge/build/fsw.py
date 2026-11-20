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

from Basilisk.architecture import messaging, sysModel
from Basilisk.fswAlgorithms import (
    attTrackingError,
    cssWlsEst,
    inertial3D,
    inertialUKF,
    mrpFeedback,
    rwMotorTorque,
    sunlineSuKF,
)
from Basilisk.simulation import simpleNav
from Basilisk.utilities import RigidBodyKinematics as rbk

from bskforge.build.context import SimContext, SpacecraftBundle
from bskforge.build.registry import apply_params
from bskforge.errors import BuildError
from bskforge.schema.fsw import (
    AllocationConfig,
    ControlConfig,
    CssWlsEstEstimator,
    FswSection,
    InertialUkfEstimator,
    SunlineSuKFEstimator,
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
    if est.type == "inertialUKF":
        return _build_inertial_ukf(ctx, bundle, est)
    if est.type == "cssWlsEst":
        return _build_css_estimator(ctx, bundle, est, cssWlsEst.cssWlsEst())
    if est.type == "sunlineSuKF":
        return _build_css_estimator(ctx, bundle, est, sunlineSuKF.sunlineSuKF())
    raise BuildError(f"unsupported estimator type {est.type!r}")


def _build_inertial_ukf(
    ctx: SimContext, bundle: SpacecraftBundle, est: InertialUkfEstimator
) -> Any:
    """Wire the inertial UKF to the vehicle's star trackers and mass props."""
    st_names = est.st_sensors or [
        sensor.name for sensor in bundle.config.sensors if sensor.type == "starTracker"
    ]
    if not st_names:
        raise BuildError(
            f"spacecraft {bundle.config.name!r}: inertialUKF needs at least one "
            "starTracker sensor (or list names in st_sensors)"
        )
    ukf = inertialUKF.inertialUKF()
    ukf.ModelTag = f"{bundle.config.name}_ukf"
    bridge = _StSensorToAttBridge()
    bridge.ModelTag = f"{bundle.config.name}_stAtt"
    st_structs = []
    for name in st_names:
        entry = bundle.sensor_msgs.get(name)
        if entry is None:
            raise BuildError(
                f"spacecraft {bundle.config.name!r}: inertialUKF references "
                f"unknown sensor {name!r}"
            )
        att_msg = messaging.STAttMsg()
        bridge.pairs.append((entry[0], att_msg))
        st_msg = inertialUKF.STMessage()
        st_msg.stInMsg.subscribeTo(att_msg)
        var = est.measurement_noise**2
        st_msg.noise = [
            var if row == col else 0.0 for row in range(3) for col in range(3)
        ]
        st_structs.append(st_msg)
    bridge.att_msgs = [att for _, att in bridge.pairs]
    ukf.STDatasStruct.STMessages = st_structs
    ukf.STDatasStruct.numST = len(st_structs)
    ukf.massPropsInMsg.subscribeTo(bundle.veh_config_msg)
    if bundle.rw is not None:
        ukf.rwParamsInMsg.subscribeTo(bundle.rw.cfg_msg)
        ukf.rwSpeedsInMsg.subscribeTo(bundle.rw.effector.rwSpeedOutMsg)
    dummies: list[Any] = [messaging.AccDataMsg()]  # gyro buffer: connected, silent
    dummies[0].write(messaging.AccDataMsgPayload())
    if bundle.rw is not None:
        ukf.rwParamsInMsg.subscribeTo(bundle.rw.cfg_msg)
        ukf.rwSpeedsInMsg.subscribeTo(bundle.rw.effector.rwSpeedOutMsg)
    else:
        # the module requires every port connected; written-zero messages
        # simply neutralize those inputs
        dummies += [messaging.RWArrayConfigMsg(), messaging.RWSpeedMsg()]
        dummies[1].write(messaging.RWArrayConfigMsgPayload())  # ty: ignore
        dummies[2].write(messaging.RWSpeedMsgPayload())  # ty: ignore
        ukf.rwParamsInMsg.subscribeTo(dummies[1])
        ukf.rwSpeedsInMsg.subscribeTo(dummies[2])
    ukf.gyrBuffInMsg.subscribeTo(dummies[0])
    bundle.fsw["ukf_dummy_msgs"] = dummies
    att_var = est.process_noise_att**2
    rate_var = est.process_noise_rate**2
    ukf.qNoise = _diag_flat_36(att_var, att_var, att_var, rate_var, rate_var, rate_var)
    ukf.covarInit = _diag_flat_36(
        est.covar_init_att,
        est.covar_init_att,
        est.covar_init_att,
        est.covar_init_rate,
        est.covar_init_rate,
        est.covar_init_rate,
    )
    ukf.switchMag = est.switch_mag
    ukf.alpha = est.alpha
    ukf.beta = est.beta
    ukf.kappa = est.kappa
    ukf.stateInit = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    apply_params(ukf, est.params, f"estimator {ukf.ModelTag!r}")
    # the bridge is added before the UKF so it converts first in the FSW task
    ctx.add_fsw(bridge)
    ctx.add_fsw(ukf)
    ctx.register_tag(ukf.ModelTag, ukf)
    bundle.fsw["estimator"] = ukf
    bundle.fsw["st_bridge"] = bridge  # keep the Python model alive
    bundle.estimator_msg = ukf.navStateOutMsg
    return ukf.navStateOutMsg


def _build_css_estimator(
    ctx: SimContext,
    bundle: SpacecraftBundle,
    est: CssWlsEstEstimator | SunlineSuKFEstimator,
    module: Any,
) -> Any:
    """Wire a CSS sunline filter (WLS or square-root UKF) to its cluster."""
    cluster = bundle.css_clusters.get(est.cluster)
    if cluster is None:
        known = ", ".join(sorted(bundle.css_clusters)) or "none configured"
        raise BuildError(
            f"spacecraft {bundle.config.name!r}: estimator {est.type!r} needs "
            f"css_constellation {est.cluster!r}; available clusters: {known}"
        )
    constellation, const_msg = cluster
    module.ModelTag = f"{bundle.config.name}_{est.type}"
    module.cssDataInMsg.subscribeTo(constellation.constellationOutMsg)
    module.cssConfigInMsg.subscribeTo(const_msg)
    if isinstance(est, SunlineSuKFEstimator):
        if est.q_obs is not None:
            module.qObs = est.q_obs
        module.outputSunline = est.output_sunline
    elif est.use_weights is not None:
        module.useWeights = est.use_weights
    apply_params(module, est.params, f"estimator {module.ModelTag!r}")
    ctx.add_fsw(module)
    ctx.register_tag(module.ModelTag, module)
    bundle.fsw["estimator"] = module
    bundle.estimator_msg = module.navStateOutMsg
    return module.navStateOutMsg


def _diag_flat_36(*variances: float) -> list[float]:
    """Flatten a 6x6 diagonal covariance (padded to 36 entries)."""
    diag = list(variances) + [0.0] * (6 - len(variances))
    flat = [0.0] * 36
    for index, value in enumerate(diag):
        flat[index * 6 + index] = value
    return flat


class _StSensorToAttBridge(sysModel.SysModel):
    """Re-publish star-tracker quaternion outputs as ``STAttMsg``.

    Basilisk sensor modules write ``STSensorMsg`` (quaternion), while the
    inertial filter modules read ``STAttMsg`` (MRP); this small Python
    ``SysModel`` performs that conversion once per FSW tick.
    """

    def __init__(self) -> None:
        """Initialize the message pair lists."""
        super().__init__()
        self.pairs: list[tuple[Any, Any]] = []
        self.att_msgs: list[Any] = []

    def Reset(self, CurrentSimNanos: int) -> None:
        """Write a zero attitude so downstream readers are initialized."""
        for _, att_msg in self.pairs:
            att_msg.write(
                messaging.STAttMsgPayload(),  # ty: ignore
                CurrentSimNanos,
                self.moduleID,
            )

    def UpdateState(self, CurrentSimNanos: int) -> None:
        """Convert every sensor measurement to its MRP message."""
        for sensor_msg, att_msg in self.pairs:
            sensed = sensor_msg.read()
            quat = [float(component) for component in sensed.qInrtl2Case]
            if not any(quat):
                continue  # sensor has not written yet; keep the reset value
            payload = messaging.STAttMsgPayload()  # ty: ignore
            payload.timeTag = sensed.timeTag
            payload.MRP_BdyInrtl = [float(component) for component in rbk.EP2MRP(quat)]
            att_msg.write(payload, CurrentSimNanos, self.moduleID)


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
