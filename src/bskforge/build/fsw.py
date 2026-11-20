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

import numpy as np
from Basilisk.architecture import messaging, sysModel
from Basilisk.fswAlgorithms import (
    attTrackingError,
    celestialTwoBodyPoint,
    cssWlsEst,
    dipoleMapping,
    hillPoint,
    inertial3D,
    inertialUKF,
    locationPointing,
    mrpFeedback,
    mrpPD,
    mrpSteering,
    mtbMomentumManagementSimple,
    rateServoFullNonlinear,
    rwMotorTorque,
    rwNullSpace,
    sunlineEphem,
    sunlineSuKF,
    sunSafePoint,
    tamComm,
    thrFiringRound,
    thrFiringSchmitt,
    thrForceMapping,
    thrMomentumDumping,
    thrMomentumManagement,
    torque2Dipole,
    velocityPoint,
    waypointReference,
)
from Basilisk.simulation import magnetometer, simpleNav
from Basilisk.utilities import RigidBodyKinematics as rbk

from bskforge.build.context import SimContext, SpacecraftBundle
from bskforge.build.environment import MAGNETIC_FIELD
from bskforge.build.registry import apply_params
from bskforge.errors import BuildError
from bskforge.schema.fsw import (
    AllocationConfig,
    ControlConfig,
    CssWlsEstEstimator,
    DesaturationConfig,
    FswSection,
    InertialUkfEstimator,
    MomentumManagementConfig,
    SunlineSuKFEstimator,
    ThrusterTorqueAllocation,
)


def build_fsw(ctx: SimContext, bundle: SpacecraftBundle) -> None:
    """Assemble the vehicle's flight-software stack and message graph.

    Order: estimator (navigation) → guidance → attitude tracking error →
    control → allocation → momentum management → desaturation. The
    commanded torque reaches the hub directly only when no allocation is
    configured; otherwise the effectors provide the torque.

    Args:
        ctx: Simulation build context.
        bundle: Vehicle whose ``fsw`` section is built.
    """
    cfg = bundle.config.fsw
    if cfg is None:
        return

    needs_nav = cfg.guidance is not None and cfg.guidance.type in {
        "sunSafePoint",
        "locationPointing",
    }
    nav_msg = _build_estimator(ctx, bundle, cfg, needs_nav)
    ref_msg, ref_kind = _build_guidance(ctx, bundle, cfg, nav_msg)

    if cfg.control is not None:
        if ref_kind == "guid":
            # guidance modules that emit a tracking error directly
            err_msg = ref_msg
        else:
            err_msg = _build_tracking_error(ctx, bundle, nav_msg, ref_msg)
        ctrl = _build_control(ctx, bundle, cfg.control, err_msg)
        bundle.fsw["control"] = ctrl
        if cfg.allocation is None:
            # with no effector allocation, the commanded torque acts on the
            # hub directly; otherwise only the allocation may actuate
            bundle.ext_ft.cmdTorqueInMsg.subscribeTo(ctrl.cmdTorqueOutMsg)
        else:
            _build_allocation(ctx, bundle, cfg.allocation, ctrl)
    if cfg.momentum_management is not None:
        _build_momentum_management(ctx, bundle, cfg.momentum_management)
    if cfg.desaturation is not None:
        _build_desaturation(ctx, bundle, cfg.desaturation)


def _build_estimator(
    ctx: SimContext, bundle: SpacecraftBundle, cfg: FswSection, needs_nav: bool = False
) -> Any:
    """Build the navigation/estimator module and return its attitude msg."""
    if cfg.estimator is None:
        if not needs_nav and cfg.control is None:
            return bundle.sc.scStateOutMsg
        # the control chain needs a NavAttMsg; insert a silent truth pass
        nav = simpleNav.SimpleNav()
        nav.ModelTag = f"{bundle.config.name}_nav"
        nav.scStateInMsg.subscribeTo(bundle.sc.scStateOutMsg)
        ctx.add_fsw(nav)
        ctx.register_tag(nav.ModelTag, nav)
        bundle.fsw["estimator"] = nav
        return nav.attOutMsg
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


class _DumpRearm(sysModel.SysModel):
    """Re-arm a ``thrMomentumManagement`` module on a fixed period.

    The module computes its delta-H request only once per ``Reset`` and
    ``thrMomentumDumping`` discards requests stamped at ``t = 0``, so a
    plain chain would never fire; this bridge re-triggers the evaluation
    every ``period`` seconds after start-up.
    """

    def __init__(self, desat: Any, period: float) -> None:
        """Store the module to re-arm and the trigger period in seconds."""
        super().__init__()
        self.desat = desat
        self.period_ns = int(period * 1e9)

    def Reset(self, CurrentSimNanos: int) -> None:
        """No state to reset."""

    def UpdateState(self, CurrentSimNanos: int) -> None:
        """Re-arm the delta-H evaluation on period multiples past t=0."""
        if CurrentSimNanos and CurrentSimNanos % self.period_ns == 0:
            self.desat.Reset(CurrentSimNanos)


def _ensure_trans_msg(ctx: SimContext, bundle: SpacecraftBundle) -> Any:
    """Return the vehicle's ``NavTransMsg``, adding a silent truth pass.

    The orbit-aware guidance modules read ``NavTransMsg``; ``SimpleNav``
    produces one, so it is reused from the estimator when possible and
    inserted silently otherwise.
    """
    cached = bundle.fsw.get("trans_msg")
    if cached is not None:
        return cached
    estimator = bundle.fsw.get("estimator")
    if estimator is not None and hasattr(estimator, "transOutMsg"):
        bundle.fsw["trans_msg"] = estimator.transOutMsg
        return bundle.fsw["trans_msg"]
    nav = simpleNav.SimpleNav()
    nav.ModelTag = f"{bundle.config.name}_navTrans"
    nav.scStateInMsg.subscribeTo(bundle.sc.scStateOutMsg)
    ctx.add_fsw(nav)
    ctx.register_tag(nav.ModelTag, nav)
    bundle.fsw["trans_msg"] = nav.transOutMsg
    return bundle.fsw["trans_msg"]


class _SpiceToEphemeris(sysModel.SysModel):
    """Re-publish a ``SpicePlanetStateMsg`` as an ``EphemerisMsg``.

    The ephemeris-driven guidance modules read ``EphemerisMsg``, while
    ``SpiceInterface`` writes the distinct ``SpicePlanetStateMsg`` type;
    this bridge copies position, velocity and orientation each tick.
    """

    def __init__(self, spice_msg: Any, out_msg: Any) -> None:
        """Store the source message and the ephemeris destination."""
        super().__init__()
        self.spice_msg = spice_msg
        self.out_msg = out_msg

    def Reset(self, CurrentSimNanos: int) -> None:
        """Write a zero-initialized ephemeris so readers start clean."""
        self.out_msg.write(
            messaging.EphemerisMsgPayload(),
            CurrentSimNanos,
            self.moduleID,
        )

    def UpdateState(self, CurrentSimNanos: int) -> None:
        """Copy the live SPICE state into the ephemeris message."""
        planet = self.spice_msg.read()
        payload = messaging.EphemerisMsgPayload()
        payload.timeTag = CurrentSimNanos
        payload.r_BdyZero_N = [float(x) for x in planet.PositionVector]
        payload.v_BdyZero_N = [float(x) for x in planet.VelocityVector]
        j2k_to_planet = np.array(planet.J20002Pfix, dtype=float).reshape(3, 3)
        payload.sigma_BN = [float(x) for x in rbk.C2MRP(j2k_to_planet)]
        self.out_msg.write(payload, CurrentSimNanos, self.moduleID)


def _ephem_from(ctx: SimContext, source: Any, key: str) -> Any:
    """Return a cached ``EphemerisMsg`` carrying the same state as ``source``."""
    cache: dict[str, Any] = ctx.__dict__.setdefault("_ephem_msgs", {})
    if key in cache:
        return cache[key]
    out = messaging.EphemerisMsg()
    if "SpicePlanetState" in type(source).__name__:
        bridge = _SpiceToEphemeris(source, out)
        bridge.ModelTag = f"ephem_{key}"
        ctx.add_dyn(bridge)
        ctx.register_tag(bridge.ModelTag, bridge)
    else:
        out.subscribeTo(source)  # ty: ignore
    cache[key] = out
    return out


def _ephem_for(ctx: SimContext, name: str, purpose: str) -> Any:
    """Return an ``EphemerisMsg`` for body ``name`` (bridging SPICE output)."""
    return _ephem_from(ctx, _body_ephem(ctx, name, purpose), name.lower())


def _central_ephem(ctx: SimContext) -> Any | None:
    """Return the central body's live ephemeris message when SPICE-driven."""
    if ctx.central_body is None:
        return None
    for name, body in ctx.grav_factory.gravBodies.items():
        if body is ctx.central_body:
            return ctx.planet_state_msgs.get(str(name).lower())
    return None


def _body_ephem(ctx: SimContext, name: str, purpose: str) -> Any:
    """Return the ephemeris message for ``name`` (sun fallback: ``[world.sun]``)."""
    lowered = name.lower()
    msg = ctx.planet_state_msgs.get(lowered)
    if msg is None and lowered == "sun":
        msg = ctx.ensure_sun_msg()
    if msg is None:
        raise BuildError(
            f"{purpose} needs the ephemeris of {name!r}; enable [world.spice] "
            f"and add {name!r} to [[world.gravities]]"
        )
    return msg


class _GroundStateWriter(sysModel.SysModel):
    """Publish the inertial state of a fixed point on a SPICE body.

    ``locationPointing`` wants a ``GroundStateMsg``; this bridge converts
    geodetic coordinates to an inertial vector every FSW tick using the
    planet's SPICE orientation.
    """

    def __init__(
        self,
        planet_msg: Any,
        out_msg: Any,
        lat_deg: float,
        lon_deg: float,
        elevation: float,
        radius_eq: float,
        radius_polar: float,
    ) -> None:
        """Store the ephemeris source, destination and point coordinates."""
        super().__init__()
        self.planet_msg = planet_msg
        self.out_msg = out_msg
        lat = np.radians(lat_deg)
        lon = np.radians(lon_deg)
        a, b = radius_eq, radius_polar
        # geodetic latitude is defined by the surface normal of the ellipsoid
        normal = np.array(
            [np.cos(lat) * np.cos(lon), np.cos(lat) * np.sin(lon), np.sin(lat)]
        )
        # surface point along that normal, offset by the elevation
        gamma = np.sqrt(
            (a * normal[0]) ** 2 + (a * normal[1]) ** 2 + (b * normal[2]) ** 2
        )
        surface = np.array([a * a * normal[0], a * a * normal[1], b * b * normal[2]])
        self.body_fixed = surface / gamma + elevation * normal

    def Reset(self, CurrentSimNanos: int) -> None:
        """Write a zero-initialized message so readers start clean."""
        self.out_msg.write(
            messaging.GroundStateMsgPayload(),
            CurrentSimNanos,
            self.moduleID,
        )

    def UpdateState(self, CurrentSimNanos: int) -> None:
        """Rotate the body-fixed point into the inertial frame."""
        planet = self.planet_msg.read()
        dcm_bn = np.array(planet.J20002Pfix, dtype=float).reshape(
            3, 3
        )  # N -> planet-fixed
        position = np.array([float(x) for x in planet.PositionVector])
        r_l_n = position + dcm_bn.T @ self.body_fixed
        payload = messaging.GroundStateMsgPayload()
        # locationPointing reads r_LN_N: the location's inertial position
        payload.r_LN_N = [float(x) for x in r_l_n]
        payload.r_LP_N = [float(x) for x in r_l_n - position]
        norm = float(np.linalg.norm(r_l_n))
        payload.rHat_LP_N = (
            [float(x) / norm for x in r_l_n] if norm > 0.0 else [0.0, 0.0, 0.0]
        )
        self.out_msg.write(payload, CurrentSimNanos, self.moduleID)


def _resolve_scenario_path(ctx: SimContext, path_str: str) -> Any:
    """Resolve ``path_str`` against the scenario file directory when relative."""
    from pathlib import Path

    path = Path(path_str)
    if path.is_absolute() or path.exists():
        return path
    source = getattr(ctx.config, "source_path", None)
    if source is not None:
        candidate = Path(str(source)).parent / path
        if candidate.exists():
            return candidate
    raise BuildError(
        f"file {path_str!r} not found "
        "(relative paths resolve against the scenario file's directory)"
    )


def _build_guidance(
    ctx: SimContext, bundle: SpacecraftBundle, cfg: FswSection, nav_msg: Any
) -> tuple[Any, str]:
    """Build the attitude-reference module; return its msg and kind.

    ``kind`` is ``"ref"`` for an ``AttRefMsg`` consumed by
    ``attTrackingError``, or ``"guid"`` for a module that emits the
    tracking error (``AttGuidMsg``) directly.
    """
    if cfg.guidance is None:
        return None, "ref"
    guid = cfg.guidance
    name = bundle.config.name
    if guid.type == "inertial3D":
        ref = inertial3D.inertial3D()
        ref.ModelTag = f"{name}_inertial3D"
        ref.sigma_R0N = list(guid.sigma_R0N)
        apply_params(ref, guid.params, f"guidance {ref.ModelTag!r}")
        ctx.add_fsw(ref)
        ctx.register_tag(ref.ModelTag, ref)
        bundle.fsw["guidance"] = ref
        return ref.attRefOutMsg, "ref"
    if guid.type == "velocityPoint":
        ref = velocityPoint.velocityPoint()
        ref.ModelTag = f"{name}_velocityPoint"
        ref.mu = guid.mu if guid.mu is not None else ctx.central_mu
        ref.transNavInMsg.subscribeTo(_ensure_trans_msg(ctx, bundle))
        planet = _central_ephem(ctx)
        if planet is not None:
            ref.celBodyInMsg.subscribeTo(_ephem_from(ctx, planet, "central"))
        apply_params(ref, guid.params, f"guidance {ref.ModelTag!r}")
        ctx.add_fsw(ref)
        ctx.register_tag(ref.ModelTag, ref)
        bundle.fsw["guidance"] = ref
        return ref.attRefOutMsg, "ref"
    if guid.type == "hillPoint":
        ref = hillPoint.hillPoint()
        ref.ModelTag = f"{name}_hillPoint"
        ref.transNavInMsg.subscribeTo(_ensure_trans_msg(ctx, bundle))
        planet = _central_ephem(ctx)
        if planet is not None:
            ref.celBodyInMsg.subscribeTo(_ephem_from(ctx, planet, "central"))
        apply_params(ref, guid.params, f"guidance {ref.ModelTag!r}")
        ctx.add_fsw(ref)
        ctx.register_tag(ref.ModelTag, ref)
        bundle.fsw["guidance"] = ref
        return ref.attRefOutMsg, "ref"
    if guid.type == "sunSafePoint":
        sunline = sunlineEphem.sunlineEphem()
        sunline.ModelTag = f"{name}_sunlineEphem"
        sunline.scPositionInMsg.subscribeTo(_ensure_trans_msg(ctx, bundle))
        sunline.scAttitudeInMsg.subscribeTo(nav_msg)
        sunline.sunPositionInMsg.subscribeTo(
            _ephem_from(ctx, ctx.ensure_sun_msg(), "sun")
        )
        ctx.add_fsw(sunline)
        ctx.register_tag(sunline.ModelTag, sunline)
        ref = sunSafePoint.sunSafePoint()
        ref.ModelTag = f"{name}_sunSafePoint"
        ref.sHatBdyCmd = list(guid.sun_axis_B)
        ref.sunAxisSpinRate = guid.spin_rate
        ref.sunDirectionInMsg.subscribeTo(sunline.navStateOutMsg)
        ref.imuInMsg.subscribeTo(nav_msg)
        apply_params(ref, guid.params, f"guidance {ref.ModelTag!r}")
        ctx.add_fsw(ref)
        ctx.register_tag(ref.ModelTag, ref)
        bundle.fsw["guidance"] = ref
        return ref.attGuidanceOutMsg, "guid"
    if guid.type == "locationPointing":
        ref = locationPointing.locationPointing()
        ref.ModelTag = f"{name}_locationPointing"
        latlon = guid.target_latlon or []
        ref.pHat_B = list(guid.point_axis_B)
        ref.cHat_B = list(guid.scan_axis_B)
        ref.scAttInMsg.subscribeTo(nav_msg)
        ref.scTransInMsg.subscribeTo(_ensure_trans_msg(ctx, bundle))
        if guid.target_body is not None:
            ref.celBodyInMsg.subscribeTo(
                _ephem_for(ctx, guid.target_body, "locationPointing")
            )
        elif guid.target_position_N is not None:
            target = messaging.NavTransMsgPayload()
            target.r_BN_N = list(guid.target_position_N)
            ref.scTargetInMsg.subscribeTo(messaging.NavTransMsg().write(target))
        else:
            planet = _central_ephem(ctx)
            if planet is None:
                raise BuildError(
                    f"spacecraft {name!r}: locationPointing target_latlon needs "
                    "the central body's SPICE orientation"
                )
            writer = _GroundStateWriter(
                planet,
                messaging.GroundStateMsg(),
                latlon[0],
                latlon[1],
                latlon[2],
                float(ctx.central_body.radEquator),
                float(ctx.central_body.radEquator)
                * float(ctx.central_body.radiusRatio),
            )
            writer.ModelTag = f"{name}_groundState"
            ctx.add_fsw(writer)
            ctx.register_tag(writer.ModelTag, writer)
            ref.locationInMsg.subscribeTo(writer.out_msg)
            bundle.fsw["ground_state"] = writer
        apply_params(ref, guid.params, f"guidance {ref.ModelTag!r}")
        ctx.add_fsw(ref)
        ctx.register_tag(ref.ModelTag, ref)
        bundle.fsw["guidance"] = ref
        return ref.attRefOutMsg, "ref"
    if guid.type == "celestialTwoBodyPoint":
        ref = celestialTwoBodyPoint.celestialTwoBodyPoint()
        ref.ModelTag = f"{name}_celestialTwoBodyPoint"
        ref.transNavInMsg.subscribeTo(_ensure_trans_msg(ctx, bundle))
        ref.celBodyInMsg.subscribeTo(
            _ephem_for(ctx, guid.target_body, "celestialTwoBodyPoint")
        )
        ref.secCelBodyInMsg.subscribeTo(
            _ephem_for(ctx, guid.constraint_body, "celestialTwoBodyPoint")
        )
        apply_params(ref, guid.params, f"guidance {ref.ModelTag!r}")
        ctx.add_fsw(ref)
        ctx.register_tag(ref.ModelTag, ref)
        bundle.fsw["guidance"] = ref
        return ref.attRefOutMsg, "ref"
    if guid.type == "waypointReference":
        path = _resolve_scenario_path(ctx, str(guid.waypoint_file))
        ref = waypointReference.WaypointReference()
        ref.ModelTag = f"{name}_waypointReference"
        ref.dataFileName = str(path)
        ref.delimiter = guid.delimiter
        ref.attitudeType = guid.attitude_type
        ref.useReferenceFrame = guid.reference_frame
        ref.headerLines = guid.header_lines
        apply_params(ref, guid.params, f"guidance {ref.ModelTag!r}")
        ctx.add_fsw(ref)
        ctx.register_tag(ref.ModelTag, ref)
        bundle.fsw["guidance"] = ref
        return ref.attRefOutMsg, "ref"
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
    if control.type == "mrpPD":
        ctrl = mrpPD.mrpPD()
        ctrl.ModelTag = f"{bundle.config.name}_mrpPD"
        ctrl.K = control.k
        ctrl.P = control.p
        ctrl.knownTorquePntB_B = list(control.known_torque)
        ctrl.guidInMsg.subscribeTo(err_msg)
        ctrl.vehConfigInMsg.subscribeTo(bundle.veh_config_msg)
        apply_params(ctrl, control.params, f"control {ctrl.ModelTag!r}")
        ctx.add_fsw(ctrl)
        ctx.register_tag(ctrl.ModelTag, ctrl)
        return ctrl
    if control.type == "rateServo":
        steering = mrpSteering.mrpSteering()
        steering.ModelTag = f"{bundle.config.name}_mrpSteering"
        steering.omega_max = control.max_rate
        steering.K1 = control.steering_k1
        steering.K3 = control.steering_k3
        steering.guidInMsg.subscribeTo(err_msg)
        ctx.add_fsw(steering)
        ctx.register_tag(steering.ModelTag, steering)
        ctrl = rateServoFullNonlinear.rateServoFullNonlinear()
        ctrl.ModelTag = f"{bundle.config.name}_rateServo"
        ctrl.P = control.p
        ctrl.Ki = control.ki
        ctrl.integralLimit = control.resolved_integral_limit()
        ctrl.knownTorquePntB_B = list(control.known_torque)
        ctrl.guidInMsg.subscribeTo(err_msg)
        ctrl.vehConfigInMsg.subscribeTo(bundle.veh_config_msg)
        ctrl.rateSteeringInMsg.subscribeTo(steering.rateCmdOutMsg)
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
        # torque commands live in the full body frame; the allocation
        # spreads them across however many wheels the set contains
        alloc.numControlAxes = 3
        alloc.controlAxes_B = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        alloc.rwParamsInMsg.subscribeTo(bundle.rw.cfg_msg)
        alloc.vehControlInMsg.subscribeTo(ctrl.cmdTorqueOutMsg)
        bundle.rw.effector.rwMotorCmdInMsg.subscribeTo(alloc.rwMotorTorqueOutMsg)
        apply_params(alloc, allocation.params, f"allocation {alloc.ModelTag!r}")
        ctx.add_fsw(alloc)
        ctx.register_tag(alloc.ModelTag, alloc)
        bundle.fsw["allocation"] = alloc
        return
    if allocation.type == "thrusterTorque":
        thruster = bundle.thruster
        if thruster is None:
            raise BuildError(
                f"spacecraft {bundle.config.name!r}: thrusterTorque allocation "
                "requires a thrusters effector"
            )
        mapping = _build_thr_force_mapping(
            ctx,
            bundle,
            f"{bundle.config.name}_thrForceMapping",
            ctrl.cmdTorqueOutMsg,
            ang_err_thresh=allocation.ang_err_thresh,
        )
        mapping.thrForceSign = allocation.force_sign
        on_time_msg = _build_firing_logic(ctx, bundle, allocation, mapping)
        thruster.effector.cmdsInMsg.subscribeTo(on_time_msg)
        thruster.cmd_src_msg = on_time_msg
        bundle.fsw["allocation"] = mapping
        return
    if allocation.type == "dipoleMapping":
        mtb = _require_mtb_torque_path(ctx, bundle)
        chain = _build_dipole_chain(ctx, bundle, ctrl.cmdTorqueOutMsg)
        if allocation.steering_matrix is not None:
            bundle.fsw["dipole_mapping"].steeringMatrix = list(
                allocation.steering_matrix
            )
        mtb.effector.mtbCmdInMsg.subscribeTo(chain)
        mtb.cmd_src_msg = chain
        bundle.fsw["allocation"] = bundle.fsw["dipole_mapping"]
        return
    raise BuildError(f"unsupported allocation type {allocation.type!r}")


def _build_thr_force_mapping(
    ctx: SimContext,
    bundle: SpacecraftBundle,
    tag: str,
    torque_msg: Any,
    ang_err_thresh: float | None = None,
) -> Any:
    """Build ``thrForceMapping`` mapping a torque msg to thruster pulses."""
    thruster = bundle.thruster
    if thruster is None:
        raise BuildError(
            f"spacecraft {bundle.config.name!r}: thruster force mapping "
            "requires a thrusters effector"
        )
    mapping = thrForceMapping.thrForceMapping()
    mapping.ModelTag = tag
    mapping.thrForceSign = 1
    mapping.numControlAxes = 3
    mapping.controlAxes_B = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    if ang_err_thresh is not None:
        mapping.angErrThresh = ang_err_thresh
    mapping.cmdTorqueInMsg.subscribeTo(torque_msg)
    mapping.thrConfigInMsg.subscribeTo(thruster.cfg_msg)
    mapping.vehConfigInMsg.subscribeTo(bundle.veh_config_msg)
    ctx.add_fsw(mapping)
    ctx.register_tag(mapping.ModelTag, mapping)
    return mapping


def _build_firing_logic(
    ctx: SimContext,
    bundle: SpacecraftBundle,
    allocation: ThrusterTorqueAllocation,
    mapping: Any,
) -> Any:
    """Convert mapped pulse forces to thruster on-times; return the msg."""
    name = bundle.config.name
    thruster = bundle.thruster
    if thruster is None:
        raise BuildError(
            f"spacecraft {bundle.config.name!r}: thruster firing requires "
            "a thrusters effector"
        )
    if allocation.firing == "schmitt":
        firing = thrFiringSchmitt.thrFiringSchmitt()
        firing.ModelTag = f"{name}_thrFiring"
        firing.thrMinFireTime = allocation.min_fire_time
        firing.level_on = allocation.level_on
        firing.level_off = allocation.level_off
        firing.thrConfInMsg.subscribeTo(thruster.cfg_msg)
        firing.thrForceInMsg.subscribeTo(mapping.thrForceCmdOutMsg)
        apply_params(firing, allocation.params, f"firing {firing.ModelTag!r}")
        ctx.add_fsw(firing)
        ctx.register_tag(firing.ModelTag, firing)
        bundle.fsw["thruster_firing"] = firing
        return firing.onTimeOutMsg
    firing = thrFiringRound.ThrFiringRound()
    firing.ModelTag = f"{name}_thrFiring"
    devices = list(thruster.factory.thrusterList.values())
    firing.setNumThrusters(len(devices))
    firing.setThrForceMax([float(device.u_max) for device in devices])
    firing.setThrMinFireTime(allocation.min_fire_time)
    firing.setControlPeriodSec(ctx.config.simulation.dt.fsw)
    firing.setOnTimeResolutionSec(allocation.min_fire_time)
    firing.thrForceInMsg.subscribeTo(mapping.thrForceCmdOutMsg)
    ctx.add_fsw(firing)
    ctx.register_tag(firing.ModelTag, firing)
    bundle.fsw["thruster_firing"] = firing
    return firing.thrOnTimeOutMsg


def _require_mtb_torque_path(ctx: SimContext, bundle: SpacecraftBundle) -> Any:
    """Validate that the vehicle can produce magnetic dipole commands."""
    if bundle.mtb is None:
        raise BuildError(
            f"spacecraft {bundle.config.name!r}: dipole commands require an "
            "mtb effector"
        )
    return bundle.mtb


def _body_field_msg(ctx: SimContext, bundle: SpacecraftBundle) -> Any:
    """Return the local magnetic field in body frame for dipole steering.

    Builds an internal magnetometer + ``tamComm`` pair fed from the
    vehicle's environment field message (the pattern used by
    ``scenarioMtbMomentumManagementSimple``).
    """
    field_msg = ctx.env_msgs.get((bundle.config.name, MAGNETIC_FIELD))
    if field_msg is None:
        raise BuildError(
            f"spacecraft {bundle.config.name!r}: dipole commands need an "
            "[environment.magnetic_field] model (field drives the torque "
            "conversion)"
        )
    tam = magnetometer.Magnetometer()
    tam.ModelTag = f"{bundle.config.name}_mtbFieldSensor"
    tam.magInMsg.subscribeTo(field_msg)
    tam.stateInMsg.subscribeTo(bundle.sc.scStateOutMsg)
    ctx.add_fsw(tam)
    ctx.register_tag(tam.ModelTag, tam)
    comm = tamComm.tamComm()
    comm.ModelTag = f"{bundle.config.name}_mtbFieldComm"
    # the internal TAM shares the body frame; dcm_BS defaults to zeros
    comm.dcm_BS = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    comm.tamInMsg.subscribeTo(tam.tamDataOutMsg)
    ctx.add_fsw(comm)
    ctx.register_tag(comm.ModelTag, comm)
    return comm.tamOutMsg


def _default_dipole_steering(bundle: SpacecraftBundle) -> list[float]:
    """Least-squares steering matrix (N x 3, row-major) for the MTB set.

    Maps a body-frame dipole request to per-bar dipoles via the
    pseudo-inverse of the bar-axis matrix; ``dipoleMapping``'s own default
    is a zero matrix, so an explicit steering is required.
    """
    bars = [
        bar
        for effector in bundle.config.effectors
        if effector.type == "mtb"
        for bar in effector.bars
    ]
    axes = [bar.direction_B for bar in bars]
    normed = [
        [float(component) / np.linalg.norm(axis) for component in axis] for axis in axes
    ]
    steering = np.linalg.pinv(np.array(normed).T)
    return [float(value) for value in steering.flatten()]


def _build_dipole_chain(
    ctx: SimContext, bundle: SpacecraftBundle, torque_msg: Any
) -> Any:
    """Chain ``torque2Dipole`` + ``dipoleMapping`` onto a torque request."""
    mtb = bundle.mtb
    if mtb is None:
        raise BuildError(
            f"spacecraft {bundle.config.name!r}: dipole commands require an "
            "mtb effector"
        )
    t2d = torque2Dipole.torque2Dipole()
    t2d.ModelTag = f"{bundle.config.name}_torque2Dipole"
    t2d.tauRequestInMsg.subscribeTo(torque_msg)
    t2d.tamSensorBodyInMsg.subscribeTo(_body_field_msg(ctx, bundle))
    ctx.add_fsw(t2d)
    ctx.register_tag(t2d.ModelTag, t2d)
    mapping = dipoleMapping.dipoleMapping()
    mapping.ModelTag = f"{bundle.config.name}_dipoleMapping"
    mapping.dipoleRequestBodyInMsg.subscribeTo(t2d.dipoleRequestOutMsg)
    mapping.mtbArrayConfigParamsInMsg.subscribeTo(mtb.params_msg)
    mapping.steeringMatrix = _default_dipole_steering(bundle)
    ctx.add_fsw(mapping)
    ctx.register_tag(mapping.ModelTag, mapping)
    bundle.fsw["dipole_mapping"] = mapping
    return mapping.dipoleRequestMtbOutMsg


def _build_momentum_management(
    ctx: SimContext,
    bundle: SpacecraftBundle,
    management: MomentumManagementConfig,
) -> None:
    """Build wheel-momentum dumping through thrusters or MTBs."""
    if bundle.rw is None:
        raise BuildError(
            f"spacecraft {bundle.config.name!r}: momentum management requires "
            "a reactionWheels effector"
        )
    if management.type == "thrMomentumDumping":
        thruster = bundle.thruster
        if thruster is None:
            raise BuildError(
                f"spacecraft {bundle.config.name!r}: thrMomentumDumping "
                "requires a thrusters effector"
            )
        if thruster.cmd_src_msg is not None:
            raise BuildError(
                f"spacecraft {bundle.config.name!r}: thrMomentumDumping and "
                "the thrusterTorque allocation both drive the thruster "
                "command input; keep the allocation on rwMotorTorque"
            )
        desat = thrMomentumManagement.thrMomentumManagement()
        desat.ModelTag = f"{bundle.config.name}_thrMomentumManagement"
        desat.hs_min = management.hs_min
        desat.rwSpeedsInMsg.subscribeTo(bundle.rw.effector.rwSpeedOutMsg)
        desat.rwConfigDataInMsg.subscribeTo(bundle.rw.cfg_msg)
        # ``thrMomentumManagement`` computes delta-H only once per reset,
        # and ``thrMomentumDumping`` ignores requests time-stamped at zero;
        # periodically re-arming closes the dumping loop after start-up.
        rearm = _DumpRearm(desat, management.rearm_period)
        rearm.ModelTag = f"{bundle.config.name}_thrDumpRearm"
        ctx.add_fsw(rearm)
        ctx.add_fsw(desat)
        ctx.register_tag(desat.ModelTag, desat)
        bundle.fsw["dump_rearm"] = rearm  # keep the Python model alive
        mapping = _build_thr_force_mapping(
            ctx,
            bundle,
            f"{bundle.config.name}_thrForceMapping",
            desat.deltaHOutMsg,
            ang_err_thresh=3.15,
        )
        dump = thrMomentumDumping.thrMomentumDumping()
        dump.ModelTag = f"{bundle.config.name}_thrDump"
        dump.thrMinFireTime = management.thr_min_fire_time
        dump.maxCounterValue = management.max_counter_value
        dump.thrusterConfInMsg.subscribeTo(thruster.cfg_msg)
        dump.deltaHInMsg.subscribeTo(desat.deltaHOutMsg)
        dump.thrusterImpulseInMsg.subscribeTo(mapping.thrForceCmdOutMsg)
        thruster.effector.cmdsInMsg.subscribeTo(dump.thrusterOnTimeOutMsg)
        thruster.cmd_src_msg = dump.thrusterOnTimeOutMsg
        apply_params(dump, management.params, f"momentum dump {dump.ModelTag!r}")
        ctx.add_fsw(dump)
        ctx.register_tag(dump.ModelTag, dump)
        bundle.fsw["momentum_management"] = dump
        return
    if management.type == "mtbMomentumManagementSimple":
        mtb = _require_mtb_torque_path(ctx, bundle)
        if mtb.cmd_src_msg is not None:
            raise BuildError(
                f"spacecraft {bundle.config.name!r}: "
                "mtbMomentumManagementSimple and the dipoleMapping "
                "allocation both drive the MTB command input"
            )
        mtb_mms = mtbMomentumManagementSimple.mtbMomentumManagementSimple()
        mtb_mms.ModelTag = f"{bundle.config.name}_mtbMomentumMgmt"
        mtb_mms.Kp = management.kp
        mtb_mms.rwParamsInMsg.subscribeTo(bundle.rw.cfg_msg)
        mtb_mms.rwSpeedsInMsg.subscribeTo(bundle.rw.effector.rwSpeedOutMsg)
        apply_params(mtb_mms, management.params, f"momentum mgmt {mtb_mms.ModelTag!r}")
        ctx.add_fsw(mtb_mms)
        ctx.register_tag(mtb_mms.ModelTag, mtb_mms)
        chain = _build_dipole_chain(ctx, bundle, mtb_mms.tauMtbRequestOutMsg)
        mtb.effector.mtbCmdInMsg.subscribeTo(chain)
        mtb.cmd_src_msg = chain
        bundle.fsw["momentum_management"] = mtb_mms
        return
    raise BuildError(f"unsupported momentum management {management.type!r}")


def _build_desaturation(
    ctx: SimContext, bundle: SpacecraftBundle, desaturation: DesaturationConfig
) -> None:
    """Insert ``rwNullSpace`` between the motor-torque allocation and RWs."""
    if desaturation.type != "rwNullSpace":
        raise BuildError(f"unsupported desaturation {desaturation.type!r}")
    allocation = bundle.fsw.get("allocation")
    if bundle.rw is None or not isinstance(allocation, rwMotorTorque.rwMotorTorque):
        raise BuildError(
            f"spacecraft {bundle.config.name!r}: rwNullSpace desaturation "
            "requires an rwMotorTorque allocation with reactionWheels"
        )
    wheels = bundle.config.reaction_wheels()
    num_wheels = len(wheels.wheels) if wheels else 0
    speeds = messaging.RWSpeedMsgPayload()  # ty: ignore
    speeds.wheelSpeeds = list(desaturation.desired_speeds or [0.0] * num_wheels)
    bias_msg = messaging.RWSpeedMsg().write(speeds)
    constellation = messaging.RWConstellationMsgPayload()  # ty: ignore
    constellation.numRW = num_wheels
    constellation.reactionWheels = [
        messaging.RWConfigElementMsgPayload(  # ty: ignore
            gsHat_B=list(wheel.direction_B),
            Js=wheels.inertia if wheels else 0.0,
            uMax=wheel.max_torque if wheel.max_torque is not None else 1e10,
        )
        for wheel in (wheels.wheels if wheels else [])
    ]
    constellation_msg = messaging.RWConstellationMsg().write(constellation)
    null_space = rwNullSpace.rwNullSpace()
    null_space.ModelTag = f"{bundle.config.name}_rwNullSpace"
    null_space.OmegaGain = desaturation.omega_gain
    if desaturation.tau is not None:
        null_space.tau = desaturation.tau
    null_space.rwMotorTorqueInMsg.subscribeTo(allocation.rwMotorTorqueOutMsg)
    null_space.rwSpeedsInMsg.subscribeTo(bundle.rw.effector.rwSpeedOutMsg)
    null_space.rwConfigInMsg.subscribeTo(constellation_msg)
    null_space.rwDesiredSpeedsInMsg.subscribeTo(bias_msg)
    apply_params(null_space, desaturation.params, f"desat {null_space.ModelTag!r}")
    bundle.rw.effector.rwMotorCmdInMsg.subscribeTo(null_space.rwMotorTorqueOutMsg)
    ctx.add_fsw(null_space)
    ctx.register_tag(null_space.ModelTag, null_space)
    bundle.fsw["desaturation"] = null_space
