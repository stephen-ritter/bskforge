#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Flight-software stack models: ``[spacecraft.fsw]``.

The FSW section is declarative: it names a *guidance*, *estimator*,
*control* and *allocation* module by ``type``, and the builder wires the
standard Basilisk message graph between them. Each block exposes a
``params`` table for module-specific knobs (matched against the installed
Basilisk module at build time).
"""

from __future__ import annotations

from typing import Annotated, Literal

from pydantic import Field, model_validator

from bskforge.schema.base import PassthroughModel, Vec3


# --- Guidance --------------------------------------------------------------
class Inertial3DGuidance(PassthroughModel):
    """Inertial pointing reference: ``type = "inertial3D"``.

    Attributes:
        sigma_R0N: MRP of the reference frame relative to inertial.
    """

    type: Literal["inertial3D"] = "inertial3D"
    sigma_R0N: Vec3 = [0.0, 0.0, 0.0]


class VelocityPointGuidance(PassthroughModel):
    """Velocity (ram) pointing: ``type = "velocityPoint"``.

    Points the mission axis along the velocity vector of the orbit.
    """

    type: Literal["velocityPoint"] = "velocityPoint"
    mu: float | None = Field(
        None,
        gt=0,
        description="Central-body gravitational parameter "
        "[m^3/s^2]; taken from the world's central body when unset",
    )


class HillPointGuidance(PassthroughModel):
    """LVLH (nadir) pointing: ``type = "hillPoint"``."""

    type: Literal["hillPoint"] = "hillPoint"


class SunSafePointGuidance(PassthroughModel):
    """Sun-safe pointing: ``type = "sunSafePoint"``.

    Steers ``sun_axis_B`` at the sun, optionally spinning about the
    sunline. The sun direction comes from the vehicle's navigation state
    and the ``[world]`` sun ephemeris; the module output feeds the control
    law directly (it emits a tracking error, not a reference frame).
    """

    type: Literal["sunSafePoint"] = "sunSafePoint"
    sun_axis_B: Vec3 = [1.0, 0.0, 0.0]
    spin_rate: float = Field(
        0.0, description="Spin about the sunline [rad/s] when the sun is not visible"
    )


class LocationPointingGuidance(PassthroughModel):
    """Ground/body/target pointing: ``type = "locationPointing"``.

    Exactly one target: ``target_body`` (another gravity body),
    ``target_latlon`` (a point fixed on the central body, requiring
    SPICE planet orientation) or ``target_position_N`` (a fixed inertial
    position).
    """

    type: Literal["locationPointing"] = "locationPointing"
    target_body: str | None = None
    target_latlon: list[float] | None = Field(
        None,
        description="[latitude, longitude, elevation] [deg, deg, m] "
        "on the central body",
    )
    target_position_N: Vec3 | None = None
    point_axis_B: Vec3 = [1.0, 0.0, 0.0]
    scan_axis_B: Vec3 = [0.0, 1.0, 0.0]

    @model_validator(mode="after")
    def _one_target(self) -> LocationPointingGuidance:
        targets = [
            self.target_body is not None,
            self.target_latlon is not None,
            self.target_position_N is not None,
        ]
        if sum(targets) != 1:
            raise ValueError(
                "locationPointing needs exactly one of target_body, "
                "target_latlon or target_position_N"
            )
        if self.target_latlon is not None and len(self.target_latlon) != 3:
            raise ValueError("target_latlon is [lat_deg, lon_deg, elev_m]")
        return self


class CelestialTwoBodyPointGuidance(PassthroughModel):
    """Two-body constrained pointing: ``type = "celestialTwoBodyPoint"``.

    Points at ``target_body`` while keeping the ``constraint_body`` (the
    sun by default) in a safe cone; both bodies need SPICE ephemerides.
    """

    type: Literal["celestialTwoBodyPoint"] = "celestialTwoBodyPoint"
    target_body: str
    constraint_body: str = "sun"


class WaypointReferenceGuidance(PassthroughModel):
    """File-driven attitude timeline: ``type = "waypointReference"``.

    Attributes:
        waypoint_file: CSV/TXT with columns time, attitude, rate and
            angular acceleration per row (relative paths resolve against
            the scenario file's directory).
        delimiter: Column separator.
        attitude_type: 0 = MRP, 1 = quaternion (w, x, y, z),
            2 = quaternion (x, y, z, w).
        reference_frame: Rates/accelerations expressed in the reference
            frame (True) or the inertial frame (False).
        header_lines: Rows to skip at the top of the file.
    """

    type: Literal["waypointReference"] = "waypointReference"
    waypoint_file: str
    delimiter: str = ","
    attitude_type: Literal[0, 1, 2] = 0
    reference_frame: bool = False
    header_lines: int = Field(0, ge=0)


GuidanceConfig = Annotated[
    Inertial3DGuidance
    | VelocityPointGuidance
    | HillPointGuidance
    | SunSafePointGuidance
    | LocationPointingGuidance
    | CelestialTwoBodyPointGuidance
    | WaypointReferenceGuidance,
    Field(discriminator="type"),
]


# --- Estimator -------------------------------------------------------------
class SimpleNavEstimator(PassthroughModel):
    """Truth-pass-through navigation: ``type = "simpleNav"``."""

    type: Literal["simpleNav"] = "simpleNav"


class InertialUkfEstimator(PassthroughModel):
    """Star-tracker inertial unscented Kalman filter: ``type = "inertialUKF"``.

    Estimates attitude and body rate from the vehicle's star trackers
    (propagating between measurements with the vehicle inertia).

    Attributes:
        st_sensors: Star-tracker sensor names to fuse; all when empty.
        process_noise_att: Attitude process-noise sigma [rad].
        process_noise_rate: Body-rate process-noise sigma [rad/s].
        covar_init_att: Initial attitude covariance (variance).
        covar_init_rate: Initial rate covariance (variance).
        switch_mag: Measurement switch threshold on the MRP error norm.
    """

    type: Literal["inertialUKF"] = "inertialUKF"
    st_sensors: list[str] = Field(default_factory=list)
    measurement_noise: float = Field(
        0.00017, gt=0, description="ST attitude measurement sigma [rad]"
    )
    process_noise_att: float = Field(0.0017, gt=0)
    process_noise_rate: float = Field(0.00017, gt=0)
    covar_init_att: float = Field(0.04, gt=0)
    covar_init_rate: float = Field(0.0001, gt=0)
    switch_mag: float = Field(1.2, gt=0)
    alpha: float = Field(0.02, gt=0, description="UKF spread parameter")
    beta: float = Field(2.0, gt=0, description="UKF prior distribution knob")
    kappa: float = Field(0.0, ge=0, description="UKF secondary scaling")


class CssWlsEstEstimator(PassthroughModel):
    """CSS weighted-least-squares sunline estimator: ``type = "cssWlsEst"``."""

    type: Literal["cssWlsEst"] = "cssWlsEst"
    cluster: str = "css"
    use_weights: bool | None = None


class SunlineSuKFEstimator(PassthroughModel):
    """CSS square-root UKF sunline estimator: ``type = "sunlineSuKF"``."""

    type: Literal["sunlineSuKF"] = "sunlineSuKF"
    cluster: str = "css"
    q_obs: float | None = Field(None, gt=0, description="Measurement noise")
    output_sunline: bool = True


EstimatorConfig = Annotated[
    SimpleNavEstimator
    | InertialUkfEstimator
    | CssWlsEstEstimator
    | SunlineSuKFEstimator,
    Field(discriminator="type"),
]


# --- Control ---------------------------------------------------------------
class MrpFeedbackControl(PassthroughModel):
    """MRP feedback attitude control: ``type = "mrpFeedback"``.

    Attributes:
        k: Proportional gain on the attitude error MRP.
        p: Proportional gain on the rate error.
        ki: Integral gain (negative disables integral action).
        integral_limit: Integral-term saturation; auto-derived when unset.
        known_torque: Feed-forward torque cancellation ``knownTorquePntB_B``.
    """

    type: Literal["mrpFeedback"] = "mrpFeedback"
    k: float = 3.5
    p: float = 30.0
    ki: float = -1.0
    integral_limit: float | None = Field(None, gt=0)
    known_torque: Vec3 = [0.0, 0.0, 0.0]

    def resolved_integral_limit(self) -> float:
        """Return the explicit limit, or derive it from ``ki``."""
        if self.integral_limit is not None:
            return self.integral_limit
        if self.ki < 0:
            return 0.0
        return 2.0 / self.ki * 0.1


class MrpPdControl(PassthroughModel):
    """MRP + rate PD control: ``type = "mrpPD"``.

    A pure PD law (no integral action) with inertia-weighted rate term.
    """

    type: Literal["mrpPD"] = "mrpPD"
    k: float = Field(3.5, description="Position gain on sigma_eb")
    p: float = Field(30.0, description="Rate gain on omega_bn")
    known_torque: Vec3 = [0.0, 0.0, 0.0]


class RateServoControl(PassthroughModel):
    """Steering + rate-servo control: ``type = "rateServo"``.

    Chains ``mrpSteering`` (generates a rate command bounded by
    ``max_rate``) into ``rateServoFullNonlinear`` (PI rate tracking with
    wheel-space decomposition when wheels are configured).
    """

    type: Literal["rateServo"] = "rateServo"
    p: float = 150.0
    ki: float = 0.01
    integral_limit: float | None = Field(None, gt=0)
    known_torque: Vec3 = [0.0, 0.0, 0.0]
    max_rate: float = Field(0.005, gt=0, description="Steering rate limit [rad/s]")
    steering_k1: float = Field(
        1.0, description="mrpSteering linear gain K1 on the error MRP"
    )
    steering_k3: float = Field(
        0.0, description="mrpSteering cubic gain K3 on the error MRP"
    )

    def resolved_integral_limit(self) -> float:
        """Return the explicit limit, or derive it from ``ki``."""
        if self.integral_limit is not None:
            return self.integral_limit
        if self.ki <= 0:
            return 0.0
        return 2.0 / self.ki * 0.1


ControlConfig = Annotated[
    MrpFeedbackControl | MrpPdControl | RateServoControl,
    Field(discriminator="type"),
]


# --- Allocation ------------------------------------------------------------
class RwMotorTorqueAllocation(PassthroughModel):
    """Map commanded torque to RW motor torques: ``type = "rwMotorTorque"``."""

    type: Literal["rwMotorTorque"] = "rwMotorTorque"


class ThrusterTorqueAllocation(PassthroughModel):
    """Map commanded torque to thruster pulses: ``type = "thrusterTorque"``.

    Uses ``thrForceMapping`` (least-squares allocation over the configured
    thruster set) followed by a firing-logic stage that converts forces to
    integer on-times.

    Attributes:
        force_sign: Sign convention between commanded and produced torque.
        ang_err_thresh: Angle-error threshold for the null-space steering
            update (radians); values > pi disable re-steering, matching
            Basilisk's momentum-dumping usage.
        firing: Pulse shaping: ``"schmitt"`` adds hysteresis around the
            force thresholds, ``"round"`` snaps on-times to a resolution.
        min_fire_time: Smallest commanded on-time for the firing stage.
        level_on: Schmitt trigger turn-on force fraction (of peak force).
        level_off: Schmitt trigger turn-off force fraction.
    """

    type: Literal["thrusterTorque"] = "thrusterTorque"
    force_sign: Literal[-1, 1] = 1
    ang_err_thresh: float | None = None
    firing: Literal["schmitt", "round"] = "schmitt"
    min_fire_time: float = Field(0.002, gt=0, description="Min on-time [s]")
    level_on: float = Field(0.75, gt=0, le=1)
    level_off: float = Field(0.25, ge=0, lt=1)


class DipoleMappingAllocation(PassthroughModel):
    """Map commanded torque to MTB dipoles: ``type = "dipoleMapping"``.

    Chains ``torque2Dipole`` (needs the field vector in the body frame, so
    an ``[environment.magnetic_field]`` model is required) with
    ``dipoleMapping``.

    Attributes:
        steering_matrix: row-major N-by-3 steering matrix of
            ``dipoleMapping`` (N torque bars); the pseudo-inverse of the
            bar axes when unset.
    """

    type: Literal["dipoleMapping"] = "dipoleMapping"
    steering_matrix: list[float] | None = Field(None, description="9 entries")


AllocationConfig = Annotated[
    RwMotorTorqueAllocation | ThrusterTorqueAllocation | DipoleMappingAllocation,
    Field(discriminator="type"),
]


# --- Momentum management -----------------------------------------------------
class ThrMomentumDumping(PassthroughModel):
    """Dump RW momentum through the thrusters: ``type = "thrMomentumDumping"``.

    Chains ``thrMomentumManagement`` (wheel speeds → desired delta-H),
    ``thrForceMapping`` (delta-H → pulse forces) and
    ``thrMomentumDumping`` (rate-limits actual firings). Requires
    ``reactionWheels`` and ``thrusters`` effectors; it takes over the
    thruster command path.

    Attributes:
        hs_min: Wheel momentum magnitude per axis below which no dump
            torque is commanded.
        thr_min_fire_time: Smallest commanded thruster on-time.
        max_counter_value: FSW periods to wait between subsequent firings.
        rearm_period: Seconds between re-evaluations of the dump request;
            ``thrMomentumManagement`` computes its delta-H once per reset,
            so bskforge periodically re-arms it to close the loop.
    """

    type: Literal["thrMomentumDumping"] = "thrMomentumDumping"
    hs_min: float = Field(0.0, ge=0, description="Momentum threshold [N*m*s]")
    thr_min_fire_time: float = Field(0.02, gt=0, description="Min on-time [s]")
    max_counter_value: int = Field(100, gt=0)
    rearm_period: float = Field(10.0, gt=0, description="Re-evaluation [s]")


class MtbMomentumManagementSimpleMM(PassthroughModel):
    """Dump RW momentum through MTBs: ``type = "mtbMomentumManagementSimple"``.

    Requires ``reactionWheels`` and ``mtb`` effectors plus an
    ``[environment.magnetic_field]`` model; the dipole commands go through
    the same ``torque2Dipole``/``dipoleMapping`` chain as the
    ``dipoleMapping`` allocation.

    Attributes:
        kp: Proportional gain on the wheel-momentum error.
    """

    type: Literal["mtbMomentumManagementSimple"] = "mtbMomentumManagementSimple"
    kp: float = Field(0.003, gt=0)


MomentumManagementConfig = Annotated[
    ThrMomentumDumping | MtbMomentumManagementSimpleMM,
    Field(discriminator="type"),
]


# --- Desaturation ------------------------------------------------------------
class RwNullSpaceDesat(PassthroughModel):
    """Wheel-speed null-space steering: ``type = "rwNullSpace"``.

    Inserts ``rwNullSpace`` between the motor-torque allocation and the
    wheel effector; requires a ``rwMotorTorque`` allocation.

    Attributes:
        omega_gain: Speed-error feedback gain.
        tau: Null-space projection time constant.
        desired_speeds: Per-wheel bias speeds (rad/s); zeros when unset.
    """

    type: Literal["rwNullSpace"] = "rwNullSpace"
    omega_gain: float = Field(3e-7, gt=0)
    tau: float | None = None
    desired_speeds: list[float] = Field(default_factory=list)


DesaturationConfig = Annotated[RwNullSpaceDesat, Field(discriminator="type")]


class FswSection(PassthroughModel):
    """``[spacecraft.fsw]``: the per-vehicle flight-software stack.

    Attributes:
        guidance: Attitude reference generator.
        estimator: Navigation/attitude estimate feeding the control law.
        control: Attitude control law producing a commanded body torque.
        allocation: Maps the commanded torque onto effector device commands.
        momentum_management: Reaction-wheel momentum dumping through
            thrusters or MTBs.
        desaturation: Wheel-speed null-space steering stage.
    """

    guidance: GuidanceConfig | None = None
    estimator: EstimatorConfig | None = None
    control: ControlConfig | None = None
    allocation: AllocationConfig | None = None
    momentum_management: MomentumManagementConfig | None = None
    desaturation: DesaturationConfig | None = None
