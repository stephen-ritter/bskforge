#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Sensor models: ``[[spacecraft.sensors]]`` and CSS constellations."""

from __future__ import annotations

from typing import Annotated, Literal

from pydantic import Field, model_validator

from bskforge.schema.base import PassthroughModel, ScenarioModel, Vec3
from bskforge.schema.recorder import RecorderConfig


class CoarseSunSensorConfig(PassthroughModel):
    """Coarse sun sensor: ``type = "coarseSunSensor"``.

    Attributes:
        name: Sensor ModelTag; unique within the vehicle.
        boresight_B: Unit boresight direction ``nHat_B`` in the body frame.
        fov_deg: Half-angle field of view.
        noise_sigma: White-noise standard deviation on the cosine output.
        bias: Additive sensor bias on the cosine output.
    """

    type: Literal["coarseSunSensor"] = "coarseSunSensor"
    name: str = "css"
    boresight_B: Vec3 = [1.0, 0.0, 0.0]
    fov_deg: float = Field(30.0, gt=0, description="Half-angle FOV [deg]")
    noise_sigma: float | None = Field(None, ge=0, description="Output noise sigma [-]")
    bias: float | None = Field(None, description="Output bias [-]")
    recorder: RecorderConfig = Field(default_factory=RecorderConfig)


class StarTrackerConfig(PassthroughModel):
    """Star tracker: ``type = "starTracker"``.

    Attributes:
        name: Sensor ModelTag; unique within the vehicle.
        sigma_SB: MRP orientation of sensor frame relative to body frame.
        noise_sigma: Attitude random-walk standard deviation [rad].
        white_noise: Per-axis measurement noise standard deviation [rad],
            written into the sensor ``PMatrix``.
    """

    type: Literal["starTracker"] = "starTracker"
    name: str = "st"
    sigma_SB: Vec3 = [0.0, 0.0, 0.0]
    noise_sigma: float | None = Field(None, ge=0, description="Random-walk sigma [rad]")
    white_noise: float | None = Field(None, ge=0, description="White-noise sigma [rad]")
    recorder: RecorderConfig = Field(default_factory=RecorderConfig)


class ImuSensorConfig(PassthroughModel):
    """Inertial measurement unit: ``type = "imuSensor"``.

    Attributes:
        name: Sensor ModelTag; unique within the vehicle.
        sigma_PB: MRP orientation of platform frame relative to body frame.
        gyro_bias: Gyro output bias [rad/s].
        accel_bias: Accelerometer output bias [m/s^2].
        gyro_walk: Gyro random-walk bounds [rad/s^1.5].
        accel_walk: Accelerometer random-walk bounds [m/s^2.5].
    """

    type: Literal["imuSensor"] = "imuSensor"
    name: str = "imu"
    sigma_PB: Vec3 = [0.0, 0.0, 0.0]
    gyro_bias: Vec3 = [0.0, 0.0, 0.0]
    accel_bias: Vec3 = [0.0, 0.0, 0.0]
    gyro_walk: Vec3 | None = Field(None, description="Gyro random walk")
    accel_walk: Vec3 | None = Field(None, description="Accel random walk")
    recorder: RecorderConfig = Field(default_factory=RecorderConfig)


class MagnetometerConfig(PassthroughModel):
    """Magnetometer: ``type = "magnetometer"``.

    Requires an ``[environment.magnetic_field]`` model to measure.

    Attributes:
        name: Sensor ModelTag; unique within the vehicle.
        sigma_SB: MRP orientation of sensor frame relative to body frame.
        noise_sigma: White-noise standard deviation per axis [T].
        bias: Additive sensor bias [T].
        saturate: Clamp output to ``[minOutput, maxOutput]``.
    """

    type: Literal["magnetometer"] = "magnetometer"
    name: str = "magnetometer"
    sigma_SB: Vec3 = [0.0, 0.0, 0.0]
    noise_sigma: float | None = Field(None, ge=0, description="Noise sigma [T]")
    bias: Vec3 | None = Field(None, description="Sensor bias [T]")
    saturate: bool = False
    recorder: RecorderConfig = Field(default_factory=RecorderConfig)


class GpsReceiverConfig(PassthroughModel):
    """GNSS receiver: ``type = "gpsReceiver"``.

    The plant's opinion of position and velocity: Basilisk's native
    ``SimpleNav`` superimposes an 18-state Gauss-Markov error process on the
    true translational state and writes the estimate to ``navTransMsg``.
    The declared sigmas set the noise strength of the position and velocity
    states (three-sigma safety bounds on the draws, per Basilisk's own unit
    tests the error can never leave); the remaining states stay silent: a
    receiver fixes the orbit, not the attitude. The fixed default seed keeps
    the noise realization independent of the vehicle's name: renaming a
    spacecraft must never re-draw its instrument's luck.

    Attributes:
        name: Sensor ModelTag; unique within the vehicle.
        pos_sigma_m: Position error scale [m].
        vel_sigma_m_s: Velocity error scale [m/s].
        rng_seed: Basilisk RNG seed for the error process (non-zero).
    """

    type: Literal["gpsReceiver"] = "gpsReceiver"
    name: str = "gnss"
    pos_sigma_m: float = Field(15.0, gt=0, description="Position error sigma [m]")
    vel_sigma_m_s: float = Field(0.01, gt=0, description="Velocity error sigma [m/s]")
    rng_seed: int = Field(0x475053, ge=1, description="Error-process RNG seed")
    recorder: RecorderConfig = Field(default_factory=RecorderConfig)


class LandmarkConfig(ScenarioModel):
    """A fixed landmark on the camera's target body.

    Attributes:
        position: Landmark position in the target-centered frame [m].
        normal: Outward surface normal in the target frame (unit).
    """

    position: Vec3
    normal: Vec3 = [1.0, 0.0, 0.0]


class CameraConfig(PassthroughModel):
    """Pinhole landmark camera: ``type = "camera"``.

    Ships with the optional ``bsk-opnav`` wheel; building one without it
    raises a clear error. The target's ephemeris comes from ``[world.spice]``
    (landmark lighting is computed from the real sun line), so the target
    must also appear in ``[[world.gravities]]``. The field of view follows
    from the optics: ``2 * atan(nx * pixel_size / (2 * focal_length))``.

    Attributes:
        name: Sensor ModelTag; unique within the vehicle.
        sigma_CB: MRP orientation of camera frame relative to body frame.
        focal_length: Focal length ``f`` [m].
        pixel_size: Square pixel pitch ``wPixel`` [m].
        resolution: Image size in pixels ``[nx, ny]``.
        target: Gravity-body name; defaults to the central body.
        landmarks: Landmarks projected into the image.
        sun_mask_angle: Minimum solar incidence angle for a landmark to be
            considered lit, radians; the Basilisk default (``-pi/2``) accepts
            any illuminated surface.
    """

    type: Literal["camera"] = "camera"
    name: str = "camera"
    sigma_CB: Vec3 = [0.0, 0.0, 0.0]
    focal_length: float = Field(25.0e-3, gt=0, description="Focal length f [m]")
    pixel_size: float = Field(11.0e-6, gt=0, description="Pixel pitch [m]")
    resolution: list[int] = Field(default_factory=lambda: [1024, 1024])
    target: str | None = Field(None, description="Target body name")
    landmarks: list[LandmarkConfig] = Field(default_factory=list)
    sun_mask_angle: float | None = Field(None, description="maskSun [rad]")
    recorder: RecorderConfig = Field(default_factory=RecorderConfig)

    @model_validator(mode="after")
    def _check_resolution(self) -> CameraConfig:
        if len(self.resolution) != 2 or any(int(n) <= 0 for n in self.resolution):
            raise ValueError("camera resolution must be [nx, ny] with positive pixels")
        return self


class CssConstellationConfig(ScenarioModel):
    """Aggregates several CSS sensors into one sunline measurement.

    Feeds the ``cssWlsEst`` / ``sunlineSuKF`` estimators through
    ``[spacecraft.fsw.estimator]``.

    Attributes:
        name: Handle referenced by estimators.
        members: ``[[spacecraft.sensors]]`` names of the coarse sun sensors.
    """

    name: str = "css"
    members: list[str] = Field(min_length=2)


SensorConfig = Annotated[
    CoarseSunSensorConfig
    | StarTrackerConfig
    | ImuSensorConfig
    | MagnetometerConfig
    | GpsReceiverConfig
    | CameraConfig,
    Field(discriminator="type"),
]
"""Discriminated sensor union for ``[[spacecraft.sensors]]``."""
