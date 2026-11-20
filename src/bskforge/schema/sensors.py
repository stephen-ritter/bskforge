#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Sensor models: ``[[spacecraft.sensors]]``."""

from __future__ import annotations

from typing import Annotated, Literal

from pydantic import Field

from bskforge.schema.base import PassthroughModel, Vec3
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
        noise_sigma: Attitude measurement noise standard deviation.
    """

    type: Literal["starTracker"] = "starTracker"
    name: str = "st"
    sigma_SB: Vec3 = [0.0, 0.0, 0.0]
    noise_sigma: float | None = Field(
        None, ge=0, description="Attitude noise sigma [rad]"
    )
    recorder: RecorderConfig = Field(default_factory=RecorderConfig)


class ImuSensorConfig(PassthroughModel):
    """Inertial measurement unit: ``type = "imuSensor"``.

    Attributes:
        name: Sensor ModelTag; unique within the vehicle.
        sigma_PB: MRP orientation of platform frame relative to body frame.
        gyro_bias: Gyro output bias.
        accel_bias: Accelerometer output bias.
    """

    type: Literal["imuSensor"] = "imuSensor"
    name: str = "imu"
    sigma_PB: Vec3 = [0.0, 0.0, 0.0]
    gyro_bias: Vec3 = [0.0, 0.0, 0.0]
    accel_bias: Vec3 = [0.0, 0.0, 0.0]
    recorder: RecorderConfig = Field(default_factory=RecorderConfig)


class MagnetometerConfig(PassthroughModel):
    """Magnetometer (magnetic-field wiring lands in Phase 2)."""

    type: Literal["magnetometer"] = "magnetometer"
    name: str = "magnetometer"
    sigma_SB: Vec3 = [0.0, 0.0, 0.0]
    recorder: RecorderConfig = Field(default_factory=RecorderConfig)


class CameraConfig(PassthroughModel):
    """Camera (imaging pipeline lands in Phase 3)."""

    type: Literal["camera"] = "camera"
    name: str = "camera"
    sigma_CB: Vec3 = [0.0, 0.0, 1.0]
    recorder: RecorderConfig = Field(default_factory=RecorderConfig)


SensorConfig = Annotated[
    CoarseSunSensorConfig
    | StarTrackerConfig
    | ImuSensorConfig
    | MagnetometerConfig
    | CameraConfig,
    Field(discriminator="type"),
]
"""Discriminated sensor union for ``[[spacecraft.sensors]]``."""
