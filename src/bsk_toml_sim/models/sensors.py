#
#  ISC License
#
#  Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for
#  any purpose with or without fee is hereby granted, provided that the
#  above copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

"""Sensor models for Basilisk TOML simulations."""

from pydantic import BaseModel, Field

from bsk_toml_sim.models.recorder import RecorderConfig


class CameraConfig(BaseModel):
    """Camera sensor configuration.

    Models the camera module that simulates a camera with image corruption
    effects such as Gaussian noise, blur, dark current, and cosmic rays.

    Attributes:
        name: Unique identifier for this camera.
        sigma_CB: Attitude orientation of the camera boresight in body frame.
        field_of_view: Half-angle field of view.
        image_width: Image resolution (horizontal pixels).
        image_height: Image resolution (vertical pixels).
        is_on: Whether the camera is enabled.
        gaussian: Gaussian noise scaling parameter.
        blur_param: Box blur kernel size (must be odd).
        dark_current: Dark current noise scaling factor.
        hsv_adjust: HSV color adjustment [hue_rad, sat_scale, val_scale].
        bgr_adjust: BGR color channel percentage adjustment.
        salt_pepper: Dead/stuck pixel probability scaling factor.
        cosmic_rays: Number of cosmic ray events.

    Example:
        ```toml
        [[satellite.sensors.cameras]]
        name = "imagingCam"
        sigma_CB = [0, 0, 1]
        field_of_view = 30.0
        gaussian = 2.0
        blur_param = 3
        dark_current = 1.0
        ```
    """

    name: str = "camera"
    sigma_CB: list[float] = Field(
        default_factory=lambda: [0.0, 0.0, 1.0],
        description="Camera boresight MRP sigma_CB [-]",
    )
    field_of_view: float = Field(30.0, description="Half-angle FOV [deg]")
    image_width: int = Field(640, description="Image width [pixels]")
    image_height: int = Field(480, description="Image height [pixels]")
    is_on: bool = True
    gaussian: float = Field(0.0, description="Gaussian noise scale [-]")
    blur_param: int = Field(0, description="Box blur kernel size (odd) [-]")
    dark_current: float = Field(0.0, description="Dark current scale [-]")
    hsv_adjust: list[float] = Field(
        default_factory=lambda: [0.0, 0.0, 0.0],
        description="HSV adjust [rad, sat_scale, val_scale]",
    )
    bgr_adjust: list[int] = Field(
        default_factory=lambda: [0, 0, 0],
        description="BGR percent adjustment [%]",
    )
    salt_pepper: float = Field(0.0, description="Salt/pepper scale [-]")
    cosmic_rays: float = Field(0.0, description="Cosmic ray count [-]")
    recorder: RecorderConfig = Field(
        default_factory=RecorderConfig,
        description="Recorder for camera output message",
    )


class SunSensorConfig(BaseModel):
    """Coarse sun sensor configuration.

    Attributes:
        name: Unique identifier for this sun sensor.
        sigma_SB: Attitude orientation of the sensor in body frame (MRP).
        sampling_time: Sensor output period.

    Example:
        ```toml
        [[satellite.sensors.sun_sensors]]
        name = "sunSensor1"
        sigma_SB = [0, 0, 1]
        sampling_time = 1.0
        ```
    """

    name: str = "sunSensor"
    sigma_SB: list[float] = Field(
        default_factory=lambda: [0.0, 0.0, 1.0],
        description="Sensor orientation MRP sigma_SB [-]",
    )
    sampling_time: float = Field(1.0, description="Sampling period [s]")
    recorder: RecorderConfig = Field(
        default_factory=RecorderConfig,
        description="Recorder for sun sensor output message",
    )


class MagnetometerConfig(BaseModel):
    """Magnetometer sensor configuration.

    Attributes:
        name: Unique identifier for this magnetometer.
        bias: Sensor bias on the magnetic field measurement.
        noise_sigma: Measurement noise standard deviation.
        sampling_time: Sensor output period.

    Example:
        ```toml
        [[satellite.sensors.magnetometers]]
        name = "magSensor"
        bias = [0, 0, 0]
        noise_sigma = 0.1
        sampling_time = 1.0
        ```
    """

    name: str = "magnetometer"
    bias: list[float] = Field(
        default_factory=lambda: [0.0, 0.0, 0.0],
        description="Sensor bias [nT]",
    )
    noise_sigma: float = Field(
        0.1, description="Noise standard deviation [nT]"
    )
    sampling_time: float = Field(1.0, description="Sampling period [s]")
    recorder: RecorderConfig = Field(
        default_factory=RecorderConfig,
        description="Recorder for magnetometer output message",
    )


class StarTrackerConfig(BaseModel):
    """Star tracker sensor configuration.

    Attributes:
        name: Unique identifier for this star tracker.
        sigma_SB: Attitude orientation of the sensor in body frame (MRP).
        noise_sigma: Attitude measurement noise standard deviation.
        bias: Attitude measurement bias (MRP).
        sampling_time: Sensor output period.

    Example:
        ```toml
        [[satellite.sensors.star_trackers]]
        name = "starTracker1"
        sigma_SB = [0, 0, 1]
        noise_sigma = 0.001
        sampling_time = 2.0
        ```
    """

    name: str = "starTracker"
    sigma_SB: list[float] = Field(
        default_factory=lambda: [0.0, 0.0, 1.0],
        description="Sensor orientation MRP sigma_SB [-]",
    )
    noise_sigma: float = Field(0.001, description="Attitude noise sigma [rad]")
    bias: list[float] = Field(
        default_factory=lambda: [0.0, 0.0, 0.0],
        description="Attitude bias MRP [-]",
    )
    sampling_time: float = Field(2.0, description="Sampling period [s]")
    recorder: RecorderConfig = Field(
        default_factory=RecorderConfig,
        description="Recorder for star tracker output message",
    )


class IMUConfig(BaseModel):
    """Inertial measurement unit configuration.

    Attributes:
        name: Unique identifier for this IMU.
        sigma_SB: Attitude orientation of the IMU in body frame (MRP).
        gyro_noise_sigma: Gyroscope measurement noise standard deviation.
        gyro_bias: Gyroscope bias.
        sampling_time: Sensor output period.

    Example:
        ```toml
        [[satellite.sensors.imus]]
        name = "imu"
        sigma_SB = [0, 0, 0]
        gyro_noise_sigma = 0.0001
        sampling_time = 0.01
        ```
    """

    name: str = "imu"
    sigma_SB: list[float] = Field(
        default_factory=lambda: [0.0, 0.0, 0.0],
        description="IMU orientation MRP sigma_SB [-]",
    )
    gyro_noise_sigma: float = Field(
        1e-4, description="Gyro noise sigma [rad/s]"
    )
    gyro_bias: list[float] = Field(
        default_factory=lambda: [0.0, 0.0, 0.0],
        description="Gyro bias [rad/s]",
    )
    sampling_time: float = Field(0.01, description="Sampling period [s]")
    recorder: RecorderConfig = Field(
        default_factory=RecorderConfig,
        description="Recorder for IMU output message",
    )


class SensorConfig(BaseModel):
    """Container for all sensors on a spacecraft.

    Attributes:
        cameras: List of camera sensors.
        sun_sensors: List of coarse sun sensors.
        magnetometers: List of magnetometer sensors.
        star_trackers: List of star tracker sensors.
        imus: List of inertial measurement units.

    Example:
        ```toml
        [satellite.sensors]

        [[satellite.sensors.cameras]]
        name = "imagingCam"
        sigma_CB = [0, 0, 1]
        field_of_view = 30.0

        [[satellite.sensors.sun_sensors]]
        name = "sunSensor1"
        sigma_SB = [0, 0, 1]

        [[satellite.sensors.magnetometers]]
        name = "magSensor"
        noise_sigma = 0.1
        ```
    """

    cameras: list[CameraConfig] = Field(default_factory=list)
    sun_sensors: list[SunSensorConfig] = Field(default_factory=list)
    magnetometers: list[MagnetometerConfig] = Field(default_factory=list)
    star_trackers: list[StarTrackerConfig] = Field(default_factory=list)
    imus: list[IMUConfig] = Field(default_factory=list)