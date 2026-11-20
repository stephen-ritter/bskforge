#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Sensor build phase: CSS, star tracker, IMU (mag/camera land later)."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from Basilisk.simulation import (
    coarseSunSensor,
    imuSensor,
    magnetometer,
    starTracker,
)
from Basilisk.utilities import macros

from bskforge.build.context import SimContext, SpacecraftBundle
from bskforge.build.environment import ECLIPSE, MAGNETIC_FIELD
from bskforge.build.registry import apply_params, unsupported
from bskforge.errors import BuildError
from bskforge.schema.sensors import (
    CoarseSunSensorConfig,
    ImuSensorConfig,
    MagnetometerConfig,
    StarTrackerConfig,
)

SENSOR_BUILDERS: dict[str, Callable[..., None]] = {}


def sensor_builder(type_name: str) -> Callable[[Callable], Callable]:
    """Register a builder function for a sensor ``type``."""

    def _decorator(fn: Callable) -> Callable:
        SENSOR_BUILDERS[type_name] = fn
        return fn

    return _decorator


@sensor_builder("coarseSunSensor")
def _build_css(
    ctx: SimContext, bundle: SpacecraftBundle, cfg: CoarseSunSensorConfig
) -> None:
    """Instantiate a coarse sun sensor and subscribe sun + state inputs."""
    sensor = coarseSunSensor.CoarseSunSensor()
    sensor.ModelTag = f"{bundle.config.name}_{cfg.name}"
    sensor.nHat_B = list(cfg.boresight_B)
    sensor.fov = cfg.fov_deg * macros.D2R
    if cfg.noise_sigma is not None:
        sensor.senNoiseStd = cfg.noise_sigma
    if cfg.bias is not None:
        sensor.senBias = cfg.bias
    sensor.sunInMsg.subscribeTo(ctx.ensure_sun_msg())
    sensor.stateInMsg.subscribeTo(bundle.sc.scStateOutMsg)
    eclipse_msg = ctx.env_msgs.get((bundle.config.name, ECLIPSE))
    if eclipse_msg is not None:
        sensor.sunEclipseInMsg.subscribeTo(eclipse_msg)
    apply_params(sensor, cfg.params, f"sensor {sensor.ModelTag!r}")
    _finalize(ctx, bundle, sensor, cfg)


@sensor_builder("starTracker")
def _build_star_tracker(
    ctx: SimContext, bundle: SpacecraftBundle, cfg: StarTrackerConfig
) -> None:
    """Instantiate a star tracker with body-to-sensor DCM from ``sigma_SB``."""
    sensor = starTracker.StarTracker()
    sensor.ModelTag = f"{bundle.config.name}_{cfg.name}"
    sensor.dcm_CB = _sigma_to_dcm(cfg.sigma_SB)
    if cfg.noise_sigma is not None:
        sensor.setWalkBounds([cfg.noise_sigma] * 3)
    sensor.scStateInMsg.subscribeTo(bundle.sc.scStateOutMsg)
    apply_params(sensor, cfg.params, f"sensor {sensor.ModelTag!r}")
    _finalize(ctx, bundle, sensor, cfg)


@sensor_builder("imuSensor")
def _build_imu(ctx: SimContext, bundle: SpacecraftBundle, cfg: ImuSensorConfig) -> None:
    """Instantiate an IMU with platform DCM and gyro/accel biases."""
    sensor = imuSensor.ImuSensor()
    sensor.ModelTag = f"{bundle.config.name}_{cfg.name}"
    sensor.dcm_PB = _sigma_to_dcm(cfg.sigma_PB)
    sensor.senRotBias = list(cfg.gyro_bias)
    sensor.senTransBias = list(cfg.accel_bias)
    sensor.scStateInMsg.subscribeTo(bundle.sc.scStateOutMsg)
    apply_params(sensor, cfg.params, f"sensor {sensor.ModelTag!r}")
    _finalize(ctx, bundle, sensor, cfg)


@sensor_builder("magnetometer")
def _build_magnetometer(
    ctx: SimContext, bundle: SpacecraftBundle, cfg: MagnetometerConfig
) -> None:
    """Instantiate a magnetometer measuring the environment field."""
    field_msg = ctx.env_msgs.get((bundle.config.name, MAGNETIC_FIELD))
    if field_msg is None:
        raise BuildError(
            f"spacecraft {bundle.config.name!r}: sensor {cfg.name!r} "
            "requires an [environment.magnetic_field] model to measure"
        )
    sensor = magnetometer.Magnetometer()
    sensor.ModelTag = f"{bundle.config.name}_{cfg.name}"
    sensor.dcm_SB = _sigma_to_dcm(cfg.sigma_SB)
    if cfg.noise_sigma is not None:
        sensor.senNoiseStd = [cfg.noise_sigma] * 3
    if cfg.bias is not None:
        sensor.senBias = list(cfg.bias)
    sensor.applySaturation = cfg.saturate  # ty: ignore[invalid-assignment]
    sensor.magInMsg.subscribeTo(field_msg)
    sensor.stateInMsg.subscribeTo(bundle.sc.scStateOutMsg)
    apply_params(sensor, cfg.params, f"sensor {sensor.ModelTag!r}")
    _finalize(ctx, bundle, sensor, cfg)


@sensor_builder("camera")
def _build_camera(*args: Any, **kwargs: Any) -> None:
    unsupported("Camera / imaging sensors", "Phase 3")(*args, **kwargs)


def _finalize(ctx: SimContext, bundle: SpacecraftBundle, sensor: Any, cfg: Any) -> None:
    """Add the sensor to the dynamics task and register its output message."""
    ctx.add_dyn(sensor)
    ctx.register_tag(sensor.ModelTag, sensor)
    out_msg = _sensor_output_message(sensor, cfg.type)
    bundle.sensor_msgs[cfg.name] = (out_msg, cfg.recorder)


def _sensor_output_message(sensor: Any, type_name: str) -> Any:
    """Return the primary output message to record for a sensor type."""
    if type_name == "coarseSunSensor":
        return sensor.cssDataOutMsg
    if type_name in ("starTracker", "imuSensor"):
        return sensor.sensorOutMsg
    if type_name == "magnetometer":
        return sensor.tamDataOutMsg
    raise BuildError(f"no output message mapping for sensor type {type_name!r}")


def _sigma_to_dcm(sigma: list[float]) -> Any:
    """Convert an MRP orientation to a Basilisk Eigen 3x3 DCM."""
    import numpy as np
    from Basilisk.utilities import RigidBodyKinematics as rbk
    from Basilisk.utilities import simHelpers

    c = np.asarray(rbk.MRP2C(sigma), dtype=float).flatten()
    return simHelpers.np2EigenMatrix3d(c)
