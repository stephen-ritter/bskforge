#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Sensor build phase: CSS, star tracker, IMU, magnetometer, pinhole camera."""

from __future__ import annotations

import zlib
from collections.abc import Callable
from typing import Any

import numpy as np
from Basilisk.architecture import messaging
from Basilisk.simulation import (
    coarseSunSensor,
    ephemerisConverter,
    imuSensor,
    magnetometer,
    simpleNav,
    starTracker,
)
from Basilisk.utilities import macros

from bskforge.build.context import SimContext, SpacecraftBundle
from bskforge.build.environment import ECLIPSE, MAGNETIC_FIELD
from bskforge.build.registry import apply_params
from bskforge.errors import BuildError
from bskforge.schema.sensors import (
    CameraConfig,
    CoarseSunSensorConfig,
    GpsReceiverConfig,
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
    if cfg.noise_sigma is not None:
        _seed_rng(sensor)
    _finalize(ctx, bundle, sensor, cfg)


@sensor_builder("starTracker")
def _build_star_tracker(
    ctx: SimContext, bundle: SpacecraftBundle, cfg: StarTrackerConfig
) -> None:
    """Instantiate a star tracker with body-to-sensor DCM from ``sigma_SB``."""
    sensor = starTracker.StarTracker()
    sensor.ModelTag = f"{bundle.config.name}_{cfg.name}"
    sensor.dcm_CB = _sigma_to_dcm(cfg.sigma_SB)
    noisy = False
    if cfg.noise_sigma is not None:
        sensor.setWalkBounds([cfg.noise_sigma] * 3)
        noisy = True
    if cfg.white_noise is not None:
        sensor.PMatrix = np.eye(3) * cfg.white_noise
        noisy = True
    if noisy:
        _seed_rng(sensor)
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
    noisy = False
    if cfg.gyro_walk is not None:
        sensor.setWalkBoundsGyro(list(cfg.gyro_walk))
        noisy = True
    if cfg.accel_walk is not None:
        sensor.setWalkBoundsAccel(list(cfg.accel_walk))
        noisy = True
    if noisy:
        _seed_rng(sensor)
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
    if cfg.noise_sigma is not None:
        _seed_rng(sensor)
    _finalize(ctx, bundle, sensor, cfg)


@sensor_builder("gpsReceiver")
def _build_gps_receiver(
    ctx: SimContext, bundle: SpacecraftBundle, cfg: GpsReceiverConfig
) -> None:
    """Instantiate a GNSS receiver: native SimpleNav, orbit states only.

    ``SimpleNav`` reads the true state and writes truth plus its own
    Gauss-Markov error process to ``navTransMsg``. Only the position and
    velocity rows of the 18-state error are given scale: the attitude and
    rate errors stay exactly zero, so the message is a fix, not a rumor
    about orientation.
    """
    nav = simpleNav.SimpleNav()
    nav.ModelTag = f"{bundle.config.name}_{cfg.name}"
    sigma = [cfg.pos_sigma_m] * 3 + [cfg.vel_sigma_m_s] * 3 + [0.0] * 12
    nav.PMatrix = np.diag(sigma)
    nav.walkBounds = [3.0 * s for s in sigma]
    nav.RNGSeed = int(cfg.rng_seed)
    nav.scStateInMsg.subscribeTo(bundle.sc.scStateOutMsg)
    apply_params(nav, cfg.params, f"sensor {nav.ModelTag!r}")
    _finalize(ctx, bundle, nav, cfg)


@sensor_builder("camera")
def _build_camera(ctx: SimContext, bundle: SpacecraftBundle, cfg: CameraConfig) -> None:
    """Instantiate a pinhole landmark camera pointed at a world body."""
    try:
        from Basilisk.simulation import pinholeCamera
    except ImportError as err:  # pragma: no cover - depends on install extras
        raise BuildError(
            f"spacecraft {bundle.config.name!r}: camera {cfg.name!r} needs the "
            "optional opNav Basilisk feature; install with pip install 'bsk[all]'"
        ) from err
    camera = pinholeCamera.PinholeCamera()
    camera.ModelTag = f"{bundle.config.name}_{cfg.name}"
    camera.dcm_CB = _sigma_to_dcm(cfg.sigma_CB)
    camera.f = cfg.focal_length
    camera.wPixel = cfg.pixel_size
    camera.nxPixel = int(cfg.resolution[0])
    camera.nyPixel = int(cfg.resolution[1])
    if cfg.sun_mask_angle is not None:
        camera.maskSun = cfg.sun_mask_angle
    planet_msg = _target_state_msg(ctx, cfg)
    converter = ephemerisConverter.EphemerisConverter()
    converter.ModelTag = f"{camera.ModelTag}_ephemConverter"
    converter.addSpiceInputMsg(planet_msg)
    camera.ephemerisInMsg.subscribeTo(converter.ephemOutMsgs[0])
    camera.scStateInMsg.subscribeTo(bundle.sc.scStateOutMsg)
    for landmark in cfg.landmarks:
        camera.addLandmark(list(landmark.position), list(landmark.normal))
    apply_params(camera, cfg.params, f"sensor {camera.ModelTag!r}")
    if not camera.landmarkOutMsgs:
        raise BuildError(
            f"spacecraft {bundle.config.name!r}: camera {cfg.name!r} needs at "
            "least one landmark (landmarks = [{{position = [...], ...}}])"
        )
    ctx.add_dyn(converter)
    ctx.register_tag(converter.ModelTag, converter)
    ctx.add_dyn(camera)
    ctx.register_tag(camera.ModelTag, camera)
    for index, msg in enumerate(camera.landmarkOutMsgs):
        key = cfg.name if index == 0 else f"{cfg.name}_{index + 1}"
        bundle.sensor_msgs[key] = (msg, cfg.recorder)


def _target_state_msg(ctx: SimContext, cfg: CameraConfig) -> Any:
    """Resolve the camera target's live ephemeris message from ``[world]``.

    Landmark lighting needs the target's real position relative to the sun,
    so the camera requires ``[world.spice]`` ephemerides for its target.
    """
    target = (cfg.target or ctx.config.world.central_body).lower()
    msg = ctx.planet_state_msgs.get(target)
    if msg is None:
        raise BuildError(
            f"camera target {target!r} has no SPICE ephemeris: enable "
            "[world.spice] and list the target under [[world.gravities]]"
        )
    return msg


def _finalize(ctx: SimContext, bundle: SpacecraftBundle, sensor: Any, cfg: Any) -> None:
    """Add the sensor to the dynamics task and register its output message."""
    ctx.add_dyn(sensor)
    ctx.register_tag(sensor.ModelTag, sensor)
    out_msg = _sensor_output_message(sensor, cfg.type)
    bundle.sensor_msgs[cfg.name] = (out_msg, cfg.recorder)


def build_css_constellations(ctx: SimContext, bundle: SpacecraftBundle) -> None:
    """Group configured CSS sensors into aggregated sunline constellations."""
    for cluster_cfg in bundle.config.css_constellations:
        constellation = coarseSunSensor.CSSConstellation()
        constellation.ModelTag = f"{bundle.config.name}_cssConst_{cluster_cfg.name}"
        units = []
        for member in cluster_cfg.members:
            sensor = ctx.modules_by_tag.get(f"{bundle.config.name}_{member}")
            if sensor is None or not hasattr(sensor, "cssDataOutMsg"):
                raise BuildError(
                    f"spacecraft {bundle.config.name!r}: css_constellation "
                    f"{cluster_cfg.name!r} member {member!r} is not a built "
                    "coarse sun sensor"
                )
            constellation.appendCSS(sensor)
            n_hat = np.asarray(list(sensor.nHat_B), dtype=float).reshape(-1)
            n_hat = n_hat / np.linalg.norm(n_hat)
            units.append(
                messaging.CSSUnitConfigMsgPayload(
                    nHat_B=[float(component) for component in n_hat],
                    CBias=1.0,
                )
            )
        const_msg = messaging.CSSConfigMsg()
        const_msg.write(messaging.CSSConfigMsgPayload(nCSS=len(units), cssVals=units))
        ctx.add_dyn(constellation)
        ctx.register_tag(constellation.ModelTag, constellation)
        bundle.css_clusters[cluster_cfg.name] = (constellation, const_msg)


def _seed_rng(sensor: Any) -> None:
    """Enable a sensor's internal noise generator with a stable seed.

    Basilisk sensor modules only apply noise when ``RNGSeed`` is non-zero;
    seeding from the ModelTag keeps runs reproducible across processes.
    """
    if sensor.RNGSeed == 0:
        sensor.RNGSeed = zlib.crc32(sensor.ModelTag.encode()) % 2**31


def _sensor_output_message(sensor: Any, type_name: str) -> Any:
    """Return the primary output message to record for a sensor type."""
    if type_name == "coarseSunSensor":
        return sensor.cssDataOutMsg
    if type_name in ("starTracker", "imuSensor"):
        return sensor.sensorOutMsg
    if type_name == "magnetometer":
        return sensor.tamDataOutMsg
    if type_name == "gpsReceiver":
        return sensor.transOutMsg
    raise BuildError(f"no output message mapping for sensor type {type_name!r}")


def _sigma_to_dcm(sigma: list[float]) -> Any:
    """Convert an MRP orientation to a Basilisk Eigen 3x3 DCM."""
    from Basilisk.utilities import RigidBodyKinematics as rbk
    from Basilisk.utilities import simHelpers

    c = np.asarray(rbk.MRP2C(sigma), dtype=float).flatten()
    return simHelpers.np2EigenMatrix3d(c)
