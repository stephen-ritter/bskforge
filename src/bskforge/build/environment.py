#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Environment build phase: magnetic field, atmosphere, eclipse models.

Environment modules are shared across vehicles with an identical
configuration: a Basilisk environment module can serve several spacecraft
(one output message each via ``addSpacecraftToModel``). A vehicle-level
``[spacecraft.environment]`` table is deep-merged over the global
``[environment]`` section before resolution.
"""

from __future__ import annotations

import json
from typing import Any

from Basilisk.simulation import (
    eclipse,
    exponentialAtmosphere,
    magneticFieldCenteredDipole,
    magneticFieldWMM,
    msisAtmosphere,
)
from Basilisk.utilities import simHelpers, simSetPlanetEnvironment
from Basilisk.utilities.supportDataTools.dataFetcher import (
    DataFile,
    get_path,
)

from bskforge.build.context import SimContext, SpacecraftBundle
from bskforge.build.registry import apply_params
from bskforge.errors import BuildError, UnsupportedError
from bskforge.schema.environment import (
    AtmosphereConfig,
    EnvironmentSection,
    MagneticFieldConfig,
)

#: Planets with analytic dipole / exponential presets in Basilisk.
DIPOLE_PRESET_PLANETS = (
    "earth",
    "mercury",
    "jupiter",
    "saturn",
    "uranus",
    "neptune",
)
ATMO_PRESET_PLANETS = ("earth",)

#: Kinds whose modules are shared per-configuration and per-planet.
MAGNETIC_FIELD = "magnetic_field"
ATMOSPHERE = "atmosphere"
ECLIPSE = "eclipse"


def resolve_environment(
    global_env: EnvironmentSection | None,
    vehicle_env: EnvironmentSection | None,
) -> EnvironmentSection | None:
    """Deep-merge a vehicle environment override over the global section.

    Args:
        global_env: Scenario-wide ``[environment]`` (may be ``None``).
        vehicle_env: ``[spacecraft.environment]`` (may be ``None``).

    Returns:
        The resolved section, or ``None`` when nothing is configured.
    """
    if global_env is None and vehicle_env is None:
        return None
    base = global_env.model_dump(mode="json", exclude_none=True) if global_env else {}
    override = (
        vehicle_env.model_dump(mode="json", exclude_none=True) if vehicle_env else {}
    )
    merged = {**base, **override}
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            merged[key] = {**base[key], **value}
    section = EnvironmentSection.model_validate(merged)
    return None if section.is_empty() else section


def build_environment(ctx: SimContext) -> None:
    """Create shared environment modules and wire them to each vehicle.

    After this phase ``ctx.env_msgs[(sc_name, kind)]`` carries the output
    message each consumer (sensors, disturbances) subscribes to.
    """
    _reject_unsupported(ctx)
    for bundle in ctx.bundles.values():
        env = resolve_environment(ctx.config.environment, bundle.config.environment)
        bundle.resolved_env = env
        if env is None:
            continue
        if env.magnetic_field is not None:
            _attach_magnetic_field(ctx, bundle, env.magnetic_field)
        if env.atmosphere is not None:
            _attach_atmosphere(ctx, bundle, env.atmosphere)
        if env.eclipse is not None:
            _attach_eclipse(ctx, bundle, env.eclipse)


def _reject_unsupported(ctx: SimContext) -> None:
    """Fail clearly for environment knobs with no Basilisk consumer yet."""
    sections = [ctx.config.environment] + [
        sc.environment for sc in ctx.config.spacecraft
    ]
    for section in sections:
        if section is not None and section.solar_flux is not None:
            raise UnsupportedError(
                "[environment].solar_flux has no consumer in this Basilisk "
                "build; the SRP effector carries its own flux constant"
            )


# --- magnetic field ---------------------------------------------------------


def _attach_magnetic_field(
    ctx: SimContext, bundle: SpacecraftBundle, cfg: MagneticFieldConfig
) -> None:
    """Get-or-create the magnetic-field module and attach this vehicle."""
    planet = ctx.config.world.central_body.lower()
    key = (MAGNETIC_FIELD, _signature(cfg, planet))
    module = ctx.env_modules.get(key)
    if module is None:
        if cfg.type == "wmm":
            module = magneticFieldWMM.MagneticFieldWMM()
            module.ModelTag = f"{planet}_wmm"
            wmm_path = get_path(DataFile.MagneticFieldData.WMM)
            module.configureWMMFile(str(wmm_path))
            epoch = ctx.config.world.epoch
            if epoch:
                epoch_msg = simHelpers.timeStringToGregorianUTCMsg(epoch)
                module.epochInMsg.subscribeTo(epoch_msg)
        else:
            module = magneticFieldCenteredDipole.MagneticFieldCenteredDipole()
            module.ModelTag = f"{planet}_dipole"
            if planet in DIPOLE_PRESET_PLANETS:
                simSetPlanetEnvironment.centeredDipoleMagField(module, planet)
            elif cfg.params:
                pass  # custom body: coefficients must arrive via params
            else:
                raise BuildError(
                    f"magnetic_field: no centered-dipole preset for "
                    f"{planet!r}; supply dipole coefficients in params "
                    "(g10, g11, h11, planetRadius)"
                )
        apply_params(module, cfg.params, f"magnetic_field {module.ModelTag!r}")
        ctx.add_dyn(module)
        ctx.register_tag(module.ModelTag, module)
        ctx.env_modules[key] = module
    index = len(module.scStateInMsgs)
    module.addSpacecraftToModel(bundle.sc.scStateOutMsg)
    ctx.env_msgs[(bundle.config.name, MAGNETIC_FIELD)] = module.envOutMsgs[index]


# --- atmosphere -------------------------------------------------------------


def _attach_atmosphere(
    ctx: SimContext, bundle: SpacecraftBundle, cfg: AtmosphereConfig
) -> None:
    """Get-or-create the atmosphere module and attach this vehicle."""
    planet = ctx.config.world.central_body.lower()
    key = (ATMOSPHERE, _signature(cfg, planet))
    module = ctx.env_modules.get(key)
    if module is None:
        if cfg.type == "msis":
            module = msisAtmosphere.MsisAtmosphere()
            module.ModelTag = f"{planet}_msis"
            for index, value in enumerate((cfg.ap, cfg.f107)):
                payload = _sw_payload(value)
                module.swDataInMsgs[index].subscribeTo(payload)
        else:
            module = exponentialAtmosphere.ExponentialAtmosphere()
            module.ModelTag = f"{planet}_expoAtmo"
            if planet in ATMO_PRESET_PLANETS:
                simSetPlanetEnvironment.exponentialAtmosphere(module, planet)
            elif not cfg.params:
                raise BuildError(
                    f"atmosphere: no exponential preset for {planet!r}; "
                    "set profile parameters in params (baseDensity, "
                    "scaleHeight, planetRadius)"
                )
        apply_params(module, cfg.params, f"atmosphere {module.ModelTag!r}")
        ctx.add_dyn(module)
        ctx.register_tag(module.ModelTag, module)
        ctx.env_modules[key] = module
    index = len(module.scStateInMsgs)
    module.addSpacecraftToModel(bundle.sc.scStateOutMsg)
    ctx.env_msgs[(bundle.config.name, ATMOSPHERE)] = module.envOutMsgs[index]


def _sw_payload(value: float) -> Any:
    """Create a static solar-weather data message with a constant value."""
    from Basilisk.architecture import messaging

    payload = messaging.SwDataMsgPayload()  # ty: ignore
    payload.dataValue = float(value)
    msg = messaging.SwDataMsg()
    msg.write(payload)
    return msg


# --- eclipse -----------------------------------------------------------------


def _attach_eclipse(ctx: SimContext, bundle: SpacecraftBundle, cfg: Any) -> None:
    """Create (once) the shared eclipse module and attach this vehicle."""
    if ctx.eclipse is None:
        module = eclipse.Eclipse()
        module.ModelTag = "eclipse"
        module.sunInMsg.subscribeTo(ctx.ensure_sun_msg())
        if ctx.central_body is not None:
            module.rEqCustom = float(ctx.central_body.radEquator)
        apply_params(module, cfg.params, f"eclipse {module.ModelTag!r}")
        ctx.add_dyn(module)
        ctx.register_tag(module.ModelTag, module)
        ctx.eclipse = module
    index = len(ctx.eclipse.positionInMsgs)
    ctx.eclipse.addSpacecraftToModel(bundle.sc.scStateOutMsg)
    ctx.eclipse.addPlanetToModel(ctx.central_state_msg())
    ctx.env_msgs[(bundle.config.name, ECLIPSE)] = ctx.eclipse.eclipseOutMsgs[index]


def _signature(cfg: Any, planet: str) -> str:
    """Stable key for one environment model configuration."""
    return json.dumps(
        {
            "planet": planet,
            "type": cfg.type,
            "params": cfg.params,
            **{
                k: v
                for k, v in cfg.model_dump(mode="json").items()
                if k not in ("type", "params")
            },
        },
        sort_keys=True,
        default=str,
    )
