#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""World build phase: gravity bodies via ``simIncludeGravBody``."""

from __future__ import annotations

from typing import Any

from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities.supportDataTools.dataFetcher import (
    DataFile,
    get_path,
)

from bskforge.build.context import SimContext
from bskforge.build.registry import apply_params
from bskforge.errors import BuildError
from bskforge.schema.world import GravityBody

#: Planet names mapped to ``gravBodyFactory`` factory methods.
FACTORY_METHODS = {
    "sun": "createSun",
    "mercury": "createMercury",
    "venus": "createVenus",
    "earth": "createEarth",
    "moon": "createMoon",
    "mars": "createMars",
    "mars_barycenter": "createMarsBarycenter",
    "jupiter": "createJupiter",
    "saturn": "createSaturn",
    "uranus": "createUranus",
    "neptune": "createNeptune",
}

#: Canonical spherical-harmonics file per planet (Basilisk support data).
DEFAULT_GRAV_FILES = {
    "earth": "GGM03S",
    "moon": "GGM2BData",
}


def build_world(ctx: SimContext) -> None:
    """Create the gravity factory, all gravitating bodies and SPICE.

    Args:
        ctx: The simulation build context.

    Raises:
        BuildError: Central-body configuration is inconsistent, or SPICE
            is requested without an epoch / with unknown kernel names.
    """
    world = ctx.config.world
    factory = simIncludeGravBody.gravBodyFactory()
    ctx.grav_factory = factory

    bodies: dict[str, Any] = {}
    explicit_central: str | None = None
    for body_cfg in world.gravities:
        body = _create_body(factory, body_cfg)
        apply_params(body, body_cfg.params, f"gravity body {body_cfg.name!r}")
        _apply_grav_model(body, body_cfg)
        bodies[body_cfg.name.lower()] = body
        if body_cfg.central:
            if explicit_central is not None:
                raise BuildError(
                    f"world.gravities: both {explicit_central!r} and "
                    f"{body_cfg.name!r} are marked central"
                )
            explicit_central = body_cfg.name.lower()

    central_key = world.central_body.lower()
    if explicit_central is not None and explicit_central != central_key:
        raise BuildError(
            "central-body conflict: [world] central_body = "
            f"{world.central_body!r} but gravity body {explicit_central!r} "
            "carries central = true"
        )
    if central_key not in bodies:
        bodies[central_key] = _create_body(
            factory, GravityBody(name=central_key, central=True)
        )
    central = bodies[central_key]
    central.isCentralBody = True

    ctx.central_body = central
    ctx.central_mu = float(central.mu)

    if world.spice.enabled:
        _build_spice(ctx, central_key)


def _build_spice(ctx: SimContext, central_key: str) -> None:
    """Create the SPICE interface for all factory bodies.

    Non-central bodies get ``loadEphemeris``; the central body becomes the
    SPICE reference frame origin. The per-body ``SpicePlanetStateMsg``
    outputs are indexed into ``ctx.planet_state_msgs`` by body name.
    """
    world = ctx.config.world
    if not world.epoch:
        raise BuildError(
            "[world.spice].enabled requires [world].epoch, e.g. "
            'epoch = "2026-01-01 00:00:00.0 TDB"'
        )
    kernels = world.spice.kernels
    try:
        kernel_enums = (
            [DataFile.EphemerisData[name] for name in kernels] if kernels else None
        )
    except KeyError as exc:
        known = ", ".join(m.name for m in DataFile.EphemerisData)
        raise BuildError(
            f"unknown SPICE kernel {exc.args[0]!r}; Basilisk support-data "
            f"kernels: {known}"
        ) from exc

    for name, body in ctx.grav_factory.gravBodies.items():
        if name != central_key:
            body.loadEphemeris = True

    kwargs: dict[str, Any] = {"time": world.epoch}
    if kernel_enums is not None:
        kwargs["spiceKernelFileNames"] = kernel_enums
    spice = ctx.grav_factory.createSpiceInterface(**kwargs)
    spice.zeroBase = central_key.capitalize()
    ctx.spice = spice
    ctx.add_dyn(spice)

    for index, name in enumerate(ctx.grav_factory.spicePlanetNames):
        ctx.planet_state_msgs[str(name).lower()] = spice.planetStateOutMsgs[index]


def _apply_grav_model(body: Any, body_cfg: GravityBody) -> None:
    """Attach the configured gravity model to a body."""
    if body_cfg.grav_order is None:
        return
    file_key = body_cfg.grav_file or DEFAULT_GRAV_FILES.get(body_cfg.name)
    if file_key is None:
        raise BuildError(
            f"gravity body {body_cfg.name!r}: no default grav file; set "
            "grav_file to one of: " + ", ".join(m.name for m in DataFile.LocalGravData)
        )
    try:
        data_file = DataFile.LocalGravData[file_key]
    except KeyError as exc:
        raise BuildError(
            f"unknown grav_file {file_key!r}; Basilisk support-data files: "
            + ", ".join(m.name for m in DataFile.LocalGravData)
        ) from exc
    body.useSphericalHarmonicsGravityModel(
        str(get_path(data_file)), body_cfg.grav_order
    )


def _create_body(factory: Any, body_cfg: GravityBody) -> Any:
    """Instantiate one gravity body from its configuration."""
    if body_cfg.type == "custom":
        if body_cfg.mu is None:  # guarded by schema validator
            raise BuildError(f"custom gravity body {body_cfg.name!r} needs mu")
        body = factory.createCustomGravObject(
            body_cfg.name,
            float(body_cfg.mu),
            radEquator=body_cfg.radius or 0.0,
            modelDictionaryKey=body_cfg.model_dictionary_key or "",
        )
        return body
    factory_method = getattr(factory, FACTORY_METHODS[body_cfg.name])
    return factory_method()
