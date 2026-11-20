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

from bskforge.build.context import SimContext
from bskforge.build.registry import apply_params
from bskforge.errors import BuildError, UnsupportedError
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


def build_world(ctx: SimContext) -> None:
    """Create the gravity factory and all gravitating bodies.

    Args:
        ctx: The simulation build context.

    Raises:
        UnsupportedError: If SPICE ephemerides are requested (Phase 2).
        BuildError: Central-body configuration is inconsistent.
    """
    world = ctx.config.world
    if world.spice.enabled:
        raise UnsupportedError(
            "[world.spice] ephemerides land in Phase 2; point masses from "
            "presets or custom mu run today"
        )

    factory = simIncludeGravBody.gravBodyFactory()
    ctx.grav_factory = factory

    bodies: dict[str, Any] = {}
    explicit_central: str | None = None
    for body_cfg in world.gravities:
        body = _create_body(factory, body_cfg)
        apply_params(body, body_cfg.params, f"gravity body {body_cfg.name!r}")
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


def _create_body(factory: Any, body_cfg: GravityBody) -> Any:
    """Instantiate one gravity body from its configuration."""
    if body_cfg.type == "custom":
        body = factory.createCustomGravObject(
            body_cfg.name,
            float(body_cfg.mu),  # type: ignore[arg-type]
            radEquator=body_cfg.radius or 0.0,
            modelDictionaryKey=body_cfg.model_dictionary_key or "",
        )
        return body
    factory_method = getattr(factory, FACTORY_METHODS[body_cfg.name])
    return factory_method()
