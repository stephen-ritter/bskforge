#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Disturbance build phase: gravity gradient, drag and SRP effectors."""

from __future__ import annotations

from collections.abc import Callable

from Basilisk.simulation import (
    GravityGradientEffector,
    dragDynamicEffector,
    facetDragDynamicEffector,
    facetSRPDynamicEffector,
    radiationPressure,
)

from bskforge.build.context import SimContext, SpacecraftBundle
from bskforge.build.environment import ATMOSPHERE, ECLIPSE
from bskforge.build.registry import apply_params
from bskforge.errors import BuildError
from bskforge.schema.effectors import (
    DragDisturbance,
    GravityGradientDisturbance,
    SolarRadiationPressureDisturbance,
)

DISTURBANCE_BUILDERS: dict[str, Callable[..., None]] = {}


def disturbance_builder(type_name: str) -> Callable[[Callable], Callable]:
    """Register a builder function for a disturbance ``type``."""

    def _decorator(fn: Callable) -> Callable:
        DISTURBANCE_BUILDERS[type_name] = fn
        return fn

    return _decorator


@disturbance_builder("gravityGradient")
def _build_gravity_gradient(
    ctx: SimContext, bundle: SpacecraftBundle, cfg: GravityGradientDisturbance
) -> None:
    """Attach the gravity-gradient effector for the central body."""
    effector = GravityGradientEffector.GravityGradientEffector()
    effector.ModelTag = f"{bundle.config.name}_gravGrad"
    effector.addPlanetName(str(ctx.central_body.planetName))
    apply_params(effector, cfg.params, f"disturbance {effector.ModelTag!r}")
    bundle.sc.addDynamicEffector(effector)
    ctx.add_dyn(effector)
    ctx.register_tag(effector.ModelTag, effector)


@disturbance_builder("drag")
def _build_drag(
    ctx: SimContext, bundle: SpacecraftBundle, cfg: DragDisturbance
) -> None:
    """Attach the drag effector fed by the atmosphere model."""
    atmo_msg = ctx.env_msgs.get((bundle.config.name, ATMOSPHERE))
    if atmo_msg is None:
        raise BuildError(
            f"spacecraft {bundle.config.name!r}: a drag disturbance needs "
            "an [environment.atmosphere] model to source density"
        )
    if cfg.model == "faceted":
        effector = facetDragDynamicEffector.FacetDragDynamicEffector()
        for facet in cfg.facets:
            effector.addFacet(
                facet.area,
                facet.drag_coeff,
                list(facet.normal_B),
                list(facet.location_B),
            )
    else:
        effector = dragDynamicEffector.DragDynamicEffector()
        effector.coreParams.projectedArea = cfg.area
        effector.coreParams.dragCoeff = cfg.drag_coeff
    effector.ModelTag = f"{bundle.config.name}_drag"
    effector.atmoDensInMsg.subscribeTo(atmo_msg)
    apply_params(effector, cfg.params, f"disturbance {effector.ModelTag!r}")
    bundle.sc.addDynamicEffector(effector)
    ctx.add_dyn(effector)
    ctx.register_tag(effector.ModelTag, effector)


@disturbance_builder("solarRadiationPressure")
def _build_srp(
    ctx: SimContext,
    bundle: SpacecraftBundle,
    cfg: SolarRadiationPressureDisturbance,
) -> None:
    """Attach the SRP effector (optionally eclipse-shadowed)."""
    if cfg.model == "faceted":
        effector = facetSRPDynamicEffector.FacetSRPDynamicEffector()
        effector.setNumFacets(len(cfg.facets))
        for facet in cfg.facets:
            effector.addFacet(
                facet.area,
                _IDENTITY_DCM,
                list(facet.nHat_B),
                [0.0, 0.0, 1.0],
                list(facet.location_B),
                facet.diffuse_coeff,
                facet.specular_coeff,
            )
        effector.sunInMsg.subscribeTo(ctx.ensure_sun_msg())
    else:
        effector = radiationPressure.RadiationPressure()
        effector.area = cfg.area
        effector.coefficientReflection = cfg.reflection_coeff
        effector.sunEphmInMsg.subscribeTo(ctx.ensure_sun_msg())
    effector.ModelTag = f"{bundle.config.name}_srp"
    eclipse_msg = ctx.env_msgs.get((bundle.config.name, ECLIPSE))
    if eclipse_msg is not None:
        effector.sunEclipseInMsg.subscribeTo(eclipse_msg)
    apply_params(effector, cfg.params, f"disturbance {effector.ModelTag!r}")
    bundle.sc.addDynamicEffector(effector)
    ctx.add_dyn(effector)
    ctx.register_tag(effector.ModelTag, effector)


_IDENTITY_DCM = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def build_disturbances(ctx: SimContext, bundle: SpacecraftBundle) -> None:
    """Build every ``[[spacecraft.disturbances]]`` effector of a vehicle."""
    for dist_cfg in bundle.config.disturbances:
        builder = DISTURBANCE_BUILDERS.get(dist_cfg.type)
        if builder is None:  # pragma: no cover - union is closed
            raise BuildError(f"unknown disturbance type {dist_cfg.type!r}")
        builder(ctx, bundle, dist_cfg)
