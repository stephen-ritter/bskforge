#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Scenario build pipeline: run all builder phases in Basilisk-valid order."""

from __future__ import annotations

from dataclasses import dataclass

from bskforge.build import camera, custom, environment, recorders, viz
from bskforge.build.comm import build_comm_links
from bskforge.build.context import SimContext
from bskforge.build.disturbances import build_disturbances
from bskforge.build.effectors import EFFECTOR_BUILDERS
from bskforge.build.formation import build_formations
from bskforge.build.fsw import build_fsw
from bskforge.build.sensors import SENSOR_BUILDERS, build_css_constellations
from bskforge.build.spacecraft import build_vehicle
from bskforge.build.world import build_world
from bskforge.errors import UnsupportedError
from bskforge.schema.root import ScenarioConfig


@dataclass
class BuiltScenario:
    """A fully assembled, not-yet-initialized Basilisk scenario."""

    context: SimContext


def build_scenario(config: ScenarioConfig) -> BuiltScenario:
    """Assemble the complete Basilisk graph for one scenario document.

    Phases, in order: custom tasks → world → spacecraft → environment →
    disturbances → effectors → sensors → FSW → custom modules → custom
    connections → recorders → custom events → visualization → cameras.

    Args:
        config: Validated scenario configuration.

    Returns:
        The built scenario wrapping a :class:`SimContext`.

    Raises:
        UnsupportedError: A configured feature is scheduled for a later phase.
        BuildError: The graph is internally inconsistent.
    """
    ctx = SimContext(config)

    custom.build_custom_tasks(ctx)
    build_world(ctx)

    for sc_cfg in config.spacecraft:
        build_vehicle(ctx, sc_cfg)

    environment.build_environment(ctx)

    for bundle in ctx.bundles.values():
        build_disturbances(ctx, bundle)
        for effector_cfg in bundle.config.effectors:
            builder = EFFECTOR_BUILDERS.get(effector_cfg.type)
            if builder is None:
                raise UnsupportedError(
                    f"unknown effector type {effector_cfg.type!r}; "
                    f"supported: {', '.join(sorted(EFFECTOR_BUILDERS))}"
                )
            builder(ctx, bundle, effector_cfg)
        for sensor_cfg in bundle.config.sensors:
            sensor_builder = SENSOR_BUILDERS.get(sensor_cfg.type)
            if sensor_builder is None:
                raise UnsupportedError(
                    f"unknown sensor type {sensor_cfg.type!r}; supported: "
                    f"{', '.join(sorted(SENSOR_BUILDERS))}"
                )
            sensor_builder(ctx, bundle, sensor_cfg)
        build_css_constellations(ctx, bundle)
        build_fsw(ctx, bundle)

    custom.build_custom_modules(ctx)
    custom.build_custom_connections(ctx)
    build_formations(ctx)
    build_comm_links(ctx)
    recorders.build_recorders(ctx)
    custom.build_custom_events(ctx)
    custom.apply_simulation_options(ctx)
    viz.build_viz(ctx)
    camera.build_cameras(ctx)

    return BuiltScenario(context=ctx)


__all__ = ["BuiltScenario", "build_scenario"]
