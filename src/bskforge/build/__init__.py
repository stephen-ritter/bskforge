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

from bskforge.build import custom, environment, recorders, viz
from bskforge.build.context import SimContext
from bskforge.build.effectors import EFFECTOR_BUILDERS
from bskforge.build.fsw import build_fsw
from bskforge.build.registry import unsupported
from bskforge.build.sensors import SENSOR_BUILDERS
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

    Phases, in order: custom tasks → world → spacecraft → effectors →
    sensors → FSW → custom modules → environment guard → custom
    connections → recorders → custom events → visualization.

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
        bundle = build_vehicle(ctx, sc_cfg)
        for effector_cfg in sc_cfg.effectors:
            builder = EFFECTOR_BUILDERS.get(effector_cfg.type)
            if builder is None:
                raise UnsupportedError(
                    f"unknown effector type {effector_cfg.type!r}; "
                    f"supported: {', '.join(sorted(EFFECTOR_BUILDERS))}"
                )
            builder(ctx, bundle, effector_cfg)
        for sensor_cfg in sc_cfg.sensors:
            sensor_builder = SENSOR_BUILDERS.get(sensor_cfg.type)
            if sensor_builder is None:
                raise UnsupportedError(
                    f"unknown sensor type {sensor_cfg.type!r}; supported: "
                    f"{', '.join(sorted(SENSOR_BUILDERS))}"
                )
            sensor_builder(ctx, bundle, sensor_cfg)
        build_fsw(ctx, bundle)

    custom.build_custom_modules(ctx)
    environment.check_environment_support(ctx)
    custom.build_custom_connections(ctx)
    recorders.build_recorders(ctx)
    custom.build_custom_events(ctx)
    custom.apply_simulation_options(ctx)
    viz.build_viz(ctx)

    return BuiltScenario(context=ctx)


# Registry completeness: types present in the schema but deferred phases.
EFFECTOR_BUILDERS.setdefault(
    "thrusters", unsupported("Thruster effectors", "Phase 4")
)

__all__ = ["BuiltScenario", "build_scenario"]
