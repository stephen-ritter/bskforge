#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""High-level scenario runner: load, build, run, collect, export."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import pandas as pd
from Basilisk.utilities import macros

from bskforge.build import BuiltScenario, build_scenario
from bskforge.export import frames_from_recorders, write_frames
from bskforge.loader import load_scenario
from bskforge.schema.root import ScenarioConfig


@dataclass
class ScenarioRunResult:
    """Outcome of a completed scenario run.

    Attributes:
        config: The scenario configuration that was executed.
        built: The built Basilisk graph.
        frames: Per-stream recorded data (``"sc1.state"`` → frame).
        exported_files: Paths written when export was configured.
    """

    config: ScenarioConfig
    built: BuiltScenario
    frames: dict[str, pd.DataFrame] = field(default_factory=dict)
    exported_files: list[Path] = field(default_factory=list)

    @property
    def sim(self) -> Any:
        """The underlying Basilisk ``SimBaseClass`` object."""
        return self.built.context.sim

    def to_directory(self, directory: str | Path, format: str = "csv") -> list[Path]:
        """Export all recorded streams into ``directory``."""
        self.exported_files = write_frames(self.frames, Path(directory), format)
        return self.exported_files


class ScenarioRunner:
    """Builds and executes a Basilisk simulation from a scenario config.

    Args:
        config: Validated scenario configuration.
    """

    def __init__(self, config: ScenarioConfig) -> None:
        """Store the validated scenario configuration."""
        self.config = config

    def build(self) -> BuiltScenario:
        """Assemble the Basilisk graph without initializing it."""
        return build_scenario(self.config)

    def run(
        self,
        duration: float | None = None,
        output_directory: str | Path | None = None,
    ) -> ScenarioRunResult:
        """Execute the scenario to completion and collect recorded data.

        Args:
            duration: Optional override of the configured duration (useful
                for quick smoke tests).
            output_directory: Override of ``[logging].directory`` for export.

        Returns:
            A :class:`ScenarioRunResult` with frames and exported file list.
        """
        built = self.build()
        context = built.context
        run_duration = (
            duration if duration is not None else (self.config.simulation.duration)
        )
        context.sim.ConfigureStopTime(macros.sec2nano(run_duration))
        context.sim.InitializeSimulation()
        context.sim.ExecuteSimulation()

        recorders = [
            spec for bundle in context.bundles.values() for spec in bundle.recorders
        ]
        recorders += context.scenario_recorders
        recorders += _custom_recorders(context)
        result = ScenarioRunResult(
            config=self.config,
            built=built,
            frames=frames_from_recorders(recorders),
        )

        export_dir = output_directory or self.config.logging.directory
        if export_dir is not None:
            result.to_directory(export_dir, self.config.logging.format)
        return result


def _custom_recorders(context: Any) -> list[Any]:
    """Recorders attached via ``[custom]`` are not modeled; nothing to add."""
    return []


def run_scenario(path: str | Path, **runner_kwargs: Any) -> ScenarioRunResult:
    """Load and execute a scenario file in one call."""
    config = load_scenario(path)
    return ScenarioRunner(config).run(**runner_kwargs)
