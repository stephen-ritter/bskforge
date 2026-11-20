#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Visualization models: ``[visualization]`` (Vizard)."""

from __future__ import annotations

from bskforge.schema.base import ScenarioModel


class VisualizationSection(ScenarioModel):
    """``[visualization]``: Vizard live/replay visualization settings.

    Attributes:
        enabled: Attach the Basilisk ``vizInterface`` module.
        live_stream: Stream state in real time (adds a clock-sync module).
        broadcast_stream: Broadcast messages for listener Vizard instances.
        save_file: Write a Vizard replay data file at this path.
        no_display: Run Vizard headless (performance / CI capture).
    """

    enabled: bool = False
    live_stream: bool = False
    broadcast_stream: bool = False
    save_file: str | None = None
    no_display: bool = False


class SpacecraftViz(ScenarioModel):
    """Per-vehicle Vizard appearance hints: ``[spacecraft.viz]``.

    Attributes:
        model_key: Vizard model-dictionary key for this vehicle.
    """

    model_key: str | None = None
