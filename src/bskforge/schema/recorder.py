#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Recorder configuration shared across recordable messages."""

from __future__ import annotations

from pydantic import Field

from bskforge.schema.base import ScenarioModel


class RecorderConfig(ScenarioModel):
    """Recorder settings for a single message stream.

    Attributes:
        enabled: Whether to attach a recorder to this message.
        sample_time: Recording sample period. When ``None`` the builder
            auto-computes a period yielding ``default_num_points`` over the
            scenario duration.
    """

    enabled: bool = True
    sample_time: float | None = Field(
        None, gt=0, description="Recording sample period [s] (auto if unset)"
    )
