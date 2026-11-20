#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Output/logging configuration: ``[logging]``."""

from __future__ import annotations

from typing import Literal

from pydantic import Field

from bskforge.schema.base import ScenarioModel


class LoggingSection(ScenarioModel):
    """``[logging]``: recording defaults and data export.

    Attributes:
        directory: Directory to write exported data into (``None`` skips
            export; data is still available in the returned result object).
        format: Export container format.
        default_num_points: Target point count for auto-computed recorder
            sample times when a recorder leaves ``sample_time`` unset.
    """

    directory: str | None = None
    format: Literal["csv", "parquet", "npz"] = "csv"
    default_num_points: int = Field(1000, gt=0)
