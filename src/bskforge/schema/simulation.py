#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Top-level ``[simulation]`` section models."""

from __future__ import annotations

from pydantic import Field

from bskforge.schema.base import ScenarioModel


class DtConfig(ScenarioModel):
    """Task rates for the simulation processes.

    Attributes:
        dynamics: Integration rate for the dynamics process.
        fsw: Execution rate for the flight-software process.
    """

    dynamics: float = Field(0.1, gt=0, description="Dynamics task rate [s]")
    fsw: float = Field(1.0, gt=0, description="FSW task rate [s]")


class SimulationSection(ScenarioModel):
    """``[simulation]``: global run parameters.

    Attributes:
        name: Human-readable label used for outputs.
        duration: Total simulated time.
        dt: Dynamics and FSW task rates.
    """

    name: str = "bskforge_scenario"
    duration: float = Field(600.0, gt=0, description="Simulated duration [s]")
    dt: DtConfig = Field(default_factory=DtConfig)
