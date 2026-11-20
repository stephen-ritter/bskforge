#
#  ISC License
#
#  Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for
#  any purpose with or without fee is hereby granted, provided that the
#  above copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

"""Top-level simulation configuration model."""

from typing import List, Optional

from pydantic import BaseModel, Field

from bsk_toml_sim.models.gravity import (
    CentralBodyConfig,
    GravityBodyConfig,
)
from bsk_toml_sim.models.spacecraft import SpacecraftConfig


class VisualizationConfig(BaseModel):
    """Visualization settings for Basilisk Vizard/Unity.

    Attributes:
        enabled: Whether to enable visualization.
        live_stream: Stream spacecraft state data in real-time so Vizard can
            display the simulation as it runs (requires simSynch.ClockSynch).
        broadcast_stream: Broadcast messages so multiple listener Vizards can
            pick up the data simultaneously.
        save_file: Path to the visualization data file. If set, Basilisk will
            write a saveFile that Vizard can replay after the simulation.

    Example TOML::

        [simulation.visualization]
        enabled = true
        live_stream = true
        broadcast_stream = false
    """

    enabled: bool = False
    live_stream: bool = Field(
        False,
        description="Stream state data in real-time to Vizard",
    )
    broadcast_stream: bool = Field(
        False,
        description="Broadcast messages for listener Vizards",
    )
    save_file: Optional[str] = Field(
        None,
        description="Path to visualization save file",
    )


class SimulationConfig(BaseModel):
    """Top-level simulation configuration parsed from a TOML file.

    This is the root model that contains global simulation parameters and
    references to one or more spacecraft configurations.

    Attributes:
        name: Human-readable simulation label.
        dt_dynamics: Dynamics integration timestep.
        dt_fsw: Flight software task timestep.
        duration: Total simulation wall-clock duration.
        central_body: Shorthand single central gravity body.
        gravity_bodies: Full multi-body gravity configuration.
        satellites: List of spacecraft in the simulation.
        logging: Data logging settings.

    Example TOML::

        [simulation]
        name = "LEO detumble"
        dt_dynamics = 0.1
        dt_fsw = 0.1
        duration = 600.0

        [simulation.central_body]
        name = "earth"

        [[satellites]]
        name = "sc1"

        [satellites.hub]
        mass = 750.0

        [satellites.hub.inertia]
        ixx = 900.0
        iyy = 800.0
        izz = 600.0
    """

    name: str = "untitled_simulation"
    dt_dynamics: float = Field(0.1, description="Dynamics timestep [s]")
    dt_fsw: float = Field(0.1, description="FSW timestep [s]")
    duration: float = Field(600.0, description="Simulation duration [s]")
    central_body: Optional[CentralBodyConfig] = None
    gravity_bodies: List[GravityBodyConfig] = Field(default_factory=list)
    satellites: List[SpacecraftConfig] = Field(default_factory=list)
    visualization: Optional[VisualizationConfig] = None

    def get_central_body_name(self) -> str:
        """Return the central body name.

        Checks ``central_body`` first, then falls back to the first gravity
        body marked as ``is_central_body``. Returns ``"earth"`` if neither is
        set.
        """
        if self.central_body is not None:
            return self.central_body.name
        for body in self.gravity_bodies:
            if body.is_central_body:
                return body.name
        return "earth"