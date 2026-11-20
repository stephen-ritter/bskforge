#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""World models: ``[world]`` gravity configuration."""

from __future__ import annotations

from typing import Literal

from pydantic import Field, model_validator

from bskforge.schema.base import PassthroughModel, ScenarioModel, Vec3

#: Bodies that ``simIncludeGravBody.gravBodyFactory`` can create directly.
PLANET_NAMES = (
    "sun",
    "mercury",
    "venus",
    "earth",
    "moon",
    "mars",
    "mars_barycenter",
    "jupiter",
    "saturn",
    "uranus",
    "neptune",
)


class GravityBody(PassthroughModel):
    """A gravitating body: ``[[world.gravities]]``.

    Attributes:
        name: Planet name (for ``type = "planet"``) or custom label.
        type: ``"planet"`` uses the Basilisk factory presets; ``"custom"``
            creates a body from an explicit gravitational parameter.
        central: Mark as the central body of the scenario.
        mu: Gravitational parameter for custom bodies.
        radius: Equatorial radius for custom bodies (Vizard model scaling).
        model_dictionary_key: Vizard model-dictionary key for this body.
    """

    name: str
    type: Literal["planet", "custom"] = "planet"
    central: bool = False
    mu: float | None = Field(
        None, gt=0, description="Gravitational parameter [m^3/s^2] (custom)"
    )
    radius: float | None = Field(
        None, gt=0, description="Equatorial radius [m] (custom)"
    )
    model_dictionary_key: str | None = None

    @model_validator(mode="after")
    def _check(self) -> GravityBody:
        if self.type == "planet" and self.name not in PLANET_NAMES:
            raise ValueError(
                f"unknown planet {self.name!r}; supported: "
                f'{", ".join(PLANET_NAMES)} (or set type = "custom")'
            )
        if self.type == "custom" and self.mu is None:
            raise ValueError("custom gravity bodies require an explicit mu")
        return self


class SunConfig(ScenarioModel):
    """Sun source for sensors and SRP.

    In the current phase the sun is a fixed inertial point relative to the
    scenario origin; SPICE-driven ephemerides arrive in Phase 2.
    """

    position: Vec3 = [1.496e11, 0.0, 0.0]


class SpiceConfig(ScenarioModel):
    """SPICE ephemeris configuration (Phase 2)."""

    enabled: bool = False
    kernels: list[str] = Field(default_factory=list)


class WorldSection(ScenarioModel):
    """``[world]``: gravitational environment shared by all vehicles.

    Attributes:
        central_body: Name of the central gravitating body. A preset planet
            that is auto-created when not listed in ``gravities``.
        gravities: Full body list for multi-body scenarios.
        epoch: UTC epoch string; stored for Phase 2 (SPICE) support.
        spice: SPICE configuration (Phase 2).
        sun: Fixed sun position source for sun-based sensors.
    """

    central_body: str = "earth"
    gravities: list[GravityBody] = Field(default_factory=list)
    epoch: str | None = None
    spice: SpiceConfig = Field(default_factory=SpiceConfig)
    sun: SunConfig = Field(default_factory=SunConfig)
