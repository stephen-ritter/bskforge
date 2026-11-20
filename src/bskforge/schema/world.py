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
        grav_order: Spherical-harmonics truncation order; unset keeps the
            point-mass model.
        grav_file: Basilisk support-data grav-file key (e.g. ``"GGM03S"``,
            ``"GGM03S_J2_only"``); defaults to the planet's canonical file.
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
    grav_order: int | None = Field(
        None, ge=1, description="Spherical-harmonics order [-]"
    )
    grav_file: str | None = Field(None, description="LocalGravData support-file key")

    @model_validator(mode="after")
    def _check(self) -> GravityBody:
        if self.type == "planet" and self.name not in PLANET_NAMES:
            raise ValueError(
                f"unknown planet {self.name!r}; supported: "
                f'{", ".join(PLANET_NAMES)} (or set type = "custom")'
            )
        if self.type == "custom" and self.mu is None:
            raise ValueError("custom gravity bodies require an explicit mu")
        if self.grav_file is not None and self.grav_order is None:
            raise ValueError("grav_file requires grav_order")
        return self


class SunConfig(ScenarioModel):
    """Sun source for sensors and SRP.

    In the current phase the sun is a fixed inertial point relative to the
    scenario origin; SPICE-driven ephemerides arrive in Phase 2.
    """

    position: Vec3 = [1.496e11, 0.0, 0.0]


class SpiceConfig(ScenarioModel):
    """SPICE ephemerides: ``[world.spice]``.

    Attributes:
        enabled: Drive all non-central body ephemerides (and the sun
            source) from SPICE kernels; requires ``[world].epoch``.
        kernels: Basilisk ``DataFile.EphemerisData`` keys
            (e.g. ``"de430"``, ``"naif0012"``); empty uses the standard
            default kernel set.
        zero_base: Body at the inertial origin; defaults to the central
            body. Landmark cameras need ``"sun"`` (their sun line is
            derived from the target's origin-relative position).
    """

    enabled: bool = False
    kernels: list[str] = Field(default_factory=list)
    zero_base: str | None = Field(None, description="Origin body name")


class WorldSection(ScenarioModel):
    """``[world]``: gravitational environment shared by all vehicles.

    Attributes:
        central_body: Name of the central gravitating body. A preset planet
            that is auto-created when not listed in ``gravities``.
        gravities: Full body list for multi-body scenarios.
        epoch: SPICE time string, e.g. ``"2026-01-01 00:00:00.0 TDB"``;
            required when SPICE is enabled, also feeds WMM epoch.
        spice: SPICE configuration.
        sun: Fixed sun position source for sun-based sensors; ignored
            when SPICE is enabled and the sun is among ``gravities``.
    """

    central_body: str = "earth"
    gravities: list[GravityBody] = Field(default_factory=list)
    epoch: str | None = None
    spice: SpiceConfig = Field(default_factory=SpiceConfig)
    sun: SunConfig = Field(default_factory=SunConfig)
