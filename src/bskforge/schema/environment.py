#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Environment models: ``[environment]`` and per-vehicle overrides (Phase 2)."""

from __future__ import annotations

from pydantic import Field

from bskforge.schema.base import ScenarioModel


class MagneticFieldConfig(ScenarioModel):
    """Planetary magnetic field model selection (Phase 2)."""

    type: str = "centered_dipole"
    params: dict = Field(default_factory=dict)


class AtmosphereConfig(ScenarioModel):
    """Planetary atmosphere model selection (Phase 2)."""

    type: str = "exponential"
    params: dict = Field(default_factory=dict)


class EclipseConfig(ScenarioModel):
    """Eclipse model selection (Phase 2)."""

    type: str = "penumbra"


class EnvironmentSection(ScenarioModel):
    """``[environment]``: environment models applied to every vehicle.

    All models land in Phase 2; the section is validated today so scenario
    files can already carry their intended environment configuration.
    """

    magnetic_field: MagneticFieldConfig | None = None
    atmosphere: AtmosphereConfig | None = None
    eclipse: EclipseConfig | None = None
    solar_flux: dict | None = None
