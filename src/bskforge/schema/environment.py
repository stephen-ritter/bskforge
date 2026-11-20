#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Environment models: ``[environment]`` and per-vehicle overrides."""

from __future__ import annotations

from typing import Literal

from pydantic import Field

from bskforge.schema.base import PassthroughModel, ScenarioModel


class MagneticFieldConfig(PassthroughModel):
    """Planetary magnetic field: ``[environment.magnetic_field]``.

    Attributes:
        type: ``"centered_dipole"`` (analytic, preset coefficients for
            Earth/Jupiter) or ``"wmm"`` (World Magnetic Model file, epoch
            taken from ``[world].epoch`` when set).
    """

    type: Literal["centered_dipole", "wmm"] = "centered_dipole"


class AtmosphereConfig(PassthroughModel):
    """Planetary atmosphere: ``[environment.atmosphere]``.

    Attributes:
        type: ``"exponential"`` (preset profile) or ``"msis"``
            (NRLMSISE-00 with static space-weather inputs).
        ap: Daily geomagnetic Ap index used by MSIS.
        f107: F10.7 cm solar radio flux used by MSIS.
    """

    type: Literal["exponential", "msis"] = "exponential"
    ap: float = Field(4.0, ge=0, description="Daily Ap index (MSIS)")
    f107: float = Field(150.0, ge=0, description="F10.7 flux (MSIS)")


class EclipseConfig(PassthroughModel):
    """Planet shadow model: ``[environment.eclipse]``.

    Feeds ``sunEclipseInMsg`` on sun-facing sensors and SRP effectors.
    """

    type: Literal["penumbra"] = "penumbra"


class EnvironmentSection(ScenarioModel):
    """``[environment]``: environment models applied to every vehicle.

    A vehicle-level ``[spacecraft.environment]`` table is deep-merged over
    the global section, so scenarios declare a common environment once and
    adjust single models per vehicle.

    Attributes:
        magnetic_field: Field model consumed by magnetometers (and MTBs).
        atmosphere: Density model consumed by drag effectors.
        eclipse: Umbra/penumbra shadow model.
        solar_flux: Reserved; today's SRP module carries its own flux
            constant, so setting this raises a build error.
    """

    magnetic_field: MagneticFieldConfig | None = None
    atmosphere: AtmosphereConfig | None = None
    eclipse: EclipseConfig | None = None
    solar_flux: float | None = Field(None, gt=0)

    def is_empty(self) -> bool:
        """True when no model is configured."""
        return (
            self.magnetic_field is None
            and self.atmosphere is None
            and self.eclipse is None
            and self.solar_flux is None
        )
