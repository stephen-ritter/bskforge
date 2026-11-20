#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Orbit initial-condition models (``[spacecraft.orbit]``)."""

from __future__ import annotations

from typing import Annotated, Literal

from pydantic import Field

from bskforge.schema.base import ScenarioModel, Vec3


class ClassicalElements(ScenarioModel):
    """Classical Keplerian orbit elements; angles in degrees.

    Attributes:
        a: Semi-major axis.
        e: Eccentricity.
        i: Inclination.
        raan: Right ascension of the ascending node.
        omega: Argument of perigee.
        trueAnomaly: True anomaly at epoch.
    """

    a: float = Field(7000e3, description="Semi-major axis [m]")
    e: float = Field(0.0, ge=0, description="Eccentricity [-]")
    i: float = Field(0.0, description="Inclination [deg]")
    raan: float = Field(0.0, description="RAAN [deg]")
    omega: float = Field(0.0, description="Argument of perigee [deg]")
    trueAnomaly: float = Field(0.0, description="True anomaly [deg]")


class RVState(ScenarioModel):
    """Direct inertial position/velocity initial state.

    Attributes:
        position: Inertial position ``r_CN_N``.
        velocity: Inertial velocity ``v_CN_N``.
    """

    position: Vec3 = [0.0, 0.0, 0.0]
    velocity: Vec3 = [0.0, 0.0, 0.0]


class ElementsOrbit(ScenarioModel):
    """Orbit given by classical elements: ``type = "elements"``."""

    type: Literal["elements"] = "elements"
    elements: ClassicalElements = Field(default_factory=ClassicalElements)


class RVOrbit(ScenarioModel):
    """Orbit given by position/velocity: ``type = "rv"``."""

    type: Literal["rv"] = "rv"
    rv: RVState = Field(default_factory=RVState)


OrbitConfig = Annotated[ElementsOrbit | RVOrbit, Field(discriminator="type")]
"""Discriminated orbit-initialization union."""
