#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Effector models: ``[[spacecraft.effectors]]``."""

from __future__ import annotations

from typing import Annotated, Literal

from pydantic import Field, model_validator

from bskforge.schema.base import PassthroughModel, ScenarioModel, Vec3
from bskforge.schema.recorder import RecorderConfig


class WheelConfig(PassthroughModel):
    """A single reaction wheel device.

    Attributes:
        name: Optional device label (5 characters max).
        direction_B: Spin-axis unit vector in the body frame.
        max_speed: Maximum wheel speed.
        max_torque: Motor torque limit; ``None`` uses the factory default.
        jitter_model: Jitter model for the effector integration.
        initial_speed: Wheel speed at ``Reset()``.
        params: Extra keyword arguments forwarded to
            :meth:`simIncludeRW.rwFactory.create` (e.g. ``fCoulomb``,
            ``rWB_B``, ``useRWfriction``).
    """

    name: str | None = None
    direction_B: Vec3 = [1.0, 0.0, 0.0]
    max_speed: float = Field(5500.0, gt=0, description="Max speed [rad/s]")
    max_torque: float | None = Field(None, gt=0, description="Motor torque limit [Nm]")
    jitter_model: Literal["none", "constant", "speedDependent"] = "none"
    initial_speed: float = Field(0.0, description="Initial wheel speed [rpm]")


class ReactionWheelsConfig(PassthroughModel):
    """Reaction-wheel set: ``type = "reactionWheels"``.

    Attributes:
        wheels: Individual wheel devices of this set.
        inertia: Spin-axis moment of inertia, shared by all wheels.
        recorder: Recorder for the aggregate wheel-speed message.
    """

    type: Literal["reactionWheels"] = "reactionWheels"
    wheels: list[WheelConfig] = Field(default_factory=list)
    inertia: float = Field(0.01, gt=0, description="Spin inertia Js [kg*m^2]")
    recorder: RecorderConfig = Field(default_factory=RecorderConfig)

    @model_validator(mode="after")
    def _require_wheels(self) -> ReactionWheelsConfig:
        if not self.wheels:
            raise ValueError(
                "a reactionWheels effector needs at least one "
                "[[spacecraft.effectors.wheels]] entry"
            )
        return self


class ThrustersConfig(PassthroughModel):
    """Thruster set (effector wiring lands in Phase 4)."""

    type: Literal["thrusters"] = "thrusters"
    thrusters: list[dict] = Field(default_factory=list)


class GravityGradientDisturbance(PassthroughModel):
    """Gravity-gradient torque: ``type = "gravityGradient"``.

    Uses the central body's ``mu`` and the vehicle's mass properties
    (hub inertia and CoM offset) automatically.
    """

    type: Literal["gravityGradient"] = "gravityGradient"


class DragFacet(ScenarioModel):
    """One flat plate of a faceted drag geometry.

    Attributes:
        area: Facet area.
        drag_coeff: Facet drag coefficient.
        normal_B: Outward unit normal in the body frame.
        location_B: Facet center-of-pressure offset in the body frame.
    """

    area: float = Field(gt=0, description="Facet area [m^2]")
    drag_coeff: float = Field(2.2, gt=0, description="Facet drag coeff [-]")
    normal_B: Vec3
    location_B: Vec3 = [0.0, 0.0, 0.0]


class SrpFacet(ScenarioModel):
    """One facet of a faceted SRP geometry.

    Attributes:
        area: Facet area.
        nHat_B: Outward unit normal in the body frame.
        location_B: Facet center-of-pressure offset in the body frame.
        diffuse_coeff: Diffuse reflection fraction.
        specular_coeff: Specular reflection fraction.
    """

    area: float = Field(gt=0, description="Facet area [m^2]")
    nHat_B: Vec3
    location_B: Vec3 = [0.0, 0.0, 0.0]
    diffuse_coeff: float = Field(0.2, ge=0, le=1)
    specular_coeff: float = Field(0.5, ge=0, le=1)


class DragDisturbance(PassthroughModel):
    """Atmospheric drag: ``type = "drag"``.

    Requires an ``[environment.atmosphere]`` model to source density.

    Attributes:
        model: ``"cannonball"`` (single area/Cd) or ``"faceted"``.
        area: Cannonball projected area.
        drag_coeff: Drag coefficient ``dragCoeff``.
        facets: Facet list when ``model = "faceted"``.
    """

    type: Literal["drag"] = "drag"
    model: Literal["cannonball", "faceted"] = "cannonball"
    area: float = Field(20.0, gt=0, description="Projected area [m^2]")
    drag_coeff: float = Field(2.2, gt=0, description="Drag coeff [-]")
    facets: list[DragFacet] = Field(default_factory=list)

    @model_validator(mode="after")
    def _check_facets(self) -> DragDisturbance:
        if self.model == "faceted" and not self.facets:
            raise ValueError('drag model "faceted" requires facets')
        return self


class SolarRadiationPressureDisturbance(PassthroughModel):
    """Solar radiation pressure: ``type = "solarRadiationPressure"``.

    Sun position comes from ``[world]`` (static) or SPICE; add
    ``[environment.eclipse]`` to shadow the spacecraft in eclipse.

    Attributes:
        model: ``"cannonball"`` (single area/cr) or ``"faceted"``.
        area: Cannonball area.
        reflection_coeff: Momentum transfer coefficient
            ``coefficientReflection`` (1 = absorbing, 2 = specular).
        facets: Facet list when ``model = "faceted"``.
    """

    type: Literal["solarRadiationPressure"] = "solarRadiationPressure"
    model: Literal["cannonball", "faceted"] = "cannonball"
    area: float = Field(20.0, gt=0, description="Cannonball area [m^2]")
    reflection_coeff: float = Field(
        1.3, ge=0, le=2, description="coefficientReflection [-]"
    )
    facets: list[SrpFacet] = Field(default_factory=list)

    @model_validator(mode="after")
    def _check_facets(self) -> SolarRadiationPressureDisturbance:
        if self.model == "faceted" and not self.facets:
            raise ValueError('solarRadiationPressure model "faceted" requires facets')
        return self


DisturbanceConfig = Annotated[
    GravityGradientDisturbance | DragDisturbance | SolarRadiationPressureDisturbance,
    Field(discriminator="type"),
]
"""Discriminated disturbance union for ``[[spacecraft.disturbances]]``."""


EffectorConfig = Annotated[
    ReactionWheelsConfig | ThrustersConfig,
    Field(discriminator="type"),
]
"""Discriminated effector union for ``[[spacecraft.effectors]]``."""
