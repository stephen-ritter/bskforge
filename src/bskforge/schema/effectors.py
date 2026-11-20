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

from bskforge.schema.base import PassthroughModel, Vec3
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


class DisturbanceConfig(PassthroughModel):
    """Dynamics disturbance effector: ``[[spacecraft.disturbances]]``.

    SRP and drag effector wiring lands in Phase 2; the block validates today
    so scenario files can already express their disturbance intent.
    """

    type: Literal["srp", "drag"]
    params: dict = Field(default_factory=dict)


EffectorConfig = Annotated[
    ReactionWheelsConfig | ThrustersConfig,
    Field(discriminator="type"),
]
"""Discriminated effector union for ``[[spacecraft.effectors]]``."""
