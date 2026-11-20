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

"""Effector models for Basilisk TOML simulations."""

from typing import Literal, Optional

from pydantic import BaseModel, Field

from bsk_toml_sim.models.recorder import RecorderConfig


class SingleReactionWheelConfig(BaseModel):
    """Configuration for a single reaction wheel.

    Attributes:
        name: Unique identifier for this reaction wheel.
        direction_B: Actuation direction in body frame (unit vector).
        max_speed: Maximum wheel speed.
        max_torque: Maximum torque output (unlimited if -1).
        jitter_model: Jitter model type.

    Example:
        ```toml
        [reaction_wheel]
        name = "RW-x"
        direction_B = [1, 0, 0]
        max_speed = 5500
        max_torque = -1
        jitter_model = "none"
        ```
    """

    name: str = "rw"
    direction_B: list[float] = Field(
        default_factory=lambda: [1.0, 0.0, 0.0],
        description="Actuation direction in body frame [-]",
    )
    max_speed: float = Field(5500.0, description="Maximum speed [rad/s]")
    max_torque: float = Field(
        -1.0,
        description="Maximum torque [Nm] (-1 for unlimited)",
    )
    jitter_model: Literal["none", "constant", "speedDependent"] = "none"


class ReactionWheelSetConfig(BaseModel):
    """Reaction wheel set configuration for a spacecraft.

    Groups multiple reaction wheels together with shared properties.

    Attributes:
        wheels: List of individual reaction wheel configurations.
        inertia_rw: Reaction wheel spin-axis moment of inertia.
        max_wheel_acceleration: Maximum allowed wheel acceleration.
        large_torque_threshold: Warning threshold for large torques.

    Example:
        ```toml
        [satellite.effectors.reaction_wheels]
        inertia_rw = 0.01

        [[satellite.effectors.reaction_wheels.wheels]]
        name = "RW-x"
        direction_B = [1, 0, 0]

        [[satellite.effectors.reaction_wheels.wheels]]
        name = "RW-y"
        direction_B = [0, 1, 0]

        [[satellite.effectors.reaction_wheels.wheels]]
        name = "RW-z"
        direction_B = [0, 0, 1]
        ```
    """

    wheels: list[SingleReactionWheelConfig] = Field(default_factory=list)
    inertia_rw: float = Field(
        0.01, description="RW spin-axis inertia [kg*m^2]"
    )
    max_wheel_acceleration: float = Field(
        1e6, description="Max wheel acceleration [rad/s^2]"
    )
    large_torque_threshold: float = Field(
        10.0, description="Large torque warning threshold [Nm]"
    )
    recorder: RecorderConfig = Field(
        default_factory=RecorderConfig,
        description="Recorder for RW speed output message",
    )


class ThrusterConfig(BaseModel):
    """Single thruster configuration.

    Attributes:
        name: Unique identifier for this thruster.
        direction_B: Thrust direction in body frame (unit vector).
        r_CMact_B: Position of thruster relative to hub CoM.
        thrust_magnitude: Thrust force magnitude.
        isp: Specific impulse.
        prop_density: Propellant density.

    Example:
        ```toml
        [[satellite.effectors.thrusters]]
        name = "thruster_z"
        direction_B = [0, 0, -1]
        r_CMact_B = [0, 0, -0.5]
        thrust_magnitude = 0.1
        isp = 200
        ```
    """

    name: str = "thruster"
    direction_B: list[float] = Field(
        default_factory=lambda: [0.0, 0.0, -1.0],
        description="Thrust direction in body frame [-]",
    )
    r_CMact_B: list[float] = Field(
        default_factory=lambda: [0.0, 0.0, 0.0],
        description="Thruster position relative to hub CoM [m]",
    )
    thrust_magnitude: float = Field(0.1, description="Thrust [N]")
    isp: int = Field(200, description="Specific impulse [s]")
    prop_density: float = Field(
        800.0, description="Propellant density [kg/m^3]"
    )
    recorder: RecorderConfig = Field(
        default_factory=RecorderConfig,
        description="Recorder for thruster output message",
    )


class SRPEffectorConfig(BaseModel):
    """Solar radiation pressure effector configuration.

    Attributes:
        facet_areas: List of facet areas for the spacecraft.
        facet_normals_B: Facet normals in body frame (one per facet).
        reflectivity_coeff: Specular reflection coefficient.
        absorption_coeff: Absorption coefficient.

    Example:
        ```toml
        [satellite.effectors.srp]
        facet_areas = [2.0, 2.0, 1.0, 1.0, 3.0, 3.0]
        reflectivity_coeff = 0.3
        absorption_coeff = 0.7
        ```
    """

    facet_areas: list[float] = Field(
        default_factory=list, description="Facet areas [m^2]"
    )
    facet_normals_B: list[list[float]] = Field(
        default_factory=list,
        description="Facet normals in body frame (3-element vectors) [-]",
    )
    reflectivity_coeff: float = Field(
        0.3, description="Specular reflection coefficient [-]"
    )
    absorption_coeff: float = Field(
        0.7, description="Absorption coefficient [-]"
    )


class DragEffectorConfig(BaseModel):
    """Aerodynamic drag effector configuration.

    Attributes:
        drag_coefficient: Drag coefficient Cd.
        reference_area: Reference area for drag calculation.
        facet_areas: Per-facet areas for facet-based drag.
        facet_normals_B: Facet normals in body frame.

    Example:
        ```toml
        [satellite.effectors.drag]
        drag_coefficient = 2.2
        reference_area = 10.0
        ```
    """

    drag_coefficient: float = Field(2.2, description="Drag coefficient Cd [-]")
    reference_area: float = Field(10.0, description="Reference area [m^2]")
    facet_areas: list[float] = Field(
        default_factory=list, description="Facet areas [m^2]"
    )
    facet_normals_B: list[list[float]] = Field(
        default_factory=list,
        description="Facet normals in body frame (3-element vectors) [-]",
    )


class EffectorConfig(BaseModel):
    """Container for all effectors on a spacecraft.

    Attributes:
        reaction_wheels: Reaction wheel set configuration.
        thrusters: List of thruster configurations.
        srp: Solar radiation pressure effector.
        drag: Aerodynamic drag effector.

    Example:
        ```toml
        [satellite.effectors]

        [satellite.effectors.reaction_wheels]
        inertia_rw = 0.01

        [[satellite.effectors.reaction_wheels.wheels]]
        name = "RW-x"
        direction_B = [1, 0, 0]

        [[satellite.effectors.reaction_wheels.wheels]]
        name = "RW-y"
        direction_B = [0, 1, 0]

        [[satellite.effectors.thrusters]]
        name = "thruster_z"
        direction_B = [0, 0, -1]
        thrust_magnitude = 0.1
        ```
    """

    reaction_wheels: Optional[ReactionWheelSetConfig] = None
    thrusters: list[ThrusterConfig] = Field(default_factory=list)
    srp: Optional[SRPEffectorConfig] = None
    drag: Optional[DragEffectorConfig] = None