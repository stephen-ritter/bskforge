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

"""Spacecraft and hub models for Basilisk TOML simulations."""

from typing import Optional

from pydantic import BaseModel, Field

from bsk_toml_sim.models.attitude import AttitudeConfig
from bsk_toml_sim.models.orbit import OrbitConfig
from bsk_toml_sim.models.effectors import EffectorConfig
from bsk_toml_sim.models.recorder import RecorderConfig
from bsk_toml_sim.models.sensors import SensorConfig
from bsk_toml_sim.models.vector import InertiaMatrix, Vector3D


class HubConfig(BaseModel):
    """Spacecraft hub (rigid body) physical properties.

    Attributes:
        mass: Spacecraft hub mass.
        inertia: 3x3 symmetric inertia tensor about the hub center of mass.
        r_BcB_B: Position of body-fixed point B relative to hub CoM.

    Example:
        ```toml
        [satellite.hub]
        mass = 750.0

        [satellite.hub.inertia]
        ixx = 900.0
        iyy = 800.0
        izz = 600.0

        [satellite.hub.r_BcB_B]
        x = 0.0
        y = 0.0
        z = 0.0
        ```
    """

    mass: float = Field(1.0, description="mHub [kg]")
    inertia: InertiaMatrix = Field(default_factory=InertiaMatrix)
    r_BcB_B: Vector3D = Field(
        default_factory=Vector3D,
        description="r_BcB_B body-fixed offset [m]",
    )


class SpacecraftConfig(BaseModel):
    """Complete spacecraft configuration.

    Combines hub properties, orbit initial conditions, and attitude settings.

    Attributes:
        name: Unique identifier for this spacecraft.
        hub: Rigid-body hub physical properties.
        orbit: Translational initial state.
        attitude: Rotational initial state and control law.

    Example:
        ```toml
        [[satellites]]
        name = "sc1"

        [satellites.hub]
        mass = 750.0

        [satellites.hub.inertia]
        ixx = 900.0
        iyy = 800.0
        izz = 600.0

        [satellites.orbit]
        type = "elements"

        [satellites.orbit.elements]
        semi_major_axis = 7000000.0
        eccentricity = 0.01
        inclination = 33.3

        [satellites.attitude.initial]
        sigma_BN = [0.1, 0.2, -0.3]
        omega_BN_B = [0.001, -0.01, 0.03]

        [satellites.attitude.control]
        type = "mrp_feedback"

        [satellites.attitude.control.mrp_feedback]
        k = 3.5
        p = 30.0
        ```
    """

    name: str = "satellite"
    hub: HubConfig = Field(default_factory=HubConfig)
    orbit: OrbitConfig = Field(default_factory=OrbitConfig)
    attitude: Optional[AttitudeConfig] = None
    sensors: SensorConfig = Field(default_factory=SensorConfig)
    effectors: EffectorConfig = Field(default_factory=EffectorConfig)
    recorder: RecorderConfig = Field(
        default_factory=RecorderConfig,
        description="Recorder for spacecraft state message",
    )
