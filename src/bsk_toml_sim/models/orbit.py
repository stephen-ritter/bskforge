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

"""Orbit configuration models for Basilisk TOML simulations."""

from typing import Literal, Optional

from pydantic import BaseModel, Field

from bsk_toml_sim.models.vector import Vector3D


class ClassicalOrbitElements(BaseModel):
    """Classical (Keplerian) orbit elements for initial orbit specification.

    Angles are specified in degrees and converted to radians internally.

    Attributes:
        semi_major_axis: Semi-major axis.
        eccentricity: Orbit eccentricity (0 <= e < 1 for bound orbits).
        inclination: Orbit inclination relative to reference plane.
        raan: Right ascension of the ascending node.
        arg_perigee: Argument of perigee.
        true_anomaly: True anomaly at epoch.

    Example:
        ```toml
        [satellite.orbit.elements]
        semi_major_axis = 7000000.0
        eccentricity = 0.0001
        inclination = 33.3
        raan = 48.2
        arg_perigee = 347.8
        true_anomaly = 85.3
        ```
    """

    semi_major_axis: float = Field(7000e3, description="a [m]")
    eccentricity: float = Field(0.0, description="e [-]")
    inclination: float = Field(0.0, description="i [deg]")
    raan: float = Field(0.0, description="Omega [deg]")
    arg_perigee: float = Field(0.0, description="omega [deg]")
    true_anomaly: float = Field(0.0, description="f [deg]")


class RVState(BaseModel):
    """Direct inertial position and velocity state.

    Attributes:
        position: Inertial position vector r_CN_N.
        velocity: Inertial velocity vector v_CN_N.

    Example:
        ```toml
        [satellite.orbit]
        type = "rv"

        [satellite.orbit.position]
        x = 6730829.0
        y = -2547914.0
        z = 56500.0

        [satellite.orbit.velocity]
        x = 2.1518
        y = 3.1472
        z = 0.2016
        ```
    """

    position: Vector3D = Field(default_factory=Vector3D)
    velocity: Vector3D = Field(default_factory=Vector3D)


class OrbitConfig(BaseModel):
    """Orbit initial condition configuration for a spacecraft.

    Supports two modes via the ``type`` discriminator:

    - ``"elements"``: Classical Keplerian orbit elements
    - ``"rv"``: Direct position and velocity vectors

    Attributes:
        type: Which orbit specification mode to use.
        elements: Classical orbit elements (required when type = "elements").
        rv: Position/velocity state (required when type = "rv").

    Example:
        ```toml
        [satellite.orbit]
        type = "elements"

        [satellite.orbit.elements]
        semi_major_axis = 7000000.0
        eccentricity = 0.01
        inclination = 33.3
        raan = 48.2
        arg_perigee = 347.8
        true_anomaly = 85.3
        ```
    """

    type: Literal["elements", "rv"] = "elements"
    elements: Optional[ClassicalOrbitElements] = None
    rv: Optional[RVState] = None

    def get_elements(self) -> ClassicalOrbitElements:
        """Return the classical orbit elements.

        Raises:
            ValueError: If the orbit type is not "elements".
        """
        if self.type != "elements" or self.elements is None:
            raise ValueError(
                "Orbit type is 'rv', but get_elements() was called. "
                "Set type = 'elements' and provide [orbit.elements]."
            )
        return self.elements

    def get_rv(self) -> RVState:
        """Return the RV state.

        Raises:
            ValueError: If the orbit type is not "rv".
        """
        if self.type != "rv" or self.rv is None:
            raise ValueError(
                "Orbit type is 'elements', but get_rv() was called. "
                "Set type = 'rv' and provide [orbit.position]/[orbit.velocity]."
            )
        return self.rv