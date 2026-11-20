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

"""Base vector types for Basilisk TOML simulation configuration."""

from typing import Tuple

from pydantic import BaseModel, Field


class Vector3D(BaseModel):
    """A three-dimensional vector used for positions, velocities, and directions.

    Attributes:
        x: The x component.
        y: The y component.
        z: The z component.

    Example:
        ```toml
        [satellite.orbit.position]
        x = 6730829.0
        y = -2547914.0
        z = 56500.0
        ```
    """

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def to_list(self) -> Tuple[float, float, float]:
        """Return the vector as a list of three floats."""
        return [self.x, self.y, self.z]

    def to_column_matrix(self) -> Tuple[Tuple[float], Tuple[float], Tuple[float]]:
        """Return the vector as a column matrix for Basilisk Eigen wrappers.

        Basilisk expects 3x1 Eigen matrices as lists of single-element lists,
        e.g. ``[[x], [y], [z]]``.
        """
        return [[self.x], [self.y], [self.z]]


class Vector3DAngleDeg(BaseModel):
    """A three-dimensional vector where components are interpreted as degrees.

    Attributes:
        x: The x component in degrees.
        y: The y component in degrees.
        z: The z component in degrees.
    """

    x: float = 0.0  # [deg]
    y: float = 0.0  # [deg]
    z: float = 0.0  # [deg]

    def to_radians(self) -> Tuple[float, float, float]:
        """Return the components converted to radians."""
        import math

        return (
            math.radians(self.x),
            math.radians(self.y),
            math.radians(self.z),
        )


class InertiaMatrix(BaseModel):
    """A 3x3 symmetric inertia matrix stored in column-major half-vector form.

    Basilisk uses a 9-element list for ``IHubPntBc_B`` in column-major order::

        [Ixx, Ixy, Ixz,
         Iyx, Iyy, Iyz,
         Izx, Izy, Izz]

    For a symmetric matrix, ``Ixy == Iyx``, ``Ix z == Izx``, ``Iyz == Izy``.

    Attributes:
        ixx: Moment of inertia about x-axis.
        iyy: Moment of inertia about y-axis.
        izz: Moment of inertia about z-axis.
        ixy: Product of inertia xy (== yx).
        ixz: Product of inertia xz (== zx).
        iyz: Product of inertia yz (== zy).

    Example:
        ```toml
        [satellite.hub.inertia]
        ixx = 900.0
        iyy = 800.0
        izz = 600.0
        ```
    """

    ixx: float = Field(1.0, description="I_xx [kg*m^2]")
    iyy: float = Field(1.0, description="I_yy [kg*m^2]")
    izz: float = Field(1.0, description="I_zz [kg*m^2]")
    ixy: float = Field(0.0, description="I_xy [kg*m^2]")
    ixz: float = Field(0.0, description="I_xz [kg*m^2]")
    iyz: float = Field(0.0, description="I_yz [kg*m^2]")

    def to_flat(self) -> Tuple[float, ...]:
        """Return the 9-element column-major flat list for Basilisk."""
        return (
            self.ixx, self.ixy, self.ixz,
            self.ixy, self.iyy, self.iyz,
            self.ixz, self.iyz, self.izz,
        )