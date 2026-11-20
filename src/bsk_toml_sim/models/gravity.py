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

"""Gravity body models for Basilisk TOML simulations."""

from typing import Literal, Optional

from pydantic import BaseModel


class GravityBodyConfig(BaseModel):
    """Configuration for a gravitational body in the simulation.

    Attributes:
        name: Identifier for the body (e.g. "earth", "sun", "moon").
        is_central_body: Whether this is the central gravity source.
        use_spherical_harmonics: Enable high-order gravity model.
        spherical_harmonics_degree: Max degree for spherical harmonics.

    Example:
        ```toml
        [[simulation.gravity_bodies]]
        name = "earth"
        is_central_body = true
        ```
    """

    name: Literal["earth", "sun", "moon", "mars"] = "earth"
    is_central_body: bool = False
    use_spherical_harmonics: bool = False
    spherical_harmonics_degree: Optional[int] = None


class CentralBodyConfig(BaseModel):
    """Shorthand for a single central gravity body.

    This is the simpler form when only one central body is needed.

    Attributes:
        name: The celestial body to use as central gravitating body.

    Example:
        ```toml
        [simulation.central_body]
        name = "earth"
        ```
    """

    name: Literal["earth", "sun", "moon", "mars"] = "earth"