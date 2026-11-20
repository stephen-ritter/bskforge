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

"""Recorder configuration for Basilisk TOML simulations."""

from typing import Optional

from pydantic import BaseModel, Field


class RecorderConfig(BaseModel):
    """Data recorder settings for a single message stream.

    Each recordable component (spacecraft state, sensor output, effector
    output, control torque) carries its own ``RecorderConfig`` so the user
    can independently enable/disable logging and set the sample period.

    Attributes:
        enabled: Whether to attach a recorder to this message.
        sample_time: Recording sample period in seconds.  If ``None``, the
            runner will auto-compute a sample time that yields an even
            distribution of points over the simulation duration.

    Example:
        ```toml
        [satellite.state_recorder]
        enabled = true
        sample_time = 0.5

        [satellite.attitude.control.torque_recorder]
        enabled = true
        sample_time = 0.1
        ```
    """

    enabled: bool = Field(True, description="Enable recorder [-]")
    sample_time: Optional[float] = Field(
        None,
        description="Recording sample period [s] (auto if null)",
    )