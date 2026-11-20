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

"""Attitude configuration models for Basilisk TOML simulations."""

from typing import Literal, Optional

from pydantic import BaseModel, Field

from bsk_toml_sim.models.recorder import RecorderConfig


class AttitudeInitialCondition(BaseModel):
    """Initial attitude and rate state for a spacecraft.

    Attributes:
        sigma_BN: Initial modified Rodrigues parameters (MRP).
        omega_BN_B: Initial body angular rate in body frame.

    Example:
        ```toml
        [satellite.attitude.initial]
        sigma_BN = [0.1, 0.2, -0.3]
        omega_BN_B = [0.001, -0.01, 0.03]
        ```
    """

    sigma_BN: list[float] = Field(
        default_factory=lambda: [0.0, 0.0, 0.0],
        description="Initial MRP sigma_BN_B [-]",
    )
    omega_BN_B: list[float] = Field(
        default_factory=lambda: [0.0, 0.0, 0.0],
        description="Initial body rate omega_BN_B [rad/s]",
    )


class MRPFeedbackConfig(BaseModel):
    """Gains for the MRP feedback control law (mrpFeedback module).

    Attributes:
        k: Proportional gain on attitude error (MRP sigma).
        p: Proportional gain on rate error.
        ki: Integral gain (negative to disable integral action).
        integral_limit: Saturation limit for the integral term.
        known_torque: Known/disturbance torque for feedforward cancellation.

    Example:
        ```toml
        [satellite.attitude.control.mrp_feedback]
        k = 3.5
        p = 30.0
        ki = -1.0
        ```
    """

    k: float = Field(3.5, description="K gain [-]")
    p: float = Field(30.0, description="P gain [-]")
    ki: float = Field(-1.0, description="Ki gain (negative to disable) [-]")
    integral_limit: Optional[float] = Field(
        None, description="Integral term saturation limit [-]"
    )
    known_torque: list[float] = Field(
        default_factory=lambda: [0.0, 0.0, 0.0],
        description="knownTorquePntB_B [Nm]",
    )

    def compute_integral_limit(self) -> float:
        """Compute the integral limit from K and Ki if not set explicitly."""
        if self.integral_limit is not None:
            return self.integral_limit
        if self.ki < 0:
            return 0.0
        return 2.0 / self.ki * 0.1


class AttitudeControlConfig(BaseModel):
    """Attitude control configuration for a spacecraft.

    Supports the following control types via the ``type`` field:

    - ``"mrp_feedback"``: Classic MRP feedback law with optional RW steering

    Attributes:
        type: Which attitude control law to use.
        mrp_feedback: MRP feedback control gains.
        torque_recorder: Recorder for commanded control torque message.

    Example:
        ```toml
        [satellite.attitude.control]
        type = "mrp_feedback"

        [satellite.attitude.control.mrp_feedback]
        k = 3.5
        p = 30.0
        ki = -1.0

        [satellite.attitude.control.torque_recorder]
        enabled = true
        sample_time = 0.1
        ```
    """

    type: Literal["mrp_feedback"] = "mrp_feedback"
    mrp_feedback: Optional[MRPFeedbackConfig] = None
    recorder: RecorderConfig = Field(
        default_factory=RecorderConfig,
        description="Recorder for commanded torque message",
    )


class AttitudeConfig(BaseModel):
    """Complete attitude configuration for a spacecraft.

    Combines initial conditions and control law settings.

    Attributes:
        initial: Initial attitude and rate state.
        control: Attitude control law configuration.

    Example:
        ```toml
        [satellite.attitude]

        [satellite.attitude.initial]
        sigma_BN = [0.1, 0.2, -0.3]
        omega_BN_B = [0.001, -0.01, 0.03]

        [satellite.attitude.control]
        type = "mrp_feedback"

        [satellite.attitude.control.mrp_feedback]
        k = 3.5
        p = 30.0
        ```
    """

    initial: Optional[AttitudeInitialCondition] = None
    control: Optional[AttitudeControlConfig] = None