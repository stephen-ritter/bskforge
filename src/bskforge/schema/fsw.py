#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Flight-software stack models: ``[spacecraft.fsw]``.

The FSW section is declarative: it names a *guidance*, *estimator*,
*control* and *allocation* module by ``type``, and the builder wires the
standard Basilisk message graph between them. Each block exposes a
``params`` table for module-specific knobs (matched against the installed
Basilisk module at build time).
"""

from __future__ import annotations

from typing import Annotated, Literal

from pydantic import Field

from bskforge.schema.base import PassthroughModel, Vec3


# --- Guidance --------------------------------------------------------------
class Inertial3DGuidance(PassthroughModel):
    """Inertial pointing reference: ``type = "inertial3D"``.

    Attributes:
        sigma_R0N: MRP of the reference frame relative to inertial.
    """

    type: Literal["inertial3D"] = "inertial3D"
    sigma_R0N: Vec3 = [0.0, 0.0, 0.0]


GuidanceConfig = Annotated[Inertial3DGuidance, Field(discriminator="type")]


# --- Estimator -------------------------------------------------------------
class SimpleNavEstimator(PassthroughModel):
    """Truth-pass-through navigation: ``type = "simpleNav"``."""

    type: Literal["simpleNav"] = "simpleNav"


EstimatorConfig = Annotated[SimpleNavEstimator, Field(discriminator="type")]


# --- Control ---------------------------------------------------------------
class MrpFeedbackControl(PassthroughModel):
    """MRP feedback attitude control: ``type = "mrpFeedback"``.

    Attributes:
        k: Proportional gain on the attitude error MRP.
        p: Proportional gain on the rate error.
        ki: Integral gain (negative disables integral action).
        integral_limit: Integral-term saturation; auto-derived when unset.
        known_torque: Feed-forward torque cancellation ``knownTorquePntB_B``.
    """

    type: Literal["mrpFeedback"] = "mrpFeedback"
    k: float = 3.5
    p: float = 30.0
    ki: float = -1.0
    integral_limit: float | None = Field(None, gt=0)
    known_torque: Vec3 = [0.0, 0.0, 0.0]

    def resolved_integral_limit(self) -> float:
        """Return the explicit limit, or derive it from ``ki``."""
        if self.integral_limit is not None:
            return self.integral_limit
        if self.ki < 0:
            return 0.0
        return 2.0 / self.ki * 0.1


ControlConfig = Annotated[MrpFeedbackControl, Field(discriminator="type")]


# --- Allocation ------------------------------------------------------------
class RwMotorTorqueAllocation(PassthroughModel):
    """Map commanded torque to RW motor torques: ``type = "rwMotorTorque"``."""

    type: Literal["rwMotorTorque"] = "rwMotorTorque"


AllocationConfig = Annotated[RwMotorTorqueAllocation, Field(discriminator="type")]


class FswSection(PassthroughModel):
    """``[spacecraft.fsw]``: the per-vehicle flight-software stack.

    Attributes:
        guidance: Attitude reference generator.
        estimator: Navigation/attitude estimate feeding the control law.
        control: Attitude control law producing a commanded body torque.
        allocation: Maps the commanded torque onto effector device commands.
    """

    guidance: GuidanceConfig | None = None
    estimator: EstimatorConfig | None = None
    control: ControlConfig | None = None
    allocation: AllocationConfig | None = None
