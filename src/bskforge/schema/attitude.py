#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Attitude initial-condition models (``[spacecraft.attitude]``)."""

from __future__ import annotations

from bskforge.schema.base import ScenarioModel, Vec3


class AttitudeInitial(ScenarioModel):
    """Initial attitude and rate state.

    Attributes:
        sigma_BN: Modified Rodrigues parameters of body relative to inertial.
        omega_BN_B: Body angular rate about the body-frame axes.
    """

    sigma_BN: Vec3 = [0.0, 0.0, 0.0]
    omega_BN_B: Vec3 = [0.0, 0.0, 0.0]


class AttitudeConfig(ScenarioModel):
    """``[spacecraft.attitude]``: rotational initial conditions."""

    initial: AttitudeInitial | None = None
