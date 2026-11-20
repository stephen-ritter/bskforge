#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Base schema utilities shared by all scenario models."""

from __future__ import annotations

from typing import Annotated, Any

from pydantic import AfterValidator, BaseModel, ConfigDict

from bskforge.errors import ConfigError


class ScenarioModel(BaseModel):
    """Strict base model: unknown keys are rejected with helpful errors.

    The loader converts ``extra="forbid"`` violations into did-you-mean
    suggestions; see :mod:`bskforge.loader`.
    """

    model_config = ConfigDict(extra="forbid")


class PassthroughModel(ScenarioModel):
    """Model that carries a free-form ``params`` table.

    ``params`` are applied to the underlying Basilisk object at build time.
    For module-level objects they are applied with ``setattr`` after checking
    the attribute name against the installed Basilisk build; for factory
    helpers (e.g. RW wheels) they are forwarded as keyword arguments.
    """

    params: dict[str, Any] = {}


def _exactly3(value: list[float]) -> list[float]:
    if len(value) != 3:
        raise ConfigError(f"expected 3 components, got {len(value)}: {value}")
    return value


Vec3 = Annotated[list[float], AfterValidator(_exactly3)]
"""A 3-component float vector (positions, velocities, directions, gains)."""
