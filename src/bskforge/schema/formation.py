#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Formation flying: barycenter propagation and relative Hill states."""

from __future__ import annotations

from pydantic import Field, model_validator

from bskforge.schema.base import ScenarioModel


class FormationConfig(ScenarioModel):
    """``[[formations]]``: a named vehicle group with relative states.

    The build records the formation's mass-averaged barycenter state
    and, for every member, the deputy-relative state in the
    ``relative_to`` vehicle's Hill frame.

    Attributes:
        name: Formation name; prefixes the recorded streams.
        members: Spacecraft names belonging to the formation.
        relative_to: Hill-frame chief; defaults to the first member.
        barycenter: Record the equinoctial-averaged barycenter state.
    """

    name: str
    members: list[str] = Field(min_length=2)
    relative_to: str | None = None
    barycenter: bool = True

    @model_validator(mode="after")
    def _members_unique(self) -> FormationConfig:
        if len(self.members) != len(set(self.members)):
            dupes = sorted({m for m in self.members if self.members.count(m) > 1})
            raise ValueError(f"formation {self.name!r}: duplicate members {dupes}")
        if self.relative_to is not None and self.relative_to not in self.members:
            raise ValueError(
                f"formation {self.name!r}: relative_to "
                f"{self.relative_to!r} is not a member"
            )
        return self
