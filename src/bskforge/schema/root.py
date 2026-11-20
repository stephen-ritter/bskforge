#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Root scenario model: the parsed and validated scenario document."""

from __future__ import annotations

from pydantic import Field, PrivateAttr, model_validator

from bskforge.schema.base import ScenarioModel
from bskforge.schema.custom import CustomSection
from bskforge.schema.environment import EnvironmentSection
from bskforge.schema.logging import LoggingSection
from bskforge.schema.simulation import SimulationSection
from bskforge.schema.spacecraft import SpacecraftSection
from bskforge.schema.visualization import VisualizationSection
from bskforge.schema.world import WorldSection


class ScenarioConfig(ScenarioModel):
    """Root of a ``bskforge`` scenario document.

    Top-level tables: ``[simulation]``, ``[world]``, ``[environment]``,
    ``[[spacecraft]]`` (repeatable), ``[logging]``, ``[visualization]``.
    A scenario may inherit defaults from another file via
    ``extends = "base.toml"`` (resolved by :mod:`bskforge.loader`).
    """

    simulation: SimulationSection = Field(default_factory=SimulationSection)
    world: WorldSection = Field(default_factory=WorldSection)
    environment: EnvironmentSection | None = None
    spacecraft: list[SpacecraftSection] = Field(default_factory=list)
    logging: LoggingSection = Field(default_factory=LoggingSection)
    visualization: VisualizationSection = Field(default_factory=VisualizationSection)
    custom: CustomSection = Field(default_factory=CustomSection)
    monte_carlo: dict | None = None

    #: scenario file the document was loaded from (``None`` for dicts)
    _source_path: str | None = PrivateAttr(default=None)

    @property
    def source_path(self) -> str | None:
        """Path of the scenario file this document was loaded from."""
        return self._source_path

    def set_source_path(self, path: str) -> None:
        """Record the scenario file path (called by :mod:`bskforge.loader`)."""
        self._source_path = path

    @model_validator(mode="after")
    def _cross_check(self) -> ScenarioConfig:
        names = [sc.name for sc in self.spacecraft]
        if len(names) != len(set(names)):
            dupes = sorted({n for n in names if names.count(n) > 1})
            raise ValueError(f"duplicate spacecraft names: {', '.join(dupes)}")
        for sc in self.spacecraft:
            if sc.fsw is not None and sc.fsw.allocation is not None:
                effector_type = {
                    "rwMotorTorque": "reactionWheels",
                    "thrusterTorque": "thrusters",
                    "dipoleMapping": "mtb",
                }[sc.fsw.allocation.type]
                if not sc.has_effector(effector_type):
                    raise ValueError(
                        f"spacecraft {sc.name!r} declares an fsw allocation "
                        f"but has no {effector_type} effector to allocate to"
                    )
            if sc.fsw is not None and sc.fsw.momentum_management is not None:
                kind = sc.fsw.momentum_management.type
                needed = (
                    ("reactionWheels", "thrusters")
                    if kind == "thrMomentumDumping"
                    else ("reactionWheels", "mtb")
                )
                if not all(sc.has_effector(name) for name in needed):
                    raise ValueError(
                        f"spacecraft {sc.name!r}: {kind} requires "
                        f"{' and '.join(needed)} effectors"
                    )
            if sc.fsw is not None and sc.fsw.guidance is not None:
                guid = sc.fsw.guidance
                needs_spice = guid.type == "celestialTwoBodyPoint" or (
                    guid.type == "locationPointing" and guid.target_latlon is not None
                )
                if needs_spice and not self.world.spice.enabled:
                    raise ValueError(
                        f"spacecraft {sc.name!r}: guidance "
                        f"{guid.type!r} needs SPICE ephemerides; add a "
                        "[world.spice] section"
                    )
        if self.monte_carlo:
            raise ValueError(
                "monte_carlo requires Phase 6; remove [monte_carlo] to run "
                "the deterministic scenario"
            )
        return self

    def primary_vehicle(self) -> SpacecraftSection:
        """Return the first configured vehicle.

        Raises:
            ValueError: If the scenario has no spacecraft.
        """
        if not self.spacecraft:
            raise ValueError("scenario defines no [[spacecraft]]")
        return self.spacecraft[0]
