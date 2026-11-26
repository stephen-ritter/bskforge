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
from bskforge.schema.comm import CommLinkConfig
from bskforge.schema.custom import CustomSection
from bskforge.schema.environment import EnvironmentSection
from bskforge.schema.formation import FormationConfig
from bskforge.schema.logging import LoggingSection
from bskforge.schema.monte_carlo import MonteCarloSection
from bskforge.schema.simulation import SimulationSection
from bskforge.schema.spacecraft import SpacecraftSection
from bskforge.schema.visualization import CameraSection, VisualizationSection
from bskforge.schema.world import WorldSection


class ScenarioConfig(ScenarioModel):
    """Root of a ``bskforge`` scenario document.

    Top-level tables: ``[simulation]``, ``[world]``, ``[environment]``,
    ``[[spacecraft]]`` (repeatable), ``[logging]``, ``[visualization]``,
    ``[[camera]]`` (repeatable). A scenario may inherit defaults from
    another file via ``extends = "base.toml"`` (resolved by
    :mod:`bskforge.loader`).
    """

    simulation: SimulationSection = Field(default_factory=SimulationSection)
    world: WorldSection = Field(default_factory=WorldSection)
    environment: EnvironmentSection | None = None
    spacecraft: list[SpacecraftSection] = Field(default_factory=list)
    comm_links: list[CommLinkConfig] = Field(default_factory=list)
    formations: list[FormationConfig] = Field(default_factory=list)
    logging: LoggingSection = Field(default_factory=LoggingSection)
    visualization: VisualizationSection = Field(default_factory=VisualizationSection)
    camera: list[CameraSection] = Field(default_factory=list)
    custom: CustomSection = Field(default_factory=CustomSection)
    monte_carlo: MonteCarloSection | None = None

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
        formation_names = [form.name for form in self.formations]
        if len(formation_names) != len(set(formation_names)):
            dupes = sorted({n for n in formation_names if formation_names.count(n) > 1})
            raise ValueError(f"duplicate formation names: {', '.join(dupes)}")
        for form in self.formations:
            missing = [m for m in form.members if m not in names]
            if missing:
                raise ValueError(
                    f"formation {form.name!r}: members {missing} are not "
                    "[[spacecraft]] names"
                )
        link_names = [link.name for link in self.comm_links]
        if len(link_names) != len(set(link_names)):
            dupes = sorted({n for n in link_names if link_names.count(n) > 1})
            raise ValueError(f"duplicate comm link names: {', '.join(dupes)}")
        for link in self.comm_links:
            if link.transmitter not in names:
                raise ValueError(
                    f"comm link {link.name!r}: transmitter {link.transmitter!r} "
                    "is not a [[spacecraft]] name"
                )
            if link.type == "crosslink" and link.receiver not in names:
                raise ValueError(
                    f"comm link {link.name!r}: receiver {link.receiver!r} "
                    "is not a [[spacecraft]] name"
                )
            if link.type == "crosslink" and link.transmitter == link.receiver:
                raise ValueError(
                    f"comm link {link.name!r}: transmitter and receiver must differ"
                )
            if link.type == "downlink" and not self.world.spice.enabled:
                raise ValueError(
                    f"comm link {link.name!r}: a ground station rides a "
                    "rotating planet; add a [world.spice] section"
                )
        if self.monte_carlo is not None:
            for disp in self.monte_carlo.dispersions:
                if disp.spacecraft not in names:
                    raise ValueError(
                        f"monte_carlo dispersion on {disp.spacecraft!r}: "
                        "not a [[spacecraft]] name"
                    )
        if self.camera:
            if not self.visualization.enabled:
                raise ValueError(
                    "[[camera]] frames are rendered by Vizard; add a "
                    "[visualization] section with enabled = true"
                )
            cam_ids = [cam.camera_id for cam in self.camera]
            if len(cam_ids) != len(set(cam_ids)):
                raise ValueError("camera_id must be unique across [[camera]] entries")
            for cam in self.camera:
                if cam.parent not in names:
                    raise ValueError(
                        f"camera {cam.camera_id}: parent {cam.parent!r} "
                        "is not a [[spacecraft]] name"
                    )
                if cam.downlink and cam.record_rate == 0:
                    raise ValueError(
                        f"camera {cam.camera_id}: downlink = true with "
                        "record_rate = 0 stores nothing, so nothing is "
                        "eligible to downlink"
                    )
                if cam.record_rate and (
                    cam.store_bytes is None or cam.store_files is None
                ):
                    raise ValueError(
                        f"camera {cam.camera_id}: record_rate = {cam.record_rate} "
                        "opens a store with no declared capacity - set store_bytes "
                        "(the volume) and store_files (the catalog's reach)"
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
