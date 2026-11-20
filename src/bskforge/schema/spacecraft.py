#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Spacecraft models: ``[[spacecraft]]`` and the hub rigid body."""

from __future__ import annotations

from typing import Any

from pydantic import Field, field_validator, model_validator

from bskforge.schema.attitude import AttitudeConfig
from bskforge.schema.base import PassthroughModel, ScenarioModel, Vec3
from bskforge.schema.effectors import (
    DisturbanceConfig,
    EffectorConfig,
    ReactionWheelsConfig,
)
from bskforge.schema.environment import EnvironmentSection
from bskforge.schema.fsw import FswSection
from bskforge.schema.orbit import OrbitConfig
from bskforge.schema.recorder import RecorderConfig
from bskforge.schema.sensors import CssConstellationConfig, SensorConfig
from bskforge.schema.visualization import SpacecraftViz


class HubConfig(PassthroughModel):
    """Rigid-body hub properties: ``[spacecraft.hub]``.

    Attributes:
        mass: Hub mass ``mHub``.
        inertia: Inertia tensor about the hub center of mass, either as a
            symmetric table ``{ixx, iyy, izz, ixy, ixz, iyz}`` or a flat
            row-major list of 9 values.
        r_BcB_B: Body-fixed point offset from the hub center of mass.
    """

    mass: float = Field(1.0, gt=0, description="mHub [kg]")
    inertia: Any = Field(
        default_factory=lambda: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        description="IHubPntBc_B [kg*m^2]",
    )
    r_BcB_B: Vec3 = [0.0, 0.0, 0.0]

    @field_validator("inertia", mode="before")
    @classmethod
    def _normalize_inertia(cls, value: Any) -> Any:
        """Accept a symmetric table or a flat 9-vector; store the flat form."""
        if value is None:
            return [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        if isinstance(value, dict):
            diag = {
                k: float(value.get(k, 0.0))
                for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
            }
            ixx, iyy, izz = diag["ixx"], diag["iyy"], diag["izz"]
            ixy, ixz, iyz = diag["ixy"], diag["ixz"], diag["iyz"]
            return [ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz]
        if isinstance(value, (list, tuple)) and len(value) == 9:
            return [float(v) for v in value]
        raise ValueError(
            "inertia must be a table {ixx, iyy, izz, ixy, ixz, iyz} "
            "or a flat list of 9 values"
        )


class SpacecraftLogging(ScenarioModel):
    """Per-message recorder switches: ``[spacecraft.logging]``.

    Attributes:
        state: Recorder for ``scStateOutMsg``.
        control_torque: Recorder for the control-law commanded torque.
        rw_speeds: Overrides the reaction-wheel effector's own recorder.
    """

    state: RecorderConfig = Field(default_factory=RecorderConfig)
    control_torque: RecorderConfig = Field(default_factory=RecorderConfig)
    rw_speeds: RecorderConfig | None = None


class SpacecraftSection(ScenarioModel):
    """``[[spacecraft]]``: one complete space vehicle.

    Attributes:
        name: Unique vehicle name (used as Basilisk ModelTag prefix).
        hub: Rigid-body hub.
        orbit: Translational initial state.
        attitude: Rotational initial state.
        sensors: Sensor payload list.
        css_constellations: CSS groups feeding sunline estimators.
        effectors: Control effector list.
        disturbances: Environmental disturbance effectors.
        fsw: Flight-software stack.
        environment: Per-vehicle override of the global ``[environment]``.
        logging: Recorder switches for this vehicle.
        viz: Per-vehicle Vizard hints.
    """

    name: str
    hub: HubConfig = Field(default_factory=HubConfig)
    orbit: OrbitConfig | None = None
    attitude: AttitudeConfig | None = None
    sensors: list[SensorConfig] = Field(default_factory=list)
    css_constellations: list[CssConstellationConfig] = Field(default_factory=list)
    effectors: list[EffectorConfig] = Field(default_factory=list)
    disturbances: list[DisturbanceConfig] = Field(default_factory=list)
    fsw: FswSection | None = None
    environment: EnvironmentSection | None = None
    logging: SpacecraftLogging = Field(default_factory=SpacecraftLogging)
    viz: SpacecraftViz = Field(default_factory=SpacecraftViz)

    @model_validator(mode="after")
    def _unique_names(self) -> SpacecraftSection:
        for group in (self.sensors, self.effectors):
            names = [
                getattr(item, "name", None) or getattr(item, "type", "")
                for item in group
            ]
            dupes = {n for n in names if names.count(n) > 1}
            if dupes:
                raise ValueError(
                    f"duplicate component names in spacecraft "
                    f"{self.name!r}: {', '.join(sorted(dupes))}"
                )
        css_names = {s.name for s in self.sensors if s.type == "coarseSunSensor"}
        cluster_names = [c.name for c in self.css_constellations]
        if len(cluster_names) != len(set(cluster_names)):
            raise ValueError(
                f"duplicate css_constellations names in spacecraft {self.name!r}"
            )
        for cluster in self.css_constellations:
            unknown = [m for m in cluster.members if m not in css_names]
            if unknown:
                raise ValueError(
                    f"spacecraft {self.name!r}: css_constellation "
                    f"{cluster.name!r} references unknown coarse sun "
                    f"sensors: {', '.join(unknown)}"
                )
        return self

    def reaction_wheels(self) -> ReactionWheelsConfig | None:
        """Return the RW effector config for this vehicle, if any."""
        for effector in self.effectors:
            if effector.type == "reactionWheels":
                return effector
        return None
