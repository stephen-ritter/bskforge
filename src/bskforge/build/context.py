#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Build-time context shared by all builder phases."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Optional

from Basilisk.architecture import messaging
from Basilisk.utilities import SimulationBaseClass, macros

from bskforge.errors import BuildError
from bskforge.schema.root import ScenarioConfig
from bskforge.schema.spacecraft import SpacecraftSection

DYN_PROCESS = "dynamicsProcess"
DYN_TASK = "dynamicsTask"
FSW_PROCESS = "fswProcess"
FSW_TASK = "fswTask"


@dataclass
class RecorderSpec:
    """A registered recorder: output name and Basilisk recorder module."""

    key: str
    recorder: Any


@dataclass
class RwBundle:
    """Reaction-wheel effector handles needed by later phases."""

    effector: Any
    factory: Any
    cfg_msg: Any
    recorder_cfg: Any = None


@dataclass
class SpacecraftBundle:
    """Per-vehicle state assembled across builder phases."""

    config: SpacecraftSection
    sc: Any
    ext_ft: Any
    veh_config_msg: Any
    rw: Optional[RwBundle] = None
    fsw: dict[str, Any] = field(default_factory=dict)
    sensor_msgs: dict[str, tuple[Any, Any]] = field(default_factory=dict)
    recorders: list[RecorderSpec] = field(default_factory=list)


class SimContext:
    """Owns the Basilisk ``SimBaseClass`` and everything assembled onto it.

    Attributes:
        config: Validated scenario configuration.
        sim: The Basilisk simulation object under construction.
        bundles: Per-vehicle bundles keyed by vehicle name.
        grav_factory: The shared ``gravBodyFactory``.
        central_body: ``BodyData`` of the central gravitating body.
    """

    def __init__(self, config: ScenarioConfig) -> None:
        self.config = config
        self.sim = SimulationBaseClass.SimBaseClass()
        self.bundles: dict[str, SpacecraftBundle] = {}
        self.grav_factory: Any = None
        self.central_body: Any = None
        self.central_mu: float = 0.0
        self._sun_msg: Any = None
        self.modules_by_tag: dict[str, Any] = {}
        self.processes: dict[str, Any] = {}
        self._tasks: dict[str, str] = {}

        self._create_processes()
        self._tasks[DYN_TASK] = DYN_TASK
        self._tasks[FSW_TASK] = FSW_TASK

    def _create_processes(self) -> None:
        """Create the dynamics (priority 1) and FSW (priority 2) processes."""
        sim = self.sim
        dyn_process = sim.CreateNewProcess(DYN_PROCESS, priority=1)
        self.processes[DYN_PROCESS] = dyn_process
        dyn_process.addTask(
            sim.CreateNewTask(
                DYN_TASK, macros.sec2nano(self.config.simulation.dt.dynamics)
            )
        )
        fsw_process = sim.CreateNewProcess(FSW_PROCESS, priority=2)
        self.processes[FSW_PROCESS] = fsw_process
        fsw_process.addTask(
            sim.CreateNewTask(
                FSW_TASK, macros.sec2nano(self.config.simulation.dt.fsw)
            )
        )

    # -- task helpers ------------------------------------------------------

    def add_dyn(self, model: Any, priority: int = -1) -> None:
        """Add ``model`` to the dynamics task."""
        self.sim.AddModelToTask(DYN_TASK, model, ModelPriority=priority)

    def add_fsw(self, model: Any, priority: int = -1) -> None:
        """Add ``model`` to the FSW task."""
        self.sim.AddModelToTask(FSW_TASK, model, ModelPriority=priority)

    def add_to_task(self, task_name: str, model: Any, priority: int = -1) -> None:
        """Add ``model`` to a named task (``dynamicsTask``/``fswTask``/custom).

        Raises:
            BuildError: The named task was never created.
        """
        if task_name not in self.tasks:
            known = ", ".join(sorted(self.tasks))
            raise BuildError(
                f"unknown task {task_name!r}; defined tasks: {known}"
            )
        self.sim.AddModelToTask(task_name, model, ModelPriority=priority)

    def ensure_process(self, name: str, priority: int) -> Any:
        """Return the named process, creating it if it does not exist yet.

        Args:
            name: Process name.
            priority: Priority used if the process must be created.

        Returns:
            The Basilisk process object.
        """
        if name not in self.processes:
            self.processes[name] = self.sim.CreateNewProcess(
                name, priority=priority
            )
        return self.processes[name]

    def register_tag(self, tag: str, module: Any) -> None:
        """Register a module under ``tag`` so ``[custom]`` can address it.

        Args:
            tag: ModelTag-style handle used in ``[custom.connections]``.
            module: The Basilisk module instance.

        Raises:
            BuildError: The tag is already registered.
        """
        if tag in self.modules_by_tag:
            raise BuildError(f"duplicate module tag {tag!r}")
        self.modules_by_tag[tag] = module

    @property
    def tasks(self) -> dict[str, str]:
        """Map of logical task name to Basilisk task name (identity today)."""
        return self._tasks

    @property
    def central_grav_bodies(self) -> list[Any]:
        """All gravity bodies created by the world phase (for addBodiesTo)."""
        return list(self.grav_factory.gravBodies) if self.grav_factory else []

    def stop_time_ns(self) -> int:
        """Return the configured scenario duration in nanoseconds."""
        return macros.sec2nano(self.config.simulation.duration)

    def dyn_rate_ns(self) -> int:
        """Return the dynamics task rate in nanoseconds."""
        return macros.sec2nano(self.config.simulation.dt.dynamics)

    # -- shared messages -----------------------------------------------------

    def ensure_sun_msg(self) -> Any:
        """Return the shared sun ephemeris message, creating it on demand.

        The sun position is the fixed inertial vector from ``[world.sun]``;
        SPICE-driven sun ephemerides replace this in Phase 2.
        """
        if self._sun_msg is None:
            payload = messaging.SpicePlanetStateMsgPayload()
            payload.PositionVector = list(self.config.world.sun.position)
            msg = messaging.SpicePlanetStateMsg()
            msg.write(payload)
            self._sun_msg = msg
        return self._sun_msg
