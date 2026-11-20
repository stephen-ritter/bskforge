#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Layer-3 escape hatch: ``[custom]`` Basilisk-native primitives.

The domain sections (``[[spacecraft]]`` …) compile down to the five
Basilisk primitives: instantiate a module, set its parameters, place it on a
task, subscribe messages, and react to events. The ``[custom]`` section
exposes those same primitives directly, so *any* capability of the installed
Basilisk build is expressible in a scenario file even before bskforge grows
dedicated domain support for it.
"""

from __future__ import annotations

from pydantic import Field

from bskforge.schema.base import ScenarioModel


class CustomTask(ScenarioModel):
    """A Basilisk task on a (possibly new) process: ``[[custom.tasks]]``.

    Attributes:
        name: Task name used by modules via ``task = "..."``.
        rate: Execution period.
        process: Process to host the task; created if new.
        process_priority: Priority for a newly created process
            (higher runs earlier within one sim step).
    """

    name: str
    rate: float = Field(gt=0, description="Task rate [s]")
    process: str = "customProcess"
    process_priority: int = 1


class CustomModule(ScenarioModel):
    """Any Basilisk module class: ``[[custom.modules]]``.

    Attributes:
        module: Import path to the class, e.g.
            ``"Basilisk.simulation.simpleNav.SimpleNav"``.
        tag: Unique ModelTag; connections address ``tag.messageName``.
        task: Task name to run the module on (defaults to the dynamics task).
        priority: Model priority on that task (higher runs later).
        params: Attributes applied via introspection-checked ``setattr``.
    """

    module: str
    tag: str
    task: str | None = None
    priority: int = -1
    params: dict = Field(default_factory=dict)


class CustomConnection(ScenarioModel):
    """A message subscription: ``[[custom.connections]]``.

    Attributes:
        source: ``"tag.outMsgName"`` writer.
        destination: ``"tag.inMsgName"`` reader.
    """

    source: str
    destination: str


class CustomEvent(ScenarioModel):
    """A Basilisk event: ``[[custom.events]]``.

    Attributes:
        name: Event tag.
        rate: Event evaluation period.
        condition_time: Trigger once simulated time reaches this value.
        condition: ``"pkg.module:function"`` hook overriding the time
            condition (receives the Basilisk ``eventWriteData``).
        effect: ``"pkg.module:function"`` hook run on trigger.
        transition: Basilisk transition type (``atExit`` or ``toFinalState``).
    """

    name: str
    rate: float = Field(1.0, gt=0, description="Evaluation period [s]")
    condition_time: float | None = Field(None, ge=0, description="Trigger time [s]")
    condition: str | None = None
    effect: str | None = None
    transition: str = "atExit"


class CustomSection(ScenarioModel):
    """``[custom]``: direct Basilisk graph primitives."""

    tasks: list[CustomTask] = Field(default_factory=list)
    modules: list[CustomModule] = Field(default_factory=list)
    connections: list[CustomConnection] = Field(default_factory=list)
    events: list[CustomEvent] = Field(default_factory=list)
    simulation_options: dict = Field(default_factory=dict)
