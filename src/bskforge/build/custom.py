#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Layer-3 build phase: ``[custom]`` Basilisk-native primitives."""

from __future__ import annotations

import importlib
from typing import Any, Callable

from Basilisk.utilities import macros

from bskforge.build.context import DYN_TASK, SimContext
from bskforge.build.registry import apply_params
from bskforge.errors import BuildError
from bskforge.schema.custom import CustomEvent, CustomModule


def build_custom_tasks(ctx: SimContext) -> None:
    """Create user-declared tasks (and their processes) before any modules."""
    for task_cfg in ctx.config.custom.tasks:
        process = ctx.ensure_process(task_cfg.process, task_cfg.process_priority)
        task = ctx.sim.CreateNewTask(
            task_cfg.name, macros.sec2nano(task_cfg.rate)
        )
        process.addTask(task)
        ctx._tasks[task_cfg.name] = task_cfg.name  # noqa: SLF001


def build_custom_modules(ctx: SimContext) -> None:
    """Instantiate ``[[custom.modules]]`` classes and place them on tasks."""
    for mod_cfg in ctx.config.custom.modules:
        module = _instantiate(mod_cfg)
        module.ModelTag = mod_cfg.tag
        apply_params(module, mod_cfg.params, f"custom module {mod_cfg.tag!r}")
        ctx.add_to_task(mod_cfg.task or DYN_TASK, module, mod_cfg.priority)
        ctx.register_tag(mod_cfg.tag, module)


def build_custom_connections(ctx: SimContext) -> None:
    """Subscribe ``[[custom.connections]]`` reader inputs to writer outputs."""
    for conn in ctx.config.custom.connections:
        writer_tag, out_name = _split_endpoint(conn.source, "source")
        reader_tag, in_name = _split_endpoint(conn.destination, "destination")
        writer_msg = _message(ctx, writer_tag, out_name)
        reader_msg = _message(ctx, reader_tag, in_name)
        reader_msg.subscribeTo(writer_msg)


def build_custom_events(ctx: SimContext) -> None:
    """Register ``[[custom.events]]`` on the simulation."""
    for event_cfg in ctx.config.custom.events:
        condition = _condition_fn(event_cfg)
        effect = _hook(event_cfg.effect, "effect") if event_cfg.effect else _noop
        ctx.sim.createNewEvent(
            event_cfg.name,
            macros.sec2nano(event_cfg.rate),
            condition,
            effect,
            event_cfg.transition,
        )


def apply_simulation_options(ctx: SimContext) -> None:
    """Apply ``[custom].simulation_options`` onto the SimBaseClass object."""
    apply_params(
        ctx.sim,
        ctx.config.custom.simulation_options,
        "custom.simulation_options",
    )


# ---------------------------------------------------------------------------


def _instantiate(mod_cfg: CustomModule) -> Any:
    """Import ``module:class`` and instantiate, with helpful errors."""
    target = mod_cfg.module
    parts = target.rsplit(".", 1)
    if len(parts) != 2:
        raise BuildError(
            f"custom module path {target!r} must look like "
            "'Basilisk.simulation.simpleNav.SimpleNav'"
        )
    module_path, class_name = parts
    try:
        module = importlib.import_module(module_path)
    except ImportError as exc:
        raise BuildError(
            f"cannot import custom module path {module_path!r}: {exc}"
        ) from exc
    cls = getattr(module, class_name, None)
    if cls is None:
        raise BuildError(
            f"{module_path!r} has no attribute {class_name!r}; check the "
            "Basilisk module reference for this build"
        )
    return cls()


def _split_endpoint(endpoint: str, side: str) -> tuple[str, str]:
    """Split ``"tag.messageName"`` into its tag and message field name."""
    if "." not in endpoint:
        raise BuildError(
            f"custom connection {side} {endpoint!r} must be "
            '"tag.messageName"'
        )
    tag, field = endpoint.rsplit(".", 1)
    return tag, field


def _message(ctx: SimContext, tag: str, msg_name: str) -> Any:
    """Resolve ``(tag, msg_name)`` to the live message object."""
    module = ctx.modules_by_tag.get(tag)
    if module is None:
        known = ", ".join(sorted(ctx.modules_by_tag))
        raise BuildError(
            f"custom connection references unknown tag {tag!r}; known tags: "
            f"{known}"
        )
    msg = getattr(module, msg_name, None)
    if msg is None:
        raise BuildError(
            f"module {tag!r} has no message {msg_name!r} on this Basilisk "
            "build"
        )
    return msg


def _condition_fn(event_cfg: CustomEvent) -> Callable[..., Any]:
    """Build the event condition callable from time or hook."""
    if event_cfg.condition:
        return _hook(event_cfg.condition, "condition")
    if event_cfg.condition_time is not None:
        trigger = event_cfg.condition_time

        def _time_condition(event_write_data: Any) -> list[Any]:
            return [event_write_data.sim_timeSec >= trigger, True]

        return _time_condition
    raise BuildError(
        f"event {event_cfg.name!r} needs condition_time or condition"
    )


def _hook(import_path: str, role: str) -> Callable[..., Any]:
    """Resolve a ``pkg.module:function`` hook reference."""
    if ":" not in import_path:
        raise BuildError(
            f"event {role} hook {import_path!r} must be 'pkg.module:function'"
        )
    module_path, fn_name = import_path.rsplit(":", 1)
    try:
        module = importlib.import_module(module_path)
    except ImportError as exc:
        raise BuildError(f"cannot import event {role} hook: {exc}") from exc
    fn = getattr(module, fn_name, None)
    if fn is None or not callable(fn):
        raise BuildError(f"event {role} hook {fn_name!r} not callable")
    return fn


def _noop(event_write_data: Any) -> None:  # noqa: ARG001
    """Default no-op event effect."""
