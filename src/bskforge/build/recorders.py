#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Recorder build phase: attach Basilisk recorders for configured streams."""

from __future__ import annotations

from typing import Any

from Basilisk.utilities import simHelpers

from bskforge.build.context import RecorderSpec, SimContext, SpacecraftBundle
from bskforge.schema.recorder import RecorderConfig


def build_recorders(ctx: SimContext) -> None:
    """Attach recorders for state, control torque, RW speeds and sensors."""
    for bundle in ctx.bundles.values():
        name = bundle.config.name

        _record(
            ctx,
            bundle,
            f"{name}.state",
            bundle.sc.scStateOutMsg,
            bundle.config.logging.state,
        )

        if "control" in bundle.fsw:
            _record(
                ctx,
                bundle,
                f"{name}.control_torque",
                bundle.fsw["control"].cmdTorqueOutMsg,
                bundle.config.logging.control_torque,
            )

        if bundle.rw is not None:
            cfg = bundle.config.logging.rw_speeds or bundle.rw.recorder_cfg
            _record(
                ctx,
                bundle,
                f"{name}.rw_speeds",
                bundle.rw.effector.rwSpeedOutMsg,
                cfg,
            )

        for sensor_name, (msg, rec_cfg) in bundle.sensor_msgs.items():
            _record(ctx, bundle, f"{name}.{sensor_name}", msg, rec_cfg)


def _record(
    ctx: SimContext,
    bundle: SpacecraftBundle,
    key: str,
    msg: Any,
    cfg: RecorderConfig | None,
) -> None:
    """Create one recorder on ``msg`` and register it for export."""
    if cfg is None or not cfg.enabled:
        return
    sample_time = cfg.sample_time
    if sample_time is None:
        sample_time = simHelpers.samplingTime(
            ctx.stop_time_ns(),
            ctx.dyn_rate_ns(),
            ctx.config.logging.default_num_points,
        )
    recorder = msg.recorder(int(sample_time))
    ctx.add_dyn(recorder, priority=100)
    bundle.recorders.append(RecorderSpec(key=key, recorder=recorder))
