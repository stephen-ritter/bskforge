#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Visualization build phase: Vizard via ``vizSupport``."""

from __future__ import annotations

import os
from pathlib import Path

from Basilisk.simulation import simSynch
from Basilisk.utilities import vizSupport

from bskforge.build.context import DYN_TASK, SimContext
from bskforge.errors import BuildError


def build_viz(ctx: SimContext) -> None:
    """Attach the Vizard interface module when visualization is enabled.

    Vehicles, RW effectors and per-vehicle model-dictionary keys are passed
    as parallel lists. Live streaming additionally registers a clock-sync
    module so Vizard pacing matches the simulated clock. The interface
    handle is retained on the context so the camera phase (which runs
    after this one) can register instruments against it.
    """
    viz_cfg = ctx.config.visualization
    if not viz_cfg.enabled:
        return
    if not getattr(vizSupport, "vizFound", False):
        if ctx.config.camera:
            raise BuildError(
                "[[camera]] frames are rendered by Vizard, but this "
                "Basilisk build has no vizInterface support"
            )
        import warnings

        warnings.warn(
            "visualization requested but this Basilisk build has no "
            "vizInterface support; continuing without visualization",
            stacklevel=2,
        )
        return

    bundles = list(ctx.bundles.values())
    kwargs = {
        "liveStream": viz_cfg.live_stream,
        "broadcastStream": viz_cfg.broadcast_stream,
        "noDisplay": viz_cfg.no_display,
    }
    if viz_cfg.save_file is not None:
        kwargs["saveFile"] = viz_cfg.save_file

    if viz_cfg.live_stream:
        clock = simSynch.ClockSynch()
        clock.accelFactor = 1.0  # real-time pacing
        ctx.add_dyn(clock)

    # Vizard's patience (the ~35 s message-load timeout) must start
    # counting when this world starts waiting for it, not when the
    # renderer's container booted. A launcher that runs the renderer
    # beside the world gates it on this sentinel: unlink-then-write so
    # the mtime always says "the flight now running entered its wait".
    wait_file = os.environ.get("BSKFORGE_VIZ_WAIT_FILE", "")
    if wait_file:
        marker = Path(wait_file)
        marker.parent.mkdir(parents=True, exist_ok=True)
        marker.unlink(missing_ok=True)
        marker.write_bytes(b"")

    ctx.viz = vizSupport.enableUnityVisualization(
        ctx.sim,
        DYN_TASK,
        [bundle.sc for bundle in bundles],
        # one entry per spacecraft: the RW effector module (or None);
        # vizSupport reads ``rwEffectorList[c].rwOutMsgs`` directly
        rwEffectorList=[
            bundle.rw.effector if bundle.rw is not None else None for bundle in bundles
        ],
        modelDictionaryKeyList=[
            bundle.config.viz.model_key or "" for bundle in bundles
        ],
        **kwargs,
    )
