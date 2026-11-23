#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Visualization build phase: Vizard via ``vizSupport``."""

from __future__ import annotations

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
