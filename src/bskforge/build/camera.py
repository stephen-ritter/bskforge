#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Camera build phase: Vizard-rendered instruments via the ``camera`` module.

Runs after :func:`bskforge.build.viz.build_viz` (which retains the interface
handle on the context) and registers each ``[[camera]]`` entry so Vizard
renders into it. The ordering here is a Basilisk contract, not a style
choice: the camera parameters must be set *before* ``addCamMsgToModule``
(that call allocates an image slot), and ``imageInMsg`` may subscribe only
*after* it - the slot it subscribes to does not exist until then.
"""

from __future__ import annotations

from Basilisk.simulation import camera
from Basilisk.utilities import macros

from bskforge.build.context import DYN_PROCESS, SimContext
from bskforge.errors import BuildError


def build_cameras(ctx: SimContext) -> None:
    """Attach each ``[[camera]]`` entry to the Vizard interface.

    Args:
        ctx: Build context whose ``viz`` handle was set by the
            visualization phase.

    Raises:
        BuildError: A camera is configured but no Vizard interface was
            built (the schema guarantees ``[visualization].enabled`` for a
            non-empty ``[[camera]]``, so this only fires when the interface
            build was skipped for another reason).
    """
    if not ctx.config.camera:
        return
    if ctx.viz is None:
        raise BuildError(
            "[[camera]] requires the Vizard interface; enable visualization "
            "and ensure this Basilisk build carries vizInterface support"
        )

    process = ctx.ensure_process(DYN_PROCESS, priority=3)
    for index, cam in enumerate(ctx.config.camera):
        module = camera.Camera()
        module.ModelTag = f"{cam.parent}_camera{cam.camera_id}"
        module.cameraIsOn = 1
        module.cameraID = cam.camera_id
        module.renderRate = int(macros.sec2nano(cam.render_rate))
        module.sigma_CB = list(cam.sigma_CB)
        module.cameraPos_B = list(cam.camera_pos_B)
        module.resolution = list(cam.resolution)
        module.fieldOfView = cam.field_of_view
        module.parentName = cam.parent
        module.skyBox = cam.sky_box
        if cam.save_images:
            module.saveImages = 1
            if cam.save_dir is not None:
                module.saveDir = cam.save_dir

        # Register with Vizard FIRST: this allocates opnavImageOutMsgs[index].
        ctx.viz.addCamMsgToModule(module.cameraConfigOutMsg)
        # NOW the slot exists; subscribe the image input to it.
        module.imageInMsg.subscribeTo(ctx.viz.opnavImageOutMsgs[index])

        # The camera rides its own task, paced at the render rate.
        task_name = f"cameraTask{index}"
        process.addTask(
            ctx.sim.CreateNewTask(task_name, macros.sec2nano(cam.render_rate))
        )
        ctx._tasks[task_name] = task_name
        ctx.add_to_task(task_name, module)
        ctx.register_tag(module.ModelTag, module)
