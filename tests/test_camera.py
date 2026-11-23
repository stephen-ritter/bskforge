#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""The Vizard camera: registration order is the contract.

A ``[[camera]]`` entry is only real if the module landed on a task at the
render rate, points from the declared vehicle, and subscribed to the image
slot its own registration allocated - the slot does not exist before
``addCamMsgToModule`` runs, so slot count after a build is the machine-
checkable trace of that ordering. These tests build only: the interface
waits for a Vizard connection at execution time, and no headless renderer
is assumed on a test box. The capture itself runs when ``BSKFORGE_VIZARD``
names a Vizard binary, and skips honestly otherwise.
"""

import os
import subprocess
import threading
import time
from pathlib import Path

import pytest
from Basilisk.utilities import macros

from bskforge.errors import BuildError, ConfigError
from bskforge.loader import validate_scenario
from bskforge.runner import ScenarioRunner

VIZARD = os.environ.get("BSKFORGE_VIZARD")


def camera_scenario(*, enabled=True, **camera_cfg):
    base = {
        "simulation": {"name": "cam", "duration": 8.0},
        "spacecraft": [{"name": "sc1"}],
        "visualization": {"enabled": enabled, "no_display": True},
        "camera": [dict({"parent": "sc1"}, **camera_cfg)],
    }
    return validate_scenario(base)


class TestValidation:
    def test_camera_without_visualization_refused(self):
        with pytest.raises(ConfigError, match=r"\[visualization\]"):
            camera_scenario(enabled=False, camera_id=1)

    def test_unknown_parent_refused(self):
        with pytest.raises(ConfigError, match="is not a \\[\\[spacecraft\\]\\] name"):
            camera_scenario(parent="ghost")

    def test_duplicate_camera_id_refused(self):
        with pytest.raises(ConfigError, match="unique"):
            validate_scenario(
                {
                    "simulation": {"name": "cam", "duration": 8.0},
                    "spacecraft": [{"name": "sc1"}],
                    "visualization": {"enabled": True},
                    "camera": [{"parent": "sc1"}, {"parent": "sc1"}],
                }
            )


class TestBuild:
    def test_camera_builds_onto_interface(self):
        built = ScenarioRunner(
            camera_scenario(
                camera_id=7,
                render_rate=1.5,
                resolution=(256, 256),
                sigma_CB=(0.1, 0.2, 0.3),
            )
        ).build()
        cam = built.context.modules_by_tag["sc1_camera7"]
        assert cam.cameraID == 7
        assert list(cam.resolution) == [256, 256]
        assert cam.parentName == "sc1"
        assert cam.renderRate == int(macros.sec2nano(1.5))

    def test_registration_allocates_slots_and_subscribes(self):
        built = ScenarioRunner(
            validate_scenario(
                {
                    "simulation": {"name": "cam", "duration": 8.0},
                    "spacecraft": [{"name": "sc1"}],
                    "visualization": {"enabled": True},
                    "camera": [
                        {"parent": "sc1", "camera_id": 1},
                        {"parent": "sc1", "camera_id": 2, "render_rate": 1.0},
                    ],
                }
            )
        ).build()
        # one slot per camera, allocated in registration order
        assert len(built.context.viz.opnavImageOutMsgs) == 2

    def test_camera_task_paces_at_render_rate(self):
        built = ScenarioRunner(camera_scenario(camera_id=1, render_rate=1.5)).build()
        tasks = {
            task.Name: int(task.TaskData.TaskPeriod)
            for task in built.context.sim.TaskList
        }
        assert tasks["cameraTask0"] == int(macros.sec2nano(1.5))


@pytest.mark.skipif(
    not VIZARD, reason="set BSKFORGE_VIZARD=/path/Vizard.x86_64 to capture"
)
def test_headless_capture_lands_images(tmp_path):
    """The full round trip: Vizard renders, the module receives, PNGs land."""
    assert VIZARD is not None  # the skipif guard above; this narrows the type
    binary = Path(VIZARD).resolve()
    config = camera_scenario(
        camera_id=1,
        render_rate=1.0,
        save_images=True,
        save_dir=str(tmp_path / "cameraImage"),
    )
    built = ScenarioRunner(config).build()
    sim = built.context.sim

    # Basilisk's init blocks until Vizard completes the handshake, so the
    # scenario runs on a thread while the main thread starts the renderer
    # a few seconds later - once the interface socket is bound and the
    # sim sits in "Waiting for Vizard". Launching Vizard any earlier
    # points its connect retries at an absent socket; its message-load
    # timeout (~30 s after connecting) then expires before the sim starts.
    outcome: dict[str, object] = {}

    def _fly() -> None:
        try:
            sim.InitializeSimulation()
            sim.ConfigureStopTime(macros.sec2nano(25.0))
            sim.ExecuteSimulation()
            outcome["ok"] = True
        except BaseException as exc:
            outcome["error"] = exc

    flight = threading.Thread(target=_fly, daemon=True)
    flight.start()
    time.sleep(10)

    # xvfb-run unconditionally: on a real desktop display Vizard's scene
    # init races the Basilisk world messages (celestial bodies not yet
    # loaded when the spacecraft scene builds), then floods null-reference
    # exceptions and never renders. The virtual X server keeps the
    # llvmpipe path where the handshake is stable.
    command = [
        "xvfb-run",
        "-a",
        str(binary),
        "-args",
        "-batchmode",
        "-noDisplay",
        "tcp://127.0.0.1:5556",
    ]
    viz_proc = subprocess.Popen(
        command,
        cwd=binary.parent,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    try:
        flight.join(300)
        assert not flight.is_alive(), "scenario never completed (no Vizard handshake?)"
        assert "error" not in outcome, outcome.get("error")
        frames = sorted(tmp_path.glob("cameraImage*.png"))
        assert len(frames) >= 2, f"expected rendered frames, found {len(frames)}"
    finally:
        # Unity ignores SIGTERM; escalate politely, then for real.
        viz_proc.terminate()
        try:
            viz_proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            viz_proc.kill()
            viz_proc.wait(timeout=30)


def test_build_cameras_without_interface_raises():
    """The phase guard is defensive but real: cameras refuse a missing interface.

    The schema already refuses cameras without ``[visualization].enabled``;
    this drives ``build_cameras`` directly on a context whose visualization
    phase was skipped, which is the one way ``ctx.viz`` stays ``None`` with
    cameras configured.
    """
    from bskforge.build.camera import build_cameras
    from bskforge.build.context import SimContext

    config = camera_scenario(camera_id=1)
    ctx = SimContext(config)  # processes only; no viz phase ran
    assert ctx.viz is None
    with pytest.raises(BuildError, match="Vizard interface"):
        build_cameras(ctx)
