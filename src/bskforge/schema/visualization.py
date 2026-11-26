#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Visualization models: ``[visualization]`` (Vizard) and ``[[camera]]``."""

from __future__ import annotations

import math
from typing import Literal

from pydantic import Field

from bskforge.schema.base import ScenarioModel


class VisualizationSection(ScenarioModel):
    """``[visualization]``: Vizard live/replay visualization settings.

    Attributes:
        enabled: Attach the Basilisk ``vizInterface`` module.
        live_stream: Stream state in real time (adds a clock-sync module).
        broadcast_stream: Broadcast messages for listener Vizard instances.
        save_file: Write a Vizard replay data file at this path.
        no_display: Run Vizard headless (performance / CI capture).
    """

    enabled: bool = False
    live_stream: bool = False
    broadcast_stream: bool = False
    save_file: str | None = None
    no_display: bool = False


class CameraSection(ScenarioModel):
    """``[[camera]]``: an instrument whose image Vizard renders, not computes.

    The Basilisk ``camera`` module receives frames back from Vizard over
    the interface channel and lands them on its ``opnavImageOutMsg``.
    The wiring order is Basilisk's contract, not a preference: parameters
    must be set before the interface registers the camera, and
    ``imageInMsg`` may subscribe only after registration - the image slot
    does not exist until then. Slots are allocated in registration order.

    Attributes:
        parent: ModelTag of the spacecraft the camera is mounted on.
        camera_id: Vizard's identifier for this instrument.
        render_rate: Seconds between rendered frames.
        sigma_CB: Camera frame C pointing relative to body B, as MRP.
        camera_pos_B: Camera origin in the body frame [m].
        resolution: Rendered image size (width, height) [px].
        field_of_view: Full field of view [rad].
        sky_box: Vizard skybox name.
        save_images: Write every received frame to disk as PNG.
        save_dir: Saved-image filename prefix (the directory must exist).
        record_rate: The store outlet's cadence door - save one of every
            N captured frames. 0 (the default) closes it: a pure
            navigation camera ships zero image bytes and the absence is
            visible.
        record_mode: Which door fills the store outlet. "cadence" (the
            default) files every ``record_rate``-th capture. "snap"
            files ONLY shutter-pulse deliveries - the eye renders and
            feeds the estimator continuously while the store hears
            nothing until the ground commands a snapshot. "snap" with a
            nonzero ``record_rate`` is refused: one outlet, one door.
        downlink: Recorded frames are eligible for file transfer. An
            outlet with nothing to downlink is a typo, not a config, so
            this requires an open door: a nonzero ``record_rate`` or
            ``record_mode = "snap"``.

        The bytes an open outlet writes land in the vehicle's recorder,
        which is a :class:`~bskforge.schema.store.StoreSection` - declared
        once per vehicle payload, not once per eye, because two eyes on the
        same vehicle write into the same store.
    """

    parent: str
    camera_id: int = 1
    render_rate: float = Field(2.0, gt=0, description="Seconds per rendered frame")
    sigma_CB: tuple[float, float, float] = (0.0, 0.0, 0.0)
    camera_pos_B: tuple[float, float, float] = (0.0, 0.0, 0.0)
    resolution: tuple[int, int] = (512, 512)
    field_of_view: float = Field(math.radians(50.0), gt=0, lt=math.pi)
    sky_box: str = "black"
    save_images: bool = False
    save_dir: str | None = None
    record_rate: int = Field(
        0, ge=0, description="Store every Nth captured frame; 0 stores nothing"
    )
    record_mode: Literal["cadence", "snap"] = Field(
        "cadence",
        description="Outlet door: every Nth capture ('cadence') "
        "or pulse deliveries only ('snap')",
    )
    downlink: bool = False


class SpacecraftViz(ScenarioModel):
    """Per-vehicle Vizard appearance hints: ``[spacecraft.viz]``.

    Attributes:
        model_key: Vizard model-dictionary key for this vehicle.
    """

    model_key: str | None = None
