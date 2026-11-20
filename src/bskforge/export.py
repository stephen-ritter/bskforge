#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Data export: Basilisk recorders → pandas frames → CSV/parquet/npz files.

Recorder fields are discovered from the recorder object's own internal
field vectors, so new message types export without bskforge changes.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from bskforge.build.context import RecorderSpec

_NS_TO_S = 1e-9


def frames_from_recorders(recorders: list[RecorderSpec]) -> dict[str, pd.DataFrame]:
    """Convert recorder logs into per-stream pandas frames.

    Args:
        recorders: Registered recorders from a built scenario.

    Returns:
        Mapping of stream key (e.g. ``"sc1.state"``) to a DataFrame with a
        ``time`` column in seconds plus one column per recorded payload
        field (array fields expand to ``field[i]``).
    """
    frames: dict[str, pd.DataFrame] = {}
    for spec in recorders:
        frame = _frame(spec.recorder)
        if not frame.empty:
            frames[spec.key] = frame
    return frames


def write_frames(
    frames: dict[str, pd.DataFrame], directory: Path, format: str = "csv"
) -> list[Path]:
    """Write frames to ``directory`` in the chosen container format.

    Args:
        frames: Streams from :func:`frames_from_recorders`.
        directory: Output directory (created if needed).
        format: ``"csv"``, ``"parquet"`` or ``"npz"``.

    Returns:
        Paths of the files written.
    """
    directory.mkdir(parents=True, exist_ok=True)
    written: list[Path] = []
    for key, frame in frames.items():
        if frame.empty:
            continue
        safe = key.replace("/", "_")
        if format == "csv":
            path = directory / f"{safe}.csv"
            frame.to_csv(path, index=False)
        elif format == "parquet":
            path = directory / f"{safe}.parquet"
            frame.to_parquet(path, index=False)
        elif format == "npz":
            path = directory / f"{safe}.npz"
            np.savez(path, **{c: frame[c].to_numpy() for c in frame.columns})
        else:  # pragma: no cover - guarded by schema Literal
            raise ValueError(f"unsupported export format {format!r}")
        written.append(path)
    return written


def _frame(recorder: Any) -> pd.DataFrame:
    """Build one DataFrame from a Basilisk recorder's field vectors."""
    if recorder.size() == 0:
        return pd.DataFrame()
    columns: dict[str, Any] = {
        "time": np.asarray(recorder.times(), dtype=float) * _NS_TO_S
    }
    for field in _recorded_fields(recorder):
        array = np.asarray(getattr(recorder, field), dtype=float)
        if array.ndim == 1:
            columns[field] = array
        else:
            for index in range(array.shape[1]):
                columns[f"{field}[{index}]"] = array[:, index]
    return pd.DataFrame(columns)


def _recorded_fields(recorder: Any) -> list[str]:
    """List payload field names backed by recorder storage."""
    fields = []
    for name in dir(recorder):
        if name.startswith("_") and name.endswith("_array"):
            fields.append(name[1 : -len("_array")])
    return sorted(fields)
