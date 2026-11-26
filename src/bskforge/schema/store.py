#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""``[[store]]``: a vehicle's recorder, sized before flight."""

from __future__ import annotations

from pydantic import Field

from bskforge.schema.base import ScenarioModel


class StoreSection(ScenarioModel):
    """``[[store]]``: one payload's recorder on one vehicle.

    A store outlet writes bytes, and bytes run out. A recorder with no
    declared capacity cannot say when it is full - and "full" is precisely
    the moment flight software needs a law rather than an improvisation, so
    the question is asked where the door can refuse the nonsense instead of
    leaving it to the vehicle.

    Two bounds, because one is not enough. ``capacity_bytes`` bounds the
    volume; ``capacity_files`` bounds the count. Frame size follows the
    scene rather than the intent - 64 MiB of empty sky is roughly 40,000
    frames while the same volume of a bright target is a few hundred - so a
    law written only in bytes cannot say when its own index has outgrown
    what can honestly be listed.

    The recorder belongs to the payload, not the instrument: two eyes on one
    vehicle write into one store, so they declare it once together. A second
    payload declares its own row, with its own capacity and - when it
    arrives - its own catalog.

    Attributes:
        parent: Vehicle whose recorder this is (a ``[[spacecraft]]`` name).
        payload: The payload that owns the bytes.
        capacity_bytes: Recorder capacity [bytes].
        capacity_files: Most files the store holds before the oldest
            proven-received file makes room.
    """

    parent: str
    payload: str = "cam"
    capacity_bytes: int = Field(
        gt=0, description="Recorder capacity [bytes]; 0 stores nothing"
    )
    capacity_files: int = Field(
        gt=0, description="Files the store holds before the oldest makes room"
    )
