#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Shared builder helpers: validated parameter passthrough."""

from __future__ import annotations

import difflib
from typing import Any

from bskforge.errors import BuildError


def apply_params(module: Any, params: dict[str, Any], label: str) -> None:
    """Apply a ``params`` table onto a Basilisk object with ``setattr``.

    Attribute names are checked against the installed Basilisk build, so a
    typo fails at build time with a did-you-mean hint instead of crashing
    deep inside ``InitializeSimulation``.

    Args:
        module: The Basilisk object (SWIG module, factory, body, ...).
        params: Free-form parameter table from the scenario file.
        label: Human-readable owner name for error messages.

    Raises:
        BuildError: A parameter name does not exist on ``module``.
    """
    known = set(dir(module))
    for key, value in params.items():
        if key not in known:
            close = difflib.get_close_matches(key, sorted(known), n=1, cutoff=0.6)
            hint = f" (did you mean {close[0]!r}?)" if close else ""
            raise BuildError(
                f"{label}: unknown parameter {key!r}{hint}; check against the "
                "Basilisk module documentation for this build"
            )
        setattr(module, key, value)
