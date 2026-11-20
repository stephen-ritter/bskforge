#
#  ISC License
#
#  Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for
#  any purpose with or without fee is hereby granted, provided that the
#  above copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

"""TOML parsing utilities for Basilisk simulation configuration."""

from __future__ import annotations

import tomllib
from pathlib import Path
from typing import Union

from bsk_toml_sim.models.simulation import SimulationConfig


def load_simulation_config(
    path: Union[str, Path],
) -> SimulationConfig:
    """Load a simulation configuration from a TOML file.

    Reads the TOML file at ``path``, parses it into a dictionary, and
    validates it against :class:`SimulationConfig`.

    Args:
        path: Filesystem path to the TOML configuration file.

    Returns:
        A validated :class:`SimulationConfig` instance.

    Raises:
        FileNotFoundError: If the TOML file does not exist.
        ValidationError: If the TOML content does not satisfy the model.

    Example::

        from bsk_toml_sim import load_simulation_config

        config = load_simulation_config("my_sim.toml")
        print(config.name)
        for sat in config.satellites:
            print(f"  {sat.name}: mass={sat.hub.mass} kg")
    """
    path = Path(path)

    if not path.exists():
        raise FileNotFoundError(f"Configuration file not found: {path}")

    with open(path, "rb") as f:
        data = tomllib.load(f)

    # The top-level [simulation] table maps to SimulationConfig fields.
    # If present, use it; otherwise treat the entire file as the config.
    if "simulation" in data:
        sim_data = data["simulation"]
        # Merge top-level satellites array (if defined outside [simulation])
        if "satellites" not in sim_data and "satellites" in data:
            sim_data["satellites"] = data["satellites"]
    else:
        sim_data = data

    return SimulationConfig(**sim_data)