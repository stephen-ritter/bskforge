#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Monte Carlo campaign: Basilisk ``MonteCarlo.Controller`` + dispersions."""

from __future__ import annotations

from typing import Annotated, Literal

from pydantic import Field

from bskforge.schema.base import ScenarioModel, Vec3

#: dispersion value syntax handed through to Basilisk's dispersion
#: dictionaries: ``["uniform", lo, hi]`` or ``["normal", mean, std]``
DispValue = list | None


class OrbitElementsDispersion(ScenarioModel):
    """Randomize the initial orbit: ``quantity = "orbit_elements"``.

    ``elements`` maps classical-element names (``a``, ``e``, ``i``,
    ``raan``, ``omega``, ``f``) to ``["uniform", lo, hi]`` or
    ``["normal", mean, std]`` lists; ``None`` keeps the nominal value.
    Angles in the dispersion lists are in radians.
    """

    quantity: Literal["orbit_elements"] = "orbit_elements"
    spacecraft: str
    elements: dict[str, DispValue] = Field(default_factory=dict)


class SigmaDispersion(ScenarioModel):
    """Uniform random Euler-angle MRP: ``quantity = "sigma_BN"``."""

    quantity: Literal["sigma_BN"] = "sigma_BN"
    spacecraft: str


class OmegaDispersion(ScenarioModel):
    """Gaussian initial body rate: ``quantity = "omega_BN_B"``."""

    quantity: Literal["omega_BN_B"] = "omega_BN_B"
    spacecraft: str
    std: Vec3
    mean: Vec3 = [0.0, 0.0, 0.0]


class MassDispersion(ScenarioModel):
    """Uniform hub mass: ``quantity = "mass"``."""

    quantity: Literal["mass"] = "mass"
    spacecraft: str
    range: list[float] = Field(min_length=2, max_length=2)


DispersionConfig = Annotated[
    OrbitElementsDispersion | SigmaDispersion | OmegaDispersion | MassDispersion,
    Field(discriminator="quantity"),
]


class MonteCarloSection(ScenarioModel):
    """``[monte_carlo]``: repeat the scenario with dispersed initial states.

    The deterministic ``run`` command ignores this section; the
    ``mc`` command executes the campaign through Basilisk's
    ``MonteCarlo.Controller``. Run ``k`` reseeds the process RNG with
    ``10 * k``, so campaigns are reproducible; per-module stochastic
    sensors also get dispersed ``RNGSeed`` values.

    Attributes:
        runs: Number of dispersed runs to execute.
        threads: Parallel worker processes; 1 runs inline.
        archive_dir: Directory for per-run archives; defaults to the
            logging directory with a ``montecarlo`` suffix.
        dispersions: Initial-state randomizations to apply per run.
    """

    runs: int = Field(ge=1)
    threads: int = Field(1, ge=1)
    archive_dir: str | None = None
    dispersions: list[DispersionConfig] = Field(default_factory=list)
