#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Monte Carlo campaigns: dispersions from TOML via MonteCarlo.Controller.

The Basilisk MonteCarlo controller builds and executes each run inside
a worker, so the creation/execution/retention callables below are
module-level functions (picklable) that read the campaign scenario from
the module-global :data:`_SCENARIO`, set once by
:func:`run_monte_carlo` before the controller starts.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion
from Basilisk.utilities.MonteCarlo.Controller import Controller
from Basilisk.utilities.MonteCarlo.Dispersions import (
    NormalVectorCartDispersion,
    OrbitalElementDispersion,
    UniformDispersion,
    UniformEulerAngleMRPDispersion,
)
from Basilisk.utilities.MonteCarlo.RetentionPolicy import RetentionPolicy

from bskforge.build import build_scenario
from bskforge.errors import BuildError
from bskforge.schema.monte_carlo import MonteCarloSection
from bskforge.schema.root import ScenarioConfig

_SCENARIO: ScenarioConfig | None = None


@dataclass
class MonteCarloResult:
    """Outcome of a Monte Carlo campaign.

    Attributes:
        summary: One row per run: dispersed parameters plus the final
            orbit and attitude state of every spacecraft.
        failures: Case numbers whose run raised an exception.
        archive_dir: Per-run parameter/retention archive directory.
        runs: Number of executed runs.
    """

    summary: pd.DataFrame
    failures: list[int] = field(default_factory=list)
    archive_dir: Path | None = None
    runs: int = 0


def _create_simulation() -> SimulationBaseClass.SimBaseClass:
    """Build one nominal scenario instance (MonteCarlo creation hook)."""
    if _SCENARIO is None:  # pragma: no cover - worker guard
        raise BuildError("monte carlo scenario was not initialized")
    built = build_scenario(_SCENARIO)
    ctx = built.context
    sim = ctx.sim
    sim.ConfigureStopTime(macros.sec2nano(_SCENARIO.simulation.duration))
    #: retention hooks resolve message logs through ``msgRecList``
    sim.msgRecList = {}  # ty: ignore
    sim.bskforge_bundles = ctx.bundles  # ty: ignore
    sim.bskforge_mu = ctx.central_mu  # ty: ignore
    return sim


def _execute_simulation(sim: SimulationBaseClass.SimBaseClass) -> None:
    """Run one dispersed simulation to completion (execution hook)."""
    sim.InitializeSimulation()
    sim.ExecuteSimulation()


def _retain_final_state(sim: SimulationBaseClass.SimBaseClass) -> dict:
    """Retention hook: final classical elements and attitude per SC."""
    out: dict[str, np.ndarray] = {}
    mu = sim.bskforge_mu  # ty: ignore
    for name, bundle in sim.bskforge_bundles.items():  # ty: ignore
        spec = next((r for r in bundle.recorders if r.key == f"{name}.state"), None)
        if spec is None:
            continue
        rec = spec.recorder
        r = np.array([float(x) for x in rec.r_BN_N[-1]])
        v = np.array([float(x) for x in rec.v_BN_N[-1]])
        sigma = np.array([float(x) for x in rec.sigma_BN[-1]])
        omega = np.array([float(x) for x in rec.omega_BN_B[-1]])
        t = float(rec.times()[-1])
        oe = orbitalMotion.rv2elem(mu, r, v)
        out[f"{name}.orbit_final"] = np.array(
            [[t, oe.a, oe.e, oe.i, oe.Omega, oe.omega, oe.f]]
        )
        out[f"{name}.attitude_final"] = np.array(
            [
                [
                    t,
                    float(sigma[0]),
                    float(sigma[1]),
                    float(sigma[2]),
                    float(np.linalg.norm(omega)),
                ]
            ]
        )
    return out


def _hub_path(sim: SimulationBaseClass.SimBaseClass, tag: str) -> str:
    """Attribute path of the hub module ``tag`` inside ``sim``."""
    for ti, task in enumerate(sim.TaskList):
        for mi, module in enumerate(task.TaskModels):
            if getattr(module, "ModelTag", None) == tag:
                return f"TaskList[{ti}].TaskModels[{mi}].hub"
    raise BuildError(f"monte carlo: spacecraft module {tag!r} not found")


def _nominal_elements(sim: SimulationBaseClass.SimBaseClass, mu: float) -> dict:
    """Classical elements of each hub's initial state in ``sim``."""
    out = {}
    for task in sim.TaskList:
        for module in task.TaskModels:
            hub = getattr(module, "hub", None)
            if hub is None or not hasattr(hub, "r_CN_NInit"):
                continue
            r = np.array(hub.r_CN_NInit, dtype=float).reshape(3)
            v = np.array(hub.v_CN_NInit, dtype=float).reshape(3)
            oe = orbitalMotion.rv2elem(mu, r, v)
            out[module.ModelTag] = oe
    return out


def _build_dispersions(
    section: MonteCarloSection,
    paths: dict[str, str],
    elements: dict[str, Any],
    mu: float,
) -> list[Any]:
    """Translate schema dispersions into Basilisk dispersion objects."""
    out = []
    for disp in section.dispersions:
        hub = paths[disp.spacecraft]
        if disp.quantity == "orbit_elements":
            # BSK's OrbitalElementDispersion *defines* the full orbit;
            # fill un-dispersed keys with the nominal element values
            oe = elements[disp.spacecraft]
            defaults: dict[str, Any] = {
                "a": oe.a,
                "e": oe.e,
                "i": oe.i,
                "Omega": oe.Omega,
                "omega": oe.omega,
                "f": oe.f,
            }
            user = dict(disp.elements)
            if "raan" in user:
                user["Omega"] = user.pop("raan")
            # constants must ride through as zero-width uniforms: the
            # BSK dispersion reads every non-None entry as [kind, p1, p2]
            defaults = {k: ["uniform", float(v), float(v)] for k, v in defaults.items()}
            user = {k: v for k, v in user.items() if v is not None}
            defaults.update(user)
            defaults["mu"] = mu
            out.append(
                OrbitalElementDispersion(
                    hub + ".r_CN_NInit", hub + ".v_CN_NInit", defaults
                )
            )
        elif disp.quantity == "sigma_BN":
            out.append(UniformEulerAngleMRPDispersion(hub + ".sigma_BNInit"))
        elif disp.quantity == "omega_BN_B":
            out.append(
                NormalVectorCartDispersion(
                    hub + ".omega_BN_BInit", list(disp.mean), list(disp.std)
                )
            )
        elif disp.quantity == "mass":
            out.append(UniformDispersion(hub + ".mHub", list(disp.range)))
    return out


def _summarize(controller: Controller, runs: int, names: list[str]) -> pd.DataFrame:
    """Collect retained final states into one row per run."""
    rows = []
    for case in range(runs):
        retained = controller.getRetainedData(case)
        custom = (retained or {}).get("custom", {})
        row: dict[str, Any] = {"case": case}
        for name in names:
            orbit = custom.get(f"{name}.orbit_final")
            if orbit is not None:
                (
                    _t,
                    a,
                    e,
                    inc,
                    raan,
                    argp,
                    nu,
                ) = orbit[0]
                row[f"{name}.a"] = a
                row[f"{name}.e"] = e
                row[f"{name}.i"] = inc
                row[f"{name}.raan"] = raan
                row[f"{name}.omega"] = argp
                row[f"{name}.f"] = nu
                row[f"{name}.sigma_final"] = np.linalg.norm(
                    custom[f"{name}.attitude_final"][0][1:4]
                )
        rows.append(row)
    return pd.DataFrame(rows)


def run_monte_carlo(
    config: ScenarioConfig,
    runs: int | None = None,
    archive_dir: str | Path | None = None,
) -> MonteCarloResult:
    """Execute the scenario's ``[monte_carlo]`` campaign.

    Args:
        config: Validated scenario carrying a ``monte_carlo`` section.
        runs: Override of ``[monte_carlo].runs``.
        archive_dir: Override of the per-run archive directory.

    Returns:
        A :class:`MonteCarloResult` with the per-run summary frame.
    """
    global _SCENARIO
    if config.monte_carlo is None:
        raise BuildError("scenario has no [monte_carlo] section")
    section = config.monte_carlo
    n_runs = runs if runs is not None else section.runs
    archive = Path(
        archive_dir
        or section.archive_dir
        or (config.logging.directory or ".") + f"/{config.simulation.name}_mc"
    )

    # resolve hub attribute paths once from a nominal build
    nominal = build_scenario(config)
    mu = nominal.context.central_mu
    paths = {
        sc.name: _hub_path(nominal.context.sim, sc.name) for sc in config.spacecraft
    }
    elements = _nominal_elements(nominal.context.sim, mu)
    del nominal

    _SCENARIO = config
    controller = Controller()
    controller.setSimulationFunction(_create_simulation)
    controller.setExecutionFunction(_execute_simulation)
    controller.setExecutionCount(n_runs)
    controller.setThreadCount(section.threads)
    controller.setShouldDisperseSeeds(True)
    controller.setShouldArchiveParameters(True)
    controller.setArchiveDir(str(archive))
    for disp in _build_dispersions(section, paths, elements, mu):
        controller.addDispersion(disp)
    policy = RetentionPolicy()
    policy.addRetentionFunction(_retain_final_state)
    controller.addRetentionPolicy(policy)

    failures = controller.executeSimulations() or []

    loaded = Controller.load(str(archive))
    summary = _summarize(loaded, n_runs, [sc.name for sc in config.spacecraft])
    return MonteCarloResult(
        summary=summary,
        failures=list(failures),
        archive_dir=archive,
        runs=n_runs,
    )
