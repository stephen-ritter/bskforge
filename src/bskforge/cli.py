#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""``bskforge`` command-line interface: run, mc, validate, schema, reference, graph."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from bskforge import __version__
from bskforge.errors import BskForgeError
from bskforge.schema.root import ScenarioConfig


def main(argv: list[str] | None = None) -> int:
    """CLI entry point.

    Args:
        argv: Argument list (defaults to ``sys.argv[1:]``).

    Returns:
        Process exit code.
    """
    parser = _build_parser()
    args = parser.parse_args(argv)
    try:
        return args.handler(args)
    except BskForgeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    except Exception as exc:  # Basilisk/runtime failures
        print(f"error: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2


def _cmd_run(args: argparse.Namespace) -> int:
    from bskforge.runner import run_scenario

    result = run_scenario(
        args.scenario,
        duration=args.duration,
        output_directory=args.output_dir,
    )
    print(f"ran {result.config.simulation.name!r}")
    for key, frame in sorted(result.frames.items()):
        print(f"  {key}: {len(frame)} samples")
    for path in result.exported_files:
        print(f"  wrote {path}")
    return 0


def _cmd_mc(args: argparse.Namespace) -> int:
    from bskforge.loader import load_scenario
    from bskforge.montecarlo import run_monte_carlo

    config = load_scenario(args.scenario)
    result = run_monte_carlo(config, runs=args.runs, archive_dir=args.output_dir)
    print(
        f"monte carlo: {result.runs} runs -> {result.archive_dir}"
        + (f" | failed: {result.failures}" if result.failures else "")
    )
    print(result.summary.to_string(index=False))
    return 1 if result.failures else 0


def _scenario_summary(config: ScenarioConfig) -> list[str]:
    """Human-readable subsystem breakdown for ``validate`` output."""
    world = config.world
    n_grav = len(world.gravities)
    lines = [
        f"world: {world.central_body} | epoch {world.epoch or 'default'} | "
        + (
            f"{n_grav} gravit" + ("y" if n_grav == 1 else "ies")
            if n_grav
            else f"default {world.central_body} gravity"
        )
        + (" | SPICE" if world.spice.enabled else " | Kepler/analytic"),
    ]
    if config.environment is not None:
        env = config.environment
        env_bits = [
            getattr(env.magnetic_field, "type", None),
            getattr(env.atmosphere, "type", None),
            (t := getattr(env.eclipse, "type", None)) and f"eclipse:{t}",
        ]
        lines.append("environment: " + " | ".join(str(b) for b in env_bits if b))
    for sc in config.spacecraft:
        fsw = sc.fsw
        fsw_text = "fsw off"
        if fsw is not None:
            laws = [
                fsw.guidance.type if fsw.guidance else None,
                fsw.estimator.type if fsw.estimator else None,
                fsw.control.type if fsw.control else None,
            ]
            fsw_text = "fsw " + ("/".join(p for p in laws if p) or "custom only")
            if fsw.allocation is not None:
                fsw_text += f" + {fsw.allocation.type}"
        n_s = len(sc.sensors)
        n_e = len(sc.effectors)
        n_d = len(sc.disturbances)
        lines.append(
            f"vehicle {sc.name}: "
            f"{n_s} sensor{'' if n_s == 1 else 's'} "
            f"[{', '.join(sorted({s.type for s in sc.sensors})) or '—'}] | "
            f"{n_e} effector{'' if n_e == 1 else 's'} "
            f"[{', '.join(sorted({e.type for e in sc.effectors})) or '—'}] | "
            f"{n_d} disturbance{'' if n_d == 1 else 's'} | {fsw_text}"
        )
    if config.comm_links:
        lines.append(
            "comm links: "
            + ", ".join(f"{link.name} ({link.type})" for link in config.comm_links)
        )
    if config.formations:
        lines.append(
            "formations: "
            + ", ".join(
                f"{form.name} [{', '.join(form.members)}]" for form in config.formations
            )
        )
    if config.monte_carlo is not None:
        mc = config.monte_carlo
        lines.append(
            f"monte carlo: {mc.runs} runs | "
            f"{len(mc.dispersions)} dispersion"
            + ("s" if len(mc.dispersions) != 1 else "")
            + " (used by `bskforge mc`)"
        )
    if config.custom.tasks or config.custom.modules or config.custom.connections:
        custom = config.custom
        lines.append(
            f"custom: {len(custom.tasks)} task(s), {len(custom.modules)} "
            f"module(s), {len(custom.connections)} connection(s), "
            f"{len(custom.events)} event(s)"
        )
    return lines


def _cmd_validate(args: argparse.Namespace) -> int:
    from bskforge.loader import load_scenario

    config = load_scenario(args.scenario)
    print(
        f"valid: {config.simulation.name!r} — {len(config.spacecraft)} "
        f"spacecraft, {config.simulation.duration:g} s"
    )
    for line in _scenario_summary(config):
        print("  " + line)
    return 0


def _cmd_schema(_args: argparse.Namespace) -> int:
    from bskforge.schema.root import ScenarioConfig

    print(json.dumps(ScenarioConfig.model_json_schema(), indent=2))
    return 0


def _cmd_reference(args: argparse.Namespace) -> int:
    from bskforge.reference import generate_markdown, generate_toml_template

    text = generate_toml_template() if args.toml else generate_markdown()
    if args.out:
        Path(args.out).write_text(text)
        print(f"wrote {args.out}")
    else:
        print(text, end="")
    return 0


def _cmd_graph(args: argparse.Namespace) -> int:
    from bskforge.loader import load_scenario
    from bskforge.runner import ScenarioRunner

    config = load_scenario(args.scenario)
    built = ScenarioRunner(config).build()
    dot = built.context.sim.GetMessageConnectionDot()
    out = Path(args.out) if args.out else Path(str(args.scenario) + ".messages.dot")
    out.write_text(dot)
    edges = sum(1 for line in dot.splitlines() if "->" in line)
    modules = sum(1 for line in dot.splitlines() if "[label=" in line)
    print(
        f"graph: {out} — {modules} module(s), {edges} message connection(s)"
        " (render with: dot -Tsvg -o graph.svg "
        f"{out})"
    )
    return 0


def _build_parser() -> argparse.ArgumentParser:
    """Construct the argument parser with all subcommands."""
    parser = argparse.ArgumentParser(
        prog="bskforge",
        description="Forge and run Basilisk simulations from scenario files.",
    )
    parser.add_argument(
        "--version", action="version", version=f"bskforge {__version__}"
    )
    sub = parser.add_subparsers(dest="command", required=True)

    run = sub.add_parser("run", help="run a scenario file")
    run.add_argument("scenario", type=Path)
    run.add_argument(
        "--duration",
        type=float,
        default=None,
        help="override configured duration [s]",
    )
    run.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="override [logging].directory",
    )
    run.set_defaults(handler=_cmd_run)

    mc = sub.add_parser(
        "mc",
        help="run the scenario's [monte_carlo] campaign",
    )
    mc.add_argument("scenario", type=Path)
    mc.add_argument(
        "--runs",
        type=int,
        default=None,
        help="override [monte_carlo].runs",
    )
    mc.add_argument(
        "--output-dir",
        type=str,
        default=None,
        help="per-run archive directory override",
    )
    mc.set_defaults(handler=_cmd_mc)

    validate = sub.add_parser("validate", help="validate without Basilisk")
    validate.add_argument("scenario", type=Path)
    validate.set_defaults(handler=_cmd_validate)

    schema = sub.add_parser("schema", help="print the scenario JSON schema")
    schema.set_defaults(handler=_cmd_schema)

    reference = sub.add_parser(
        "reference",
        help="print the generated scenario-format reference (Markdown)",
    )
    reference.add_argument(
        "--toml",
        action="store_true",
        help="emit a commented TOML template instead of Markdown",
    )
    reference.add_argument(
        "--out",
        type=str,
        default=None,
        help="write to this file instead of stdout",
    )
    reference.set_defaults(handler=_cmd_reference)

    graph = sub.add_parser(
        "graph", help="write the Basilisk message-connection graph (.dot)"
    )
    graph.add_argument("scenario", type=Path)
    graph.add_argument("--out", type=str, default=None)
    graph.set_defaults(handler=_cmd_graph)

    return parser


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
