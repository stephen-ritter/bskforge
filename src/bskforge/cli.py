#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""``bskforge`` command-line interface: run, validate, schema, graph."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from bskforge import __version__
from bskforge.errors import BskForgeError


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


def _cmd_validate(args: argparse.Namespace) -> int:
    from bskforge.loader import load_scenario

    config = load_scenario(args.scenario)
    print(f"valid: {config.simulation.name!r} — {len(config.spacecraft)} spacecraft")
    return 0


def _cmd_schema(_args: argparse.Namespace) -> int:
    from bskforge.schema.root import ScenarioConfig

    print(json.dumps(ScenarioConfig.model_json_schema(), indent=2))
    return 0


def _cmd_graph(args: argparse.Namespace) -> int:
    from bskforge.loader import load_scenario
    from bskforge.runner import ScenarioRunner

    config = load_scenario(args.scenario)
    built = ScenarioRunner(config).build()
    dot = built.context.sim.GetMessageConnectionDot()
    out = Path(args.out) if args.out else Path(args.scenario + ".messages.dot")
    out.write_text(dot)
    print(f"wrote {out}")
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

    validate = sub.add_parser("validate", help="validate without Basilisk")
    validate.add_argument("scenario", type=Path)
    validate.set_defaults(handler=_cmd_validate)

    schema = sub.add_parser("schema", help="print the scenario JSON schema")
    schema.set_defaults(handler=_cmd_schema)

    graph = sub.add_parser(
        "graph", help="write the Basilisk message-connection graph (.dot)"
    )
    graph.add_argument("scenario", type=Path)
    graph.add_argument("--out", type=str, default=None)
    graph.set_defaults(handler=_cmd_graph)

    return parser


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
