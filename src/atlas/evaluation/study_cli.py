"""Command line shared by studies that record runs and plot the saved JSON."""
import argparse
from collections.abc import Callable
import json
from pathlib import Path


def run_or_plot(
    description: str,
    run_study: Callable[[list[int]], dict],
    plot_study: Callable[[dict, Path], None],
) -> None:
    parser = argparse.ArgumentParser(description=description)
    commands = parser.add_subparsers(dest="command", required=True)
    run = commands.add_parser("run")
    run.add_argument("--seeds", nargs="+", type=int, default=[0, 1, 2])
    run.add_argument("--output", type=Path, required=True)
    plot = commands.add_parser("plot")
    plot.add_argument("input", type=Path)
    plot.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "plot":
        plot_study(json.loads(args.input.read_text()), args.output)
    else:
        try:
            data = run_study(args.seeds)
        except ValueError as error:
            parser.error(str(error))
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")
    print(f"wrote {args.output}")
