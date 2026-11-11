"""Phase 7 benchmark: run the evaluation suite and aggregate metrics.

Examples
--------
    python scripts/evaluate.py
    python scripts/evaluate.py --seeds 0 1 2 3 4
    python scripts/evaluate.py --scenario planning --scenario tracking_pid
    python scripts/evaluate.py --json experiments/metrics.json --csv experiments/metrics.csv
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

from atlas.evaluation import (
    SCENARIOS,
    aggregate,
    default_scenarios,
    run_suite,
)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenario",
        action="append",
        choices=sorted(SCENARIOS),
        help="run only this scenario (repeatable)",
    )
    parser.add_argument(
        "--seeds",
        type=int,
        nargs="+",
        default=[0, 1, 2],
        help="seeds to repeat each scenario with (default: 0 1 2)",
    )
    parser.add_argument(
        "--json",
        type=Path,
        help="write per-run results and aggregate summaries as JSON",
    )
    parser.add_argument(
        "--csv",
        type=Path,
        help="write aggregate summaries as CSV",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="list the scenario names and exit",
    )
    return parser.parse_args(argv)


def print_summary(summary) -> None:
    for scenario, metrics in summary.items():
        print(f"\n{scenario}")
        print(
            f"{'metric':<26}{'mean':>12}{'std':>12}"
            f"{'min':>12}{'max':>12}{'n':>5}"
        )
        print("-" * 79)
        for name, values in metrics.items():
            print(
                f"{name:<26}{values.mean:>12.6g}{values.std:>12.6g}"
                f"{values.minimum:>12.6g}{values.maximum:>12.6g}"
                f"{values.count:>5}"
            )


def write_json(path: Path, seeds, results, summary) -> None:
    payload = {
        "seeds": seeds,
        "results": [
            {
                "scenario": result.scenario,
                "seed": result.seed,
                "metrics": result.metrics,
            }
            for result in results
        ],
        "summary": {
            scenario: {
                name: values.as_dict() for name, values in metrics.items()
            }
            for scenario, metrics in summary.items()
        },
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n")


def write_csv(path: Path, summary) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            ["scenario", "metric", "count", "mean", "std", "min", "max"]
        )
        for scenario, metrics in summary.items():
            for name, values in metrics.items():
                writer.writerow([
                    scenario,
                    name,
                    values.count,
                    f"{values.mean:.9g}",
                    f"{values.std:.9g}",
                    f"{values.minimum:.9g}",
                    f"{values.maximum:.9g}",
                ])


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    if args.list:
        for name, scenario in SCENARIOS.items():
            print(f"{name:<14}{scenario.kind}")
        return

    scenarios = (
        [SCENARIOS[name] for name in args.scenario]
        if args.scenario
        else default_scenarios()
    )
    print(
        f"scenarios: {len(scenarios)}   seeds: {args.seeds}   "
        f"runs: {len(scenarios) * len(args.seeds)}"
    )

    results = run_suite(scenarios, seeds=args.seeds)
    summary = aggregate(results)
    print_summary(summary)

    if args.json:
        write_json(args.json, args.seeds, results, summary)
        print(f"\nwrote {args.json}")
    if args.csv:
        write_csv(args.csv, summary)
        print(f"wrote {args.csv}")


if __name__ == "__main__":
    main()
