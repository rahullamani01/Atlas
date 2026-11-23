"""Compare localization under paired LiDAR noise levels."""
from __future__ import annotations

import argparse
from dataclasses import fields, replace
import json
from pathlib import Path

from atlas.evaluation import get_scenario, run_localization


def run_study(seeds: list[int]) -> dict:
    scenario = get_scenario("localization")
    config = {}
    for field in fields(scenario):
        value = getattr(scenario, field.name)
        config[field.name] = f"{value.__module__}.{value.__qualname__}" if callable(value) else value
    levels = [0.02, 0.1, 0.3]
    results = []
    for seed in seeds:
        for sigma in levels:
            result = run_localization(replace(scenario, lidar_sigma=sigma), seed=seed)
            results.append({"seed": seed, "lidar_sigma": sigma, "metrics": result.metrics})
    return {"scenario": config, "seeds": seeds, "noise_levels": levels,
            "particles": 800, "beams": 60, "results": results}


def plot_study(data: dict, output: Path) -> None:
    import matplotlib.pyplot as plt

    fig, axis = plt.subplots(figsize=(6, 4))
    for seed in data["seeds"]:
        rows = [row for row in data["results"] if row["seed"] == seed]
        values = {row["lidar_sigma"]: row["metrics"]["filter_rms"] for row in rows}
        axis.plot(data["noise_levels"], [values[level] for level in data["noise_levels"]],
                  marker="o", label=f"seed {seed}")
    axis.set_xlabel("LiDAR range noise standard deviation (m)")
    axis.set_ylabel("Filter position RMS (m)")
    axis.legend()
    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=150)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    run = subparsers.add_parser("run")
    run.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2])
    run.add_argument("--output", type=Path, required=True)
    plot = subparsers.add_parser("plot")
    plot.add_argument("input", type=Path)
    plot.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "run":
        if len(set(args.seeds)) != len(args.seeds):
            parser.error("seeds must be unique")
        data = run_study(args.seeds)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(data, indent=2) + "\n")
    else:
        plot_study(json.loads(args.input.read_text()), args.output)
    print(f"wrote {args.output}")


if __name__ == "__main__":
    main()
