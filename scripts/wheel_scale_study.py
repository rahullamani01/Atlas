"""Phase 14: compare dead reckoning and scan-corrected localization under wheel bias."""
from __future__ import annotations

from dataclasses import asdict, replace
from pathlib import Path

from atlas.evaluation import get_scenario, run_localization, summarize
from atlas.evaluation.study_cli import run_or_plot


def run_study(seeds: list[int]) -> dict:
    if not seeds or len(set(seeds)) != len(seeds) or min(seeds) < 0:
        raise ValueError("provide distinct, non-negative seeds")

    scenario = get_scenario("localization")
    config = asdict(scenario)
    config["world"] = f"{scenario.world.__module__}.{scenario.world.__qualname__}"
    particles, beams = 800, 60
    results = []
    summary = []
    for bias in (0.00, 0.01, 0.03):
        condition = replace(scenario, wheel_scale_left=1 - bias, wheel_scale_right=1 + bias)
        runs = []
        for seed in seeds:
            metrics = run_localization(condition, seed, particles=particles, beams=beams).metrics
            runs.append(metrics)
            results.append({
                "seed": seed,
                "bias": bias,
                "wheel_scale_left": condition.wheel_scale_left,
                "wheel_scale_right": condition.wheel_scale_right,
                "metrics": metrics,
            })

        # Compare estimates within each drive before summarizing across seeds.
        differences = [m["odometry_rms"] - m["filter_rms"] for m in runs]
        summary.append({
            "bias": bias,
            "odometry_rms": summarize([m["odometry_rms"] for m in runs]).as_dict(),
            "filter_rms": summarize([m["filter_rms"] for m in runs]).as_dict(),
            "paired_rms_difference": summarize(differences).as_dict(),
            "goals_reached": sum(int(m["reached"]) for m in runs),
            "collisions": sum(m["collisions"] for m in runs),
        })
    return {
        "scenario": config, "particles": particles, "beams": beams,
        "seeds": seeds, "results": results, "summary": summary,
    }


def plot_study(data: dict, output: Path) -> None:
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(9, 4), sharex=True)
    for axis, metric, title in zip(axes, ("odometry_rms", "filter_rms"),
                                   ("Wheel odometry", "Particle filter")):
        for seed in data["seeds"]:
            runs = sorted((r for r in data["results"] if r["seed"] == seed),
                          key=lambda r: r["bias"])
            axis.plot([100 * r["bias"] for r in runs],
                      [r["metrics"][metric] for r in runs], "o-", label=f"seed {seed}")
        axis.set(title=title, xlabel="Opposing wheel bias per wheel (%)",
                 ylabel="Position RMS (m)", ylim=(0, None))
        axis.legend()
    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=150)
    plt.close(fig)


def main() -> None:
    run_or_plot(__doc__, run_study, plot_study)


if __name__ == "__main__":
    main()
