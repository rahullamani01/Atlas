"""Phase 18: replay one recorded localization drive through every filter setting."""
from dataclasses import asdict
from pathlib import Path

from atlas.evaluation import (
    get_scenario,
    record_localization_drive,
    replay_localization,
    summarize,
)
from atlas.evaluation.study_cli import run_or_plot

REFERENCE = ("scan_order", 0.2)
CONFIGURATIONS = (
    ("scan_order", 0.2),
    ("scan_order", 0.8),
    ("scan_order", 6.0),
    ("widest_pair", 0.2),
    ("widest_pair", 0.8),
    ("widest_pair", 6.0),
)
MATCH_TOLERANCE = 1e-12


def run_study(seeds: list[int]) -> dict:
    if not seeds or min(seeds) < 0 or len(set(seeds)) != len(seeds):
        raise ValueError("provide distinct, non-negative seeds")

    scenario = get_scenario("localization")
    config = asdict(scenario)
    config["world"] = f"{scenario.world.__module__}.{scenario.world.__qualname__}"
    particles, budget = 800, 2

    results, trajectories = [], []
    for seed in seeds:
        record = record_localization_drive(
            scenario, seed, particles, budget,
            beam_selection=REFERENCE[0], likelihood_scale=REFERENCE[1],
        )
        reference = replay_localization(
            scenario, record, particles, budget,
            beam_selection=REFERENCE[0], likelihood_scale=REFERENCE[1],
        ).metrics
        replay_delta = max(
            abs(reference[key] - record.live_metrics[key])
            for key in ("filter_rms", "filter_max", "filter_heading_rms_deg")
        )
        trajectories.append({
            "seed": seed,
            "steps": len(record.wheel_left),
            "duration": record.duration,
            "true_goal_distance": record.true_goal_distance,
            "collisions": record.collisions,
            "reached": 1.0 if record.reached else 0.0,
            "live_filter_rms": record.live_metrics["filter_rms"],
            "reference_replay_rms": reference["filter_rms"],
            "reference_max_abs_difference": replay_delta,
        })

        for policy, scale in CONFIGURATIONS:
            metrics = replay_localization(
                scenario, record, particles, budget,
                beam_selection=policy, likelihood_scale=scale,
            ).metrics
            results.append({
                "beam_selection": policy,
                "likelihood_scale": scale,
                "seed": seed,
                "metrics": metrics,
                "paired_rms_delta": metrics["filter_rms"] - reference["filter_rms"],
                "replay_matches_live": bool(
                    (policy, scale) == REFERENCE
                    and replay_delta <= MATCH_TOLERANCE
                ),
            })

    summary = []
    for policy, scale in CONFIGURATIONS:
        rows = [
            row for row in results
            if row["beam_selection"] == policy
            and row["likelihood_scale"] == scale
        ]
        summary.append({
            "beam_selection": policy,
            "likelihood_scale": scale,
            "filter_rms": summarize([
                row["metrics"]["filter_rms"] for row in rows
            ]).as_dict(),
            "filter_heading_rms_deg": summarize([
                row["metrics"]["filter_heading_rms_deg"] for row in rows
            ]).as_dict(),
            "paired_rms_delta": summarize([
                row["paired_rms_delta"] for row in rows
            ]).as_dict(),
        })

    return {
        "scenario": config,
        "method": "replay",
        "seeds": seeds,
        "particles": particles,
        "beam_budget": budget,
        "reference": {
            "beam_selection": REFERENCE[0], "likelihood_scale": REFERENCE[1],
        },
        "trajectories": trajectories,
        "results": results,
        "summary": summary,
    }


def plot_study(data: dict, output: Path) -> None:
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(9, 4), sharey=True)
    for axis, policy in zip(axes, ("scan_order", "widest_pair")):
        for seed in data["seeds"]:
            rows = sorted(
                (
                    row for row in data["results"]
                    if row["beam_selection"] == policy
                    and row["seed"] == seed
                ),
                key=lambda row: row["likelihood_scale"],
            )
            axis.plot(
                [row["likelihood_scale"] for row in rows],
                [row["metrics"]["filter_rms"] for row in rows],
                "o-", label=f"seed {seed}",
            )
        axis.set(title=policy, xlabel="Likelihood scale", xscale="log")
    axes[0].set_ylabel("Replayed filter position RMS (m)")
    axes[1].legend()
    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    run_or_plot(__doc__, run_study, plot_study)
