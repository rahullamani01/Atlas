"""Phase 17: vary the likelihood scale for two-return localization."""
from dataclasses import asdict
from pathlib import Path

from atlas.evaluation import get_scenario, run_localization, summarize
from atlas.evaluation.study_cli import run_or_plot


def run_study(seeds: list[int]) -> dict:
    if not seeds or min(seeds) < 0 or len(set(seeds)) != len(seeds):
        raise ValueError("provide distinct, non-negative seeds")

    scenario = get_scenario("localization")
    config = asdict(scenario)
    config["world"] = f"{scenario.world.__module__}.{scenario.world.__qualname__}"
    particles, budget = 800, 2
    reference = 0.2
    results, summary = [], []
    for policy in ("scan_order", "widest_pair"):
        reference_rms = {}
        for scale in (reference, 0.8, 6.0):
            runs, deltas = [], []
            for seed in seeds:
                metrics = run_localization(
                    scenario, seed, particles, budget, beam_selection=policy,
                    likelihood_scale=scale, include_drive_metrics=True,
                ).metrics
                runs.append(metrics)
                if scale == reference:
                    reference_rms[seed] = metrics["filter_rms"]
                delta = metrics["filter_rms"] - reference_rms[seed]
                deltas.append(delta)
                results.append({
                    "beam_selection": policy, "likelihood_scale": scale, "seed": seed,
                    "metrics": metrics, "paired_rms_delta": delta,
                })
            stats = {
                key: summarize([m[key] for m in runs]).as_dict()
                for key in ("filter_rms", "filter_heading_rms_deg", "duration", "true_goal_distance")
            }
            summary.append({
                "beam_selection": policy, "likelihood_scale": scale, **stats,
                "paired_rms_delta": summarize(deltas).as_dict(),
                "goals_reported": sum(int(m["reached"]) for m in runs),
                "collisions": sum(m["collisions"] for m in runs),
            })

    return {
        "scenario": config, "seeds": seeds, "particles": particles, "beam_budget": budget,
        "reference_scale": reference, "results": results, "summary": summary,
    }


def plot_study(data: dict, output: Path) -> None:
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(9, 4), sharey=True)
    for axis, policy in zip(axes, ("scan_order", "widest_pair")):
        for seed in data["seeds"]:
            rows = sorted(
                (r for r in data["results"] if r["beam_selection"] == policy and r["seed"] == seed),
                key=lambda r: r["likelihood_scale"],
            )
            axis.plot([r["likelihood_scale"] for r in rows],
                      [r["metrics"]["true_goal_distance"] for r in rows], "o-", label=f"seed {seed}")
        axis.set(title=policy, xlabel="Likelihood scale", xscale="log")
    axes[0].set_ylabel("Final true distance to goal (m)")
    axes[1].legend()
    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    run_or_plot(__doc__, run_study, plot_study)
