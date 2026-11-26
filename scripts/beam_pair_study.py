"""Phase 16: vary which two valid LiDAR returns the particle filter scores."""
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
    policies = ("scan_order", "widest_pair")
    pairs = []
    differences = []
    for seed in seeds:
        pair = {
            policy: run_localization(
                scenario, seed, particles, budget, beam_selection=policy
            ).metrics
            for policy in policies
        }
        pairs.append({"seed": seed, **pair})
        differences.append(pair["widest_pair"]["filter_rms"] - pair["scan_order"]["filter_rms"])

    summary = {}
    for policy in policies:
        runs = [pair[policy] for pair in pairs]
        summary[policy] = {
            key: summarize([m[key] for m in runs]).as_dict()
            for key in ("filter_rms", "filter_heading_rms_deg")
        }
        summary[policy]["goals_reached"] = sum(int(m["reached"]) for m in runs)
        summary[policy]["collisions"] = sum(m["collisions"] for m in runs)

    return {
        "scenario": config, "particles": particles, "beam_budget": budget,
        "seeds": seeds, "results": pairs, "summary": summary,
        "paired_rms_delta_widest_minus_scan_order": summarize(differences).as_dict(),
    }


def plot_study(data: dict, output: Path) -> None:
    import matplotlib.pyplot as plt

    fig, axis = plt.subplots(figsize=(6, 4))
    for pair in data["results"]:
        errors = [pair[policy]["filter_rms"] for policy in ("scan_order", "widest_pair")]
        axis.plot(["Scan order", "Widest pair"], errors, "o-", label=f"seed {pair['seed']}")
    axis.set(xlabel="Two-return selection", ylabel="Filter position RMS (m)", ylim=(0, None))
    axis.legend()
    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    run_or_plot(__doc__, run_study, plot_study)
