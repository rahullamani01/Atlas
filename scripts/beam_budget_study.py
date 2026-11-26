"""Phase 15: compare particle-filter beam budgets on paired localization drives."""
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
    particles = 800
    reference = 60
    results = []
    summary = []
    reference_rms = {}
    for budget in (reference, 20, 8, 2):
        runs = []
        differences = []
        for seed in seeds:
            metrics = run_localization(scenario, seed, particles=particles, beams=budget).metrics
            if budget == reference:
                reference_rms[seed] = metrics["filter_rms"]
            delta = metrics["filter_rms"] - reference_rms[seed]
            results.append({
                "seed": seed, "beam_budget": budget, "metrics": metrics,
                "rms_delta_from_reference": delta,
            })
            differences.append(delta)
            runs.append(metrics)
        summary.append({
            "beam_budget": budget,
            "filter_rms": summarize([m["filter_rms"] for m in runs]).as_dict(),
            "filter_heading_rms_deg": summarize(
                [m["filter_heading_rms_deg"] for m in runs]
            ).as_dict(),
            "rms_delta_from_reference": summarize(differences).as_dict(),
            "goals_reached": sum(int(m["reached"]) for m in runs),
            "collisions": sum(m["collisions"] for m in runs),
        })
    return {"scenario": config, "particles": particles, "reference_budget": reference,
            "seeds": seeds, "results": results, "summary": summary}


def plot_study(data: dict, output: Path) -> None:
    import matplotlib.pyplot as plt

    fig, axis = plt.subplots(figsize=(7, 4))
    for seed in data["seeds"]:
        rows = sorted((r for r in data["results"] if r["seed"] == seed),
                      key=lambda r: r["beam_budget"])
        axis.plot([r["beam_budget"] for r in rows], [r["metrics"]["filter_rms"] for r in rows],
                  "o-", label=f"seed {seed}")
    axis.set(xlabel="Maximum valid returns scored per update", ylabel="Filter position RMS (m)",
             xticks=sorted({r["beam_budget"] for r in data["results"]}), ylim=(0, None))
    axis.legend()
    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    run_or_plot(__doc__, run_study, plot_study)
