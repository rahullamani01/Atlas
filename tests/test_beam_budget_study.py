import importlib.util
import json
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

from atlas.evaluation import get_scenario


spec = importlib.util.spec_from_file_location(
    "beam_budget_study", Path(__file__).resolve().parents[1] / "scripts/beam_budget_study.py"
)
study = importlib.util.module_from_spec(spec)
spec.loader.exec_module(study)


def test_budgets_change_only_filter_sampling_and_pair_by_seed(monkeypatch):
    calls = []

    def fake_run(scenario, seed, *, particles, beams):
        calls.append((scenario, seed, particles, beams))
        rms = seed + 1 / beams
        return SimpleNamespace(metrics={
            "filter_rms": rms, "filter_heading_rms_deg": rms * 2,
            "reached": beams != 2, "collisions": int(beams == 2),
        })

    monkeypatch.setattr(study, "run_localization", fake_run)
    seeds = [4, 1]
    data = study.run_study(seeds)
    assert calls == [
        (get_scenario("localization"), seed, 800, budget)
        for budget in (60, 20, 8, 2) for seed in seeds
    ]
    for row in data["results"]:
        assert row["rms_delta_from_reference"] == pytest.approx(1 / row["beam_budget"] - 1 / 60)
    sparse = data["summary"][-1]
    assert sparse["filter_rms"]["count"] == sparse["collisions"] == 2
    assert sparse["goals_reached"] == 0
    assert sparse["rms_delta_from_reference"]["std"] == pytest.approx(0.0)


@pytest.mark.parametrize("seeds", [[], [0, 0], [-1]])
def test_invalid_seed_sets_are_rejected(seeds):
    with pytest.raises(ValueError, match="seeds"):
        study.run_study(seeds)


def test_short_drive_reproduces_reference_and_plots_saved_results(monkeypatch, tmp_path):
    scenario = replace(get_scenario("localization"), max_steps=25)
    monkeypatch.setattr(study, "get_scenario", lambda name: scenario)
    data = json.loads(json.dumps(study.run_study([0]), allow_nan=False))
    reference, *_, sparse = data["results"]
    assert reference["metrics"] == study.run_localization(scenario, seed=0).metrics
    assert sparse["metrics"] != reference["metrics"]
    assert sparse["metrics"] == study.run_localization(scenario, seed=0, beams=2).metrics
    output = tmp_path / "plots" / "beam_budget.png"
    study.plot_study(data, output)
    assert output.read_bytes().startswith(b"\x89PNG")
