import importlib.util
import json
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

from atlas.evaluation import get_scenario


spec = importlib.util.spec_from_file_location(
    "wheel_scale_study", Path(__file__).resolve().parents[1] / "scripts/wheel_scale_study.py"
)
study = importlib.util.module_from_spec(spec)
spec.loader.exec_module(study)


def test_study_pairs_errors_and_keeps_failed_runs(monkeypatch):
    nominal = get_scenario("localization")
    calls = []

    def fake_run(scenario, seed, particles, beams):
        calls.append((scenario, seed, particles, beams))
        # Both estimators vary across seeds, but their paired difference is constant.
        return SimpleNamespace(metrics={
            "odometry_rms": 2 + seed, "filter_rms": 1 + seed,
            "reached": seed == 0, "collisions": int(seed != 0),
        })

    monkeypatch.setattr(study, "run_localization", fake_run)
    data = study.run_study([0, 3])
    assert calls == [
        (replace(nominal, wheel_scale_left=1 - bias, wheel_scale_right=1 + bias), seed, 800, 60)
        for bias in (0, 0.01, 0.03) for seed in (0, 3)
    ]
    assert len(data["results"]) == 6
    for group in data["summary"]:
        assert group["goals_reached"] == group["collisions"] == 1
        assert group["paired_rms_difference"] == {
            "count": 2, "mean": 1, "std": 0, "minimum": 1, "maximum": 1,
        }


@pytest.mark.parametrize("seeds", [[], [2, 2], [-1]])
def test_study_rejects_invalid_seeds(seeds):
    with pytest.raises(ValueError, match="seeds"):
        study.run_study(seeds)


def test_wheel_bias_changes_measured_drift_and_saved_results_plot(monkeypatch, tmp_path):
    scenario = replace(get_scenario("localization"), max_steps=25)
    monkeypatch.setattr(study, "get_scenario", lambda name: scenario)
    data = json.loads(json.dumps(study.run_study([0]), allow_nan=False))
    unbiased, nominal, stressed = data["results"]
    assert unbiased["metrics"]["odometry_rms"] < nominal["metrics"]["odometry_rms"]
    assert nominal["metrics"]["odometry_rms"] < stressed["metrics"]["odometry_rms"]
    assert nominal["metrics"] == study.run_localization(scenario, seed=0).metrics

    plot = tmp_path / "plots" / "wheel_bias.png"
    study.plot_study(data, plot)
    assert plot.read_bytes().startswith(b"\x89PNG")
