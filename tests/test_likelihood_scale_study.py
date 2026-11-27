from dataclasses import replace
import importlib.util
import json
import math
from pathlib import Path

import pytest

from atlas.evaluation import get_scenario, run_localization
from atlas.simulation.simulator import Simulator


spec = importlib.util.spec_from_file_location(
    "likelihood_scale_study",
    Path(__file__).resolve().parents[1] / "scripts/likelihood_scale_study.py",
)
study = importlib.util.module_from_spec(spec)
spec.loader.exec_module(study)


@pytest.mark.parametrize("scale", [0, -1, math.nan, math.inf])
def test_localization_rejects_invalid_likelihood_scales(scale):
    with pytest.raises(ValueError, match="likelihood_scale"):
        run_localization(get_scenario("localization"), likelihood_scale=scale)


def test_drive_metrics_score_executed_steps_and_true_pose(monkeypatch):
    scenario = replace(get_scenario("localization"), max_steps=12)
    positions = []
    step = Simulator.step

    def record_step(self, v, w):
        ok = step(self, v, w)
        positions.append((self.robot.pose.x, self.robot.pose.y))
        return ok

    baseline = run_localization(scenario, seed=1, beams=2).metrics
    monkeypatch.setattr(Simulator, "step", record_step)
    metrics = run_localization(
        scenario, seed=1, beams=2, likelihood_scale=0.2, include_drive_metrics=True,
    ).metrics.copy()
    assert metrics.pop("duration") == len(positions) * scenario.dt
    assert metrics.pop("true_goal_distance") == pytest.approx(math.dist(positions[-1], scenario.goal))
    assert metrics == baseline


def test_study_pairs_within_policy_and_retains_unfinished_runs(monkeypatch, tmp_path):
    scenario = replace(get_scenario("localization"), max_steps=30)
    monkeypatch.setattr(study, "get_scenario", lambda name: scenario)
    seeds = [1, 3]
    data = json.loads(json.dumps(study.run_study(seeds), allow_nan=False))
    rows = {(r["beam_selection"], r["likelihood_scale"], r["seed"]): r for r in data["results"]}
    assert len(data["results"]) == len(rows) == 12
    for policy in ("scan_order", "widest_pair"):
        for seed in seeds:
            ref = rows[policy, 0.2, seed]["metrics"]["filter_rms"]
            assert rows[policy, 6.0, seed]["metrics"]["filter_rms"] != ref
            for scale in (0.2, 0.8, 6.0):
                row = rows[policy, scale, seed]
                assert row["paired_rms_delta"] == pytest.approx(row["metrics"]["filter_rms"] - ref)
    for group in data["summary"]:
        assert group["filter_rms"]["count"] == len(seeds)
        assert group["goals_reported"] == 0
    plot = tmp_path / "plots" / "scale.png"
    study.plot_study(data, plot)
    assert plot.read_bytes().startswith(b"\x89PNG")
