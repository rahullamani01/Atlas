from dataclasses import replace
import importlib.util
import json
from pathlib import Path

import pytest

from atlas.evaluation import (
    get_scenario,
    record_localization_drive,
    replay_localization,
    run_localization,
)


spec = importlib.util.spec_from_file_location(
    "replay_localization_study",
    Path(__file__).resolve().parents[1] / "scripts/replay_localization_study.py",
)
study = importlib.util.module_from_spec(spec)
spec.loader.exec_module(study)


def test_recording_replays_the_live_filter_exactly():
    scenario = replace(get_scenario("localization"), max_steps=24)
    record = record_localization_drive(scenario, seed=2, beams=2)
    replayed = replay_localization(scenario, record, beams=2).metrics

    assert replayed["steps"] == len(record.wheel_left) == len(record.true_poses)
    assert record.ranges.shape == (replayed["steps"], record.angles.size)
    for key in ("filter_rms", "filter_max", "filter_heading_rms_deg"):
        assert replayed[key] == record.live_metrics[key]


def test_record_matches_the_existing_localization_runner():
    scenario = replace(get_scenario("localization"), max_steps=20)
    record = record_localization_drive(
        scenario, seed=1, beams=2, likelihood_scale=0.8,
    )
    live = run_localization(
        scenario, seed=1, beams=2, likelihood_scale=0.8,
    ).metrics
    assert record.live_metrics == live


def test_replay_changes_only_the_filter_metrics():
    scenario = replace(get_scenario("localization"), max_steps=24)
    record = record_localization_drive(scenario, seed=0, beams=2)
    reference = replay_localization(scenario, record, beams=2).metrics
    other = replay_localization(
        scenario, record, beams=2, beam_selection="widest_pair",
        likelihood_scale=6.0,
    ).metrics

    assert other["steps"] == reference["steps"]
    assert other["filter_rms"] != reference["filter_rms"]
    assert set(other) == {
        "filter_rms", "filter_max", "filter_heading_rms_deg", "steps",
    }


def test_replay_rejects_malformed_recordings():
    scenario = replace(get_scenario("localization"), max_steps=8)
    record = record_localization_drive(scenario, seed=0, beams=2)
    broken = replace(record, wheel_right=record.wheel_right[:-1])
    with pytest.raises(ValueError, match="equal lengths"):
        replay_localization(scenario, broken, beams=2)

    empty = replace(
        record, wheel_left=record.wheel_left[:0],
        wheel_right=record.wheel_right[:0], ranges=record.ranges[:0],
        true_poses=record.true_poses[:0],
    )
    with pytest.raises(ValueError, match="no steps"):
        replay_localization(scenario, empty, beams=2)


def test_study_scores_every_setting_on_one_trajectory(monkeypatch, tmp_path):
    scenario = replace(get_scenario("localization"), max_steps=20)
    monkeypatch.setattr(study, "get_scenario", lambda name: scenario)
    seeds = [1, 3]
    data = json.loads(json.dumps(study.run_study(seeds), allow_nan=False))

    configs = {
        (row["beam_selection"], row["likelihood_scale"])
        for row in data["results"]
    }
    assert len(data["results"]) == len(configs) * len(seeds)
    assert len(data["trajectories"]) == len(seeds)

    for trajectory in data["trajectories"]:
        seed = trajectory["seed"]
        rows = [row for row in data["results"] if row["seed"] == seed]
        assert {row["metrics"]["steps"] for row in rows} == {
            trajectory["steps"]
        }
        reference = next(
            row for row in rows
            if row["beam_selection"] == "scan_order"
            and row["likelihood_scale"] == 0.2
        )
        assert reference["replay_matches_live"] is True
        assert trajectory["reference_max_abs_difference"] <= 1e-12
        for row in rows:
            assert row["paired_rms_delta"] == pytest.approx(
                row["metrics"]["filter_rms"] - reference["metrics"]["filter_rms"]
            )
            if row is not reference:
                assert row["replay_matches_live"] is False

    for group in data["summary"]:
        assert group["filter_rms"]["count"] == len(seeds)
        assert group["paired_rms_delta"]["count"] == len(seeds)

    plot = tmp_path / "plots" / "replay.png"
    study.plot_study(data, plot)
    assert plot.read_bytes().startswith(b"\x89PNG")
