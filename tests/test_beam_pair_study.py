from dataclasses import replace
import importlib.util
from itertools import combinations
import json
from pathlib import Path

import numpy as np
import pytest

from atlas.evaluation import get_scenario, run_localization
from atlas.localization import ParticleFilter
from atlas.planning import OccupancyGrid
from atlas.robot.differential_drive import Pose
from atlas.sensors.lidar import LidarScan


spec = importlib.util.spec_from_file_location(
    "beam_pair_study", Path(__file__).resolve().parents[1] / "scripts/beam_pair_study.py"
)
study = importlib.util.module_from_spec(spec)
spec.loader.exec_module(study)


def make_scan(degrees, hits):
    angles = np.deg2rad(degrees)
    return LidarScan(np.where(hits, 2.0, 10.0), angles, angles.copy(), (0, 0), 10.0)


@pytest.mark.parametrize(("degrees", "hits"), [
    ([-179, 0, 179], [True, True, True]),
    ([44, -179, -140, 210, 128], [False, True, True, True, True]),
    ([50, 12, 40, 20], [True] * 4),
    ([70, 70, 70], [True] * 3),
    ([0, 90, 180], [False, False, False]),
    ([0, 90, 180], [False, True, False]),
    ([0, 90, 180], [True, False, True]),
])
def test_widest_pair_matches_exhaustive_circular_separation(degrees, hits):
    scan = make_scan(degrees, hits)
    pf = ParticleFilter(OccupancyGrid(10, 10, 1), max_beams=2, beam_selection="widest_pair")
    picked = pf._selected_hits(scan)
    valid = np.flatnonzero(hits)
    assert len(set(picked)) == len(picked) == min(len(valid), 2)
    assert set(picked) <= set(valid)
    if len(valid) < 2:
        return

    def separation(i, j):
        diff = scan.angles[i] - scan.angles[j]
        return abs(np.arctan2(np.sin(diff), np.cos(diff)))

    assert separation(*picked) == pytest.approx(max(separation(i, j) for i, j in combinations(valid, 2)))


def test_update_and_residual_use_the_same_pair_as_an_explicitly_masked_scan():
    grid = OccupancyGrid(12, 12, 1)
    grid.set_blocked((6, 4), True)
    scan = make_scan([-170, -40, 10, 60, 170], [True] * 5)
    masked = replace(scan, ranges=np.where([True, False, True, False, False], scan.ranges, 10.0))
    widest = ParticleFilter(grid, seed=3, max_beams=2, beam_selection="widest_pair")
    control = ParticleFilter(grid, seed=3, max_beams=2)
    for pf in (widest, control):
        pf.initialize(Pose(4.5, 4.5, 0), position_std=0.75)
    widest.update(scan, resample=False)
    control.update(masked, resample=False)
    np.testing.assert_allclose(widest.weights, control.weights)
    assert widest.scan_residual(scan) == pytest.approx(control.scan_residual(masked))


@pytest.mark.parametrize(("policy", "budget"), [("invalid", 2), ("widest_pair", 3)])
def test_invalid_selector_configuration_is_rejected(policy, budget):
    with pytest.raises(ValueError):
        ParticleFilter(OccupancyGrid(3, 3, 1), max_beams=budget, beam_selection=policy)


def test_short_paired_study_reproduces_default_and_plots_saved_data(monkeypatch, tmp_path):
    scenario = replace(get_scenario("localization"), max_steps=30)
    monkeypatch.setattr(study, "get_scenario", lambda name: scenario)
    data = json.loads(json.dumps(study.run_study([0])))
    pair, = data["results"]
    assert pair["scan_order"] == run_localization(scenario, seed=0, beams=2).metrics
    assert pair["widest_pair"]["filter_rms"] != pair["scan_order"]["filter_rms"]
    delta = pair["widest_pair"]["filter_rms"] - pair["scan_order"]["filter_rms"]
    assert data["paired_rms_delta_widest_minus_scan_order"]["mean"] == delta
    plot = tmp_path / "plots" / "pairs.png"
    study.plot_study(data, plot)
    assert plot.read_bytes().startswith(b"\x89PNG")
