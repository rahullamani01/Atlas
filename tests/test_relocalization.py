import math
from dataclasses import replace

import pytest

from atlas.evaluation import (
    get_scenario,
    run_relocalization,
    simulate_relocalization,
)
from atlas.localization import LocalizationMonitor, ParticleFilter, Relocalizer
from atlas.planning import OccupancyGrid
from atlas.robot.differential_drive import Pose
from atlas.sensors import Lidar


def _grid_truth_scan(noise: float = 0.0):
    scenario = get_scenario("relocalization")
    world = scenario.world()
    grid = OccupancyGrid.from_world(
        world,
        resolution=scenario.resolution,
        inflate_radius=scenario.inflation,
    )
    truth = Pose(*scenario.kidnap_pose)
    lidar = Lidar(max_range=10.0, range_sigma=noise, seed=0)
    return grid, truth, lidar.scan(world, truth)


def test_scan_residual_is_low_at_the_true_pose_and_high_elsewhere():
    grid, truth, scan = _grid_truth_scan()
    pf = ParticleFilter(grid, n_particles=100, max_beams=60, seed=0)
    pf.initialize(truth, position_std=0.0, heading_std=0.0)

    assert pf.scan_residual(scan) < 0.1
    wrong = Pose(truth.x + 3.0, truth.y, truth.theta)
    assert pf.scan_residual(scan, wrong) > 0.5


def test_monitor_needs_a_sustained_failure_and_rearms():
    monitor = LocalizationMonitor(residual_limit=0.5, patience=3)

    assert not monitor.update(0.6)
    assert not monitor.update(0.6)
    assert monitor.update(0.6)
    assert not monitor.update(0.6)  # already triggered

    monitor.reset()
    assert not monitor.update(0.1)
    assert not monitor.update(0.6)
    assert not monitor.update(0.1)  # a good scan clears the streak
    assert not monitor.update(0.6)
    assert not monitor.update(0.6)
    assert monitor.update(0.6)


def test_global_search_finds_the_kidnapped_pose():
    grid, truth, scan = _grid_truth_scan()
    relocalizer = Relocalizer(grid, max_beams=60)

    hypotheses = relocalizer.search(scan)

    assert hypotheses
    best = hypotheses[0].pose
    assert math.hypot(best.x - truth.x, best.y - truth.y) < 0.35
    heading_error = abs(
        math.atan2(
            math.sin(best.theta - truth.theta),
            math.cos(best.theta - truth.theta),
        )
    )
    assert heading_error < math.radians(8.0)


def test_search_keeps_a_candidate_bank_and_accumulates_observations():
    grid, _, scan = _grid_truth_scan()
    relocalizer = Relocalizer(grid, max_beams=60)

    relocalizer.search(scan)
    assert relocalizer.hypothesis_count >= 1
    assert relocalizer.observations == 1

    relocalizer.observe(scan, dtheta=0.2)
    assert relocalizer.observations == 2
    assert relocalizer.best().score >= 0.0


def test_relocalization_run_detects_recovers_and_reaches_the_goal():
    metrics = run_relocalization(
        get_scenario("relocalization"),
        seed=0,
        particles=300,
        beams=40,
    ).metrics

    assert metrics["reached"] == 1.0
    assert metrics["failed"] == 0.0
    assert metrics["collisions"] == 0
    assert metrics["kidnapped"] == 1.0
    assert metrics["relocalizations"] == 1
    assert metrics["relocalized"] == 1.0
    assert 0.0 <= metrics["detection_latency"] < 1.0
    assert 0.0 <= metrics["recovery_time"] < 4.0
    assert metrics["error_at_kidnap"] > 8.0
    assert metrics["final_error"] < 0.2
    assert metrics["post_recovery_rms"] < 0.3
    assert metrics["post_recovery_heading_rms_deg"] < 2.0


def test_relocalization_trace_records_the_recovery_timeline():
    trace = simulate_relocalization(
        get_scenario("relocalization"),
        seed=1,
        particles=200,
        beams=30,
    )

    kinds = {event.kind for event in trace.events}
    assert {"kidnap", "lost", "relocalized", "resumed"} <= kinds
    assert len(trace.residuals) == len(trace.filter_errors)
    assert max(trace.residuals) > 0.5
    assert trace.metrics.relocalized == 1.0


def test_relocalization_is_reproducible_for_a_seed():
    scenario = replace(get_scenario("relocalization"), max_steps=300)
    first = run_relocalization(scenario, seed=3, particles=200, beams=30)
    second = run_relocalization(scenario, seed=3, particles=200, beams=30)
    assert first.metrics == second.metrics


def test_a_run_without_a_kidnap_never_relocalizes():
    scenario = replace(
        get_scenario("relocalization"),
        max_steps=300,
        kidnap_step=None,
        kidnap_pose=None,
    )
    metrics = run_relocalization(
        scenario, seed=0, particles=200, beams=30
    ).metrics

    assert metrics["kidnapped"] == 0.0
    assert metrics["relocalizations"] == 0
    assert metrics["relocalized"] == 0.0


def test_relocalizer_rejects_invalid_configuration():
    grid, _, _ = _grid_truth_scan()
    with pytest.raises(ValueError, match="max_beams"):
        Relocalizer(grid, max_beams=0)
    with pytest.raises(ValueError, match="coarse_stride"):
        Relocalizer(grid, coarse_stride=0)
    with pytest.raises(ValueError, match="patience"):
        LocalizationMonitor(patience=0)
