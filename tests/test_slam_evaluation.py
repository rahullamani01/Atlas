"""End-to-end tests for mapping and map repair during navigation (Phase 10)."""
from dataclasses import replace

from atlas.evaluation import (
    MappingMetrics,
    get_scenario,
    run_mapping,
    simulate_mapping,
)
from atlas.simulation.world import RectObstacle


def test_slam_scenario_declares_a_wrong_prior():
    scenario = get_scenario("slam")

    assert scenario.kind == "slam"
    assert scenario.prior_world is not None

    truth = set(scenario.world().obstacles)
    prior = set(scenario.prior_world().obstacles)
    assert truth - prior == {RectObstacle(6.0, 14.0, 7.0, 15.0)}
    assert prior - truth == {RectObstacle(1.0, 5.0, 2.2, 8.0)}


def test_mapping_metrics_expose_a_flat_dictionary():
    metrics = MappingMetrics(
        reached=True,
        failed=False,
        collisions=0,
        duration=1.0,
        final_error=0.1,
        replans=2,
        map_updates=3,
        added_cells=4,
        removed_cells=5,
        known_ratio=1.0,
        observed_ratio=0.2,
        prior_map_error=10,
        final_map_error=4,
        map_error_reduction=6,
        map_repair_ratio=0.6,
        filter_rms=0.05,
        filter_max=0.1,
        filter_heading_rms_deg=0.3,
    ).as_dict()

    assert metrics["reached"] == 1.0
    assert metrics["failed"] == 0.0
    assert metrics["prior_map_error"] == 10
    assert metrics["final_map_error"] == 4
    assert metrics["map_error_reduction"] == 6
    assert metrics["map_repair_ratio"] == 0.6


def test_slam_trace_repairs_the_map_and_reaches_the_goal():
    trace = simulate_mapping(
        get_scenario("slam"), seed=0, particles=300, beams=40
    )
    metrics = trace.metrics

    assert metrics.reached
    assert not metrics.failed
    assert metrics.collisions == 0
    assert metrics.replans >= 1
    assert metrics.map_updates > 0
    assert metrics.added_cells > 0
    assert metrics.removed_cells > 0
    assert metrics.known_ratio == 1.0
    assert 0.0 < metrics.observed_ratio <= 1.0
    assert metrics.prior_map_error > 0
    assert metrics.final_map_error <= metrics.prior_map_error
    assert metrics.map_error_reduction > 0
    assert metrics.map_repair_ratio > 0.0
    assert metrics.final_error < 0.3
    assert metrics.filter_rms < 0.5

    assert len(trace.trajectory) == len(trace.filter_errors) + 1
    assert len(trace.map_errors) >= 1
    assert trace.prior_grid.blocked.shape == trace.final_grid.blocked.shape
    assert trace.final_grid.blocked.shape == trace.truth_grid.blocked.shape
    assert trace.added_cells
    assert trace.removed_cells
    assert any(event.kind == "map" for event in trace.events)
    assert any(event.kind == "replan" for event in trace.events)


def test_slam_run_is_reproducible_for_a_seed():
    scenario = replace(get_scenario("slam"), max_steps=350)
    first = run_mapping(scenario, seed=3, particles=200, beams=30)
    second = run_mapping(scenario, seed=3, particles=200, beams=30)
    assert first.metrics == second.metrics
