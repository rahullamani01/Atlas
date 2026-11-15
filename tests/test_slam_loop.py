"""End-to-end tests for pose-graph loop closure (Phase 11)."""
from atlas.evaluation import (
    SlamMetrics,
    get_scenario,
    simulate_slam,
)


def test_loop_scenario_declares_its_prior():
    scenario = get_scenario("loop")

    assert scenario.kind == "slam_loop"
    assert scenario.prior_world is not None
    assert set(scenario.world().obstacles) == set(scenario.prior_world().obstacles)


def test_loop_closure_corrects_drift_and_reaches_the_goal():
    trace = simulate_slam(
        get_scenario("loop"), seed=0, particles=200, beams=30
    )
    metrics = trace.metrics

    assert metrics.reached
    assert not metrics.failed
    assert metrics.collisions == 0
    assert metrics.loop_closures >= 1
    assert metrics.odometry_rms > 0.5
    assert metrics.slam_rms < metrics.odometry_rms * 0.5
    assert metrics.slam_max <= metrics.odometry_max
    assert metrics.ate_reduction > 0.5
    assert metrics.graph_cost_after < metrics.graph_cost_before
    assert metrics.slam_map_error < metrics.raw_map_error
    assert metrics.map_error_reduction > 0
    assert metrics.filter_rms < 0.5

    assert len(trace.node_raw) == len(trace.node_corrected)
    assert len(trace.node_raw) == len(trace.node_truth)
    assert len(trace.trajectory) == len(trace.filter_errors) + 1
    assert trace.loops
    assert trace.prior_grid.blocked.shape == trace.slam_grid.blocked.shape
    assert trace.slam_grid.blocked.shape == trace.truth_grid.blocked.shape


def test_slam_metrics_expose_a_flat_dictionary():
    metrics = SlamMetrics(
        reached=True,
        failed=False,
        collisions=0,
        duration=20.0,
        final_error=0.15,
        nodes=100,
        loop_closures=3,
        odometry_rms=1.0,
        odometry_max=2.0,
        slam_rms=0.1,
        slam_max=0.2,
        slam_final=0.05,
        ate_reduction=0.9,
        graph_cost_before=100.0,
        graph_cost_after=0.01,
        prior_map_error=0,
        raw_map_error=180,
        slam_map_error=130,
        map_error_reduction=50,
        filter_rms=0.09,
        filter_heading_rms_deg=0.7,
    ).as_dict()

    assert metrics["reached"] == 1.0
    assert metrics["loop_closures"] == 3
    assert metrics["ate_reduction"] == 0.9
    assert metrics["map_error_reduction"] == 50


def test_loop_run_is_reproducible_for_a_seed():
    first = simulate_slam(
        get_scenario("loop"), seed=3, particles=200, beams=30
    ).metrics.as_dict()
    second = simulate_slam(
        get_scenario("loop"), seed=3, particles=200, beams=30
    ).metrics.as_dict()

    assert first == second
