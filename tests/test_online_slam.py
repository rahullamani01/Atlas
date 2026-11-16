"""End-to-end tests for online pose-graph correction (Phase 12)."""
from atlas.evaluation import (
    OnlineSlamMetrics,
    get_scenario,
    simulate_online_slam,
)


def test_online_scenario_declares_a_goal_sequence():
    scenario = get_scenario("online")

    assert scenario.kind == "slam_online"
    assert scenario.prior_world is not None
    assert scenario.goal_sequence == ((10.0, 9.5),)
    assert set(scenario.world().obstacles) == set(
        scenario.prior_world().obstacles
    )


def test_online_closure_corrects_the_graph_and_keeps_driving():
    trace = simulate_online_slam(
        get_scenario("online"), seed=0, particles=200, beams=30
    )
    metrics = trace.metrics

    assert metrics.reached
    assert not metrics.failed
    assert metrics.collisions == 0
    assert metrics.loop_closures >= 1
    assert metrics.closure_step > 0
    assert metrics.closure_time > 0.0
    assert metrics.correction > 0.1
    assert metrics.pre_closure_rms > 0.5
    assert metrics.post_closure_rms < metrics.pre_closure_rms * 0.5
    assert metrics.ate_reduction > 0.5
    assert metrics.slam_map_error < metrics.raw_map_error
    assert metrics.map_error_reduction > 0
    assert metrics.final_error < 0.5
    assert metrics.filter_rms < 0.5

    assert len(trace.node_pre) < len(trace.node_post)
    assert len(trace.node_post) == len(trace.node_truth)
    assert len(trace.trajectory) == len(trace.filter_errors) + 1
    assert trace.loops
    assert trace.closure_step > 0
    assert trace.raw_grid.blocked.shape == trace.slam_grid.blocked.shape
    assert trace.slam_grid.blocked.shape == trace.final_grid.blocked.shape
    assert trace.final_grid.blocked.shape == trace.truth_grid.blocked.shape


def test_online_metrics_expose_a_flat_dictionary():
    metrics = OnlineSlamMetrics(
        reached=True,
        failed=False,
        collisions=0,
        duration=31.0,
        final_error=0.05,
        nodes=130,
        loop_closures=2,
        closure_step=450,
        closure_time=22.5,
        correction=1.1,
        pre_closure_rms=0.7,
        post_closure_rms=0.1,
        ate_reduction=0.86,
        slam_rms=0.1,
        slam_final=0.12,
        graph_cost_before=17.0,
        graph_cost_after=0.005,
        raw_map_error=170,
        slam_map_error=120,
        map_error_reduction=50,
        filter_rms=0.09,
        filter_heading_rms_deg=0.7,
    ).as_dict()

    assert metrics["reached"] == 1.0
    assert metrics["loop_closures"] == 2
    assert metrics["closure_step"] == 450
    assert metrics["ate_reduction"] == 0.86
    assert metrics["map_error_reduction"] == 50


def test_online_run_is_reproducible_for_a_seed():
    first = simulate_online_slam(
        get_scenario("online"), seed=3, particles=200, beams=30
    ).metrics.as_dict()
    second = simulate_online_slam(
        get_scenario("online"), seed=3, particles=200, beams=30
    ).metrics.as_dict()

    assert first == second
