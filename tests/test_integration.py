from dataclasses import replace

import pytest

from atlas.evaluation import (
    get_scenario,
    run_integrated,
    simulate_integrated,
)


def test_integrated_run_reaches_the_goal_on_the_estimate():
    metrics = run_integrated(
        get_scenario("integrated"), seed=0, particles=300, beams=40
    ).metrics

    assert metrics["reached"] == 1.0
    assert metrics["failed"] == 0.0
    assert metrics["collisions"] == 0
    assert metrics["replans"] >= 1
    assert metrics["detected_cells"] > 0
    assert metrics["final_error"] < 0.2
    assert metrics["filter_rms"] < 0.5
    assert metrics["filter_rms"] < metrics["odometry_rms"] / 10.0


def test_integrated_run_is_reproducible_for_a_seed():
    scenario = replace(get_scenario("integrated"), max_steps=300)
    first = run_integrated(scenario, seed=3, particles=200, beams=30)
    second = run_integrated(scenario, seed=3, particles=200, beams=30)
    assert first.metrics == second.metrics


def test_integrated_trace_aligns_trajectory_and_errors():
    trace = simulate_integrated(
        replace(get_scenario("integrated"), max_steps=300),
        seed=1,
        particles=200,
        beams=30,
    )
    assert len(trace.trajectory) == len(trace.filter_errors) + 1
    assert len(trace.odometry_errors) == len(trace.filter_errors)
    assert len(trace.heading_errors) == len(trace.filter_errors)
    assert len(trace.detected_counts) == len(trace.filter_errors)
    assert any(event.kind == "detect" for event in trace.events)


def test_integrated_metrics_are_flat_dictionaries():
    metrics = run_integrated(
        replace(get_scenario("integrated"), max_steps=200),
        seed=0,
        particles=150,
        beams=24,
    ).metrics
    assert all(
        isinstance(value, (int, float)) for value in metrics.values()
    )
    assert "filter_rms" in metrics
    assert "detected_cells" in metrics
