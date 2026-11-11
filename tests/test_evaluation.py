import pytest

from dataclasses import replace

from atlas.evaluation import (
    Scenario,
    aggregate,
    default_scenarios,
    get_scenario,
    run_kinematics,
    run_localization,
    run_planning,
    run_recovery,
    run_suite,
    run_tracking,
)


def test_default_suite_covers_every_stage():
    scenarios = default_scenarios()
    kinds = {scenario.kind for scenario in scenarios}
    assert kinds == {
        "kinematics",
        "planning",
        "tracking",
        "localization",
        "recovery",
    }
    assert {scenario.name for scenario in scenarios} >= {
        "tracking_pid",
        "tracking_p",
    }


def test_unknown_scenario_raises_a_clear_error():
    with pytest.raises(KeyError, match="unknown scenario"):
        get_scenario("does-not-exist")


@pytest.mark.parametrize(
    "kwargs, message",
    [
        ({"name": "", "kind": "planning"}, "name"),
        ({"name": "x", "kind": "nope"}, "kind"),
        ({"name": "x", "kind": "planning", "resolution": 0}, "resolution"),
        ({"name": "x", "kind": "planning", "inflation": -1}, "inflation"),
        ({"name": "x", "kind": "planning", "dt": 0}, "dt"),
        ({"name": "x", "kind": "planning", "max_steps": 0}, "max_steps"),
        (
            {"name": "x", "kind": "tracking", "controller": "pd"},
            "controller",
        ),
        (
            {"name": "x", "kind": "recovery", "inject_step": 5},
            "provided together",
        ),
        (
            {
                "name": "x",
                "kind": "recovery",
                "inject_step": -1,
                "inject_rect": (2, 2, 3, 3),
            },
            "inject_step",
        ),
        (
            {
                "name": "x",
                "kind": "recovery",
                "inject_step": 5,
                "inject_rect": (3, 2, 2, 3),
            },
            "inject_rect",
        ),
    ],
)
def test_scenario_validation(kwargs, message):
    with pytest.raises(ValueError, match=message):
        Scenario(**kwargs)


def test_integrators_are_ordered_by_accuracy():
    metrics = run_kinematics(get_scenario("kinematics")).metrics
    assert metrics["exact_error"] < metrics["midpoint_error"]
    assert metrics["midpoint_error"] < metrics["euler_error"]
    assert metrics["exact_error"] < 1e-9
    assert metrics["euler_error"] > 1e-5


def test_planning_is_optimal_and_expands_fewer_cells_than_dijkstra():
    metrics = run_planning(get_scenario("planning")).metrics
    assert metrics["path_cost"] == pytest.approx(29.02, abs=0.1)
    assert metrics["a_star_expanded"] < metrics["dijkstra_expanded"]
    assert metrics["expansion_reduction"] > 0.5


def test_pid_has_lower_cross_track_error_than_p_only():
    pid = run_tracking(get_scenario("tracking_pid")).metrics
    p_only = run_tracking(get_scenario("tracking_p")).metrics

    assert pid["reached"] == 1.0
    assert p_only["reached"] == 1.0
    assert pid["collisions"] == 0
    assert p_only["collisions"] == 0
    assert pid["cross_track_rms"] < p_only["cross_track_rms"]
    assert pid["final_error"] < 0.15


def test_localization_beats_raw_odometry():
    metrics = run_localization(
        get_scenario("localization"), seed=0, particles=200, beams=24
    ).metrics
    assert metrics["filter_rms"] < metrics["odometry_rms"] / 10.0
    assert metrics["filter_rms"] < 0.5
    assert metrics["collisions"] == 0


def test_recovery_detects_replans_and_reaches_the_goal():
    metrics = run_recovery(get_scenario("recovery"), seed=0).metrics
    assert metrics["reached"] == 1.0
    assert metrics["failed"] == 0.0
    assert metrics["collisions"] == 0
    assert metrics["replans"] >= 1
    assert metrics["detected_cells"] > 0
    assert metrics["final_error"] < 0.2


def test_a_seeded_localization_run_is_reproducible():
    scenario = replace(get_scenario("localization"), max_steps=250)
    first = run_localization(scenario, seed=3, particles=100, beams=16)
    second = run_localization(scenario, seed=3, particles=100, beams=16)
    assert first.metrics == second.metrics


def test_run_suite_and_aggregate_count_every_seed():
    scenarios = [get_scenario("kinematics"), get_scenario("planning")]
    results = run_suite(scenarios, seeds=[0, 1, 2])
    assert len(results) == 6

    summary = aggregate(results)
    assert set(summary) == {"kinematics", "planning"}
    assert summary["planning"]["path_cost"].count == 3
    assert summary["planning"]["path_cost"].std == pytest.approx(0.0)
    assert summary["kinematics"]["exact_error"].mean < 1e-9
