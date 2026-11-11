import math

import pytest

from atlas.evaluation import (
    KinematicsMetrics,
    MetricSummary,
    NavigationMetrics,
    ScenarioResult,
    TrackingMetrics,
    aggregate,
    path_errors,
    rms,
    segment_projection,
    summarize,
)


def test_rms_matches_the_definition():
    assert rms([3.0, 4.0]) == pytest.approx(math.sqrt(12.5))


def test_rms_rejects_an_empty_sequence():
    with pytest.raises(ValueError, match="at least one"):
        rms([])


def test_segment_projection_uses_the_clamped_nearest_point():
    distance, heading = segment_projection((2.0, 3.0), (0.0, 0.0), (4.0, 0.0))
    assert distance == pytest.approx(3.0)
    assert heading == pytest.approx(0.0)


def test_segment_projection_handles_a_degenerate_segment():
    distance, heading = segment_projection((3.0, 4.0), (0.0, 0.0), (0.0, 0.0))
    assert distance == pytest.approx(5.0)
    assert heading == pytest.approx(0.0)


def test_path_errors_reports_distance_and_wrapped_heading_error():
    path = [(0.0, 0.0), (4.0, 0.0)]
    distances, headings = path_errors(
        path, [(2.0, 1.0, 0.25), (2.0, 1.0, math.pi)]
    )
    assert distances == pytest.approx([1.0, 1.0])
    assert headings[0] == pytest.approx(-0.25)
    assert headings[1] == pytest.approx(-math.pi)


def test_path_errors_requires_a_segment():
    with pytest.raises(ValueError, match="two points"):
        path_errors([(0.0, 0.0)], [(0.0, 0.0, 0.0)])


def test_summarize_uses_sample_standard_deviation():
    summary = summarize([1.0, 2.0, 3.0, 4.0])
    assert summary.count == 4
    assert summary.mean == pytest.approx(2.5)
    assert summary.std == pytest.approx(math.sqrt(5.0 / 3.0))
    assert summary.minimum == 1.0
    assert summary.maximum == 4.0


def test_summarize_of_one_value_has_zero_spread():
    summary = summarize([7.0])
    assert summary.as_dict() == {
        "count": 1,
        "mean": 7.0,
        "std": 0.0,
        "minimum": 7.0,
        "maximum": 7.0,
    }


def test_summarize_rejects_an_empty_sequence():
    with pytest.raises(ValueError, match="at least one"):
        summarize([])


def test_metric_objects_expose_flat_float_dictionaries():
    tracking = TrackingMetrics(
        final_error=0.1,
        cross_track_rms=0.02,
        cross_track_max=0.05,
        heading_rms_deg=2.0,
        control_effort=1.5,
        driven_length=10.0,
        duration=20.0,
        collisions=0,
        reached=True,
    ).as_dict()
    assert tracking["reached"] == 1.0
    assert tracking["collisions"] == 0

    navigation = NavigationMetrics(
        reached=False,
        failed=True,
        collisions=0,
        duration=1.0,
        final_error=3.0,
        replans=0,
        detected_cells=0,
    ).as_dict()
    assert navigation["failed"] == 1.0
    assert navigation["reached"] == 0.0


def test_aggregate_groups_metrics_by_scenario():
    results = [
        ScenarioResult("alpha", 0, {"error": 1.0, "reached": 1.0}),
        ScenarioResult("alpha", 1, {"error": 3.0, "reached": 0.0}),
        ScenarioResult("beta", 0, {"error": 5.0, "reached": 1.0}),
    ]
    summary = aggregate(results)
    assert set(summary) == {"alpha", "beta"}
    assert summary["alpha"]["error"].mean == pytest.approx(2.0)
    assert summary["alpha"]["error"].count == 2
    assert summary["beta"]["error"].mean == pytest.approx(5.0)
    assert summary["beta"]["error"].std == 0.0


def test_aggregate_of_no_results_is_empty():
    assert aggregate([]) == {}


def test_metric_summary_round_trips_to_a_dictionary():
    summary = MetricSummary(
        count=3, mean=1.0, std=0.5, minimum=0.0, maximum=2.0
    )
    assert summary.as_dict() == {
        "count": 3,
        "mean": 1.0,
        "std": 0.5,
        "minimum": 0.0,
        "maximum": 2.0,
    }


def test_metric_dataclasses_are_immutable():
    metrics = KinematicsMetrics(1.0, 2.0, 3.0)
    with pytest.raises(Exception):
        metrics.euler_error = 0.0  # type: ignore[misc]
