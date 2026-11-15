"""Loop-candidate and verification tests for Phase 11."""
import pytest

from atlas.geometry import relative_pose
from atlas.robot.differential_drive import Pose
from atlas.slam.loop_closure import LoopCloser
from atlas.slam.scan_matcher import MatchResult


class _StubMatcher:
    """Return a fixed match so candidate logic can be tested in isolation."""

    def __init__(self, pose: Pose, score: float) -> None:
        self.pose = pose
        self.score = score

    def match(self, scan, prior) -> MatchResult:
        return MatchResult(self.pose, self.score, 1)


def _estimates() -> list[Pose]:
    poses = [Pose(0.5, 0.0, 0.0), Pose(1.9, 0.0, 0.0), Pose(2.5, 0.0, 0.0)]
    poses.extend(Pose(5.0, 5.0, 0.0) for _ in range(7))
    poses.append(Pose(0.0, 0.0, 0.0))
    return poses


def test_candidates_respect_radius_and_time_gap():
    closer = LoopCloser(
        _StubMatcher(Pose(), 0.0), radius=2.0, min_gap=3, max_candidates=2
    )

    assert closer.candidates(_estimates(), 10) == [0, 1]
    assert closer.candidates(_estimates(), 2) == []


def test_detect_returns_a_relative_measurement_for_each_candidate():
    estimates = _estimates()
    matched = Pose(0.2, 0.3, 0.1)
    closer = LoopCloser(
        _StubMatcher(matched, 0.02), radius=2.0, min_gap=3, max_candidates=2
    )

    closures = closer.detect(estimates[10], estimates[10], estimates, estimates, 10)

    assert {closure.old for closure in closures} == {0, 1}
    expected = relative_pose(
        (estimates[0].x, estimates[0].y, estimates[0].theta),
        (matched.x, matched.y, matched.theta),
    )
    assert closures[0].measurement == pytest.approx(expected)
    assert closures[0].score == 0.02


def test_a_poor_match_is_rejected():
    estimates = _estimates()
    closer = LoopCloser(
        _StubMatcher(Pose(0.2, 0.3, 0.1), 0.9), radius=2.0, min_gap=3
    )

    assert closer.detect(estimates[10], estimates[10], estimates, estimates, 10) == []


def test_invalid_configuration_is_rejected():
    matcher = _StubMatcher(Pose(), 0.0)

    with pytest.raises(ValueError):
        LoopCloser(matcher, radius=0.0)
    with pytest.raises(ValueError):
        LoopCloser(matcher, min_gap=0)
    with pytest.raises(ValueError):
        LoopCloser(matcher, max_score=0.0)
    with pytest.raises(ValueError):
        LoopCloser(matcher, max_candidates=0)
    with pytest.raises(IndexError):
        LoopCloser(matcher).candidates([Pose()], 5)
