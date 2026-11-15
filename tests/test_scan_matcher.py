"""Correlative scan-matching tests for Phase 11."""
import math

import pytest

from atlas.evaluation.scenarios import demo_world
from atlas.localization.likelihood_field import DistanceField
from atlas.planning.grid import OccupancyGrid
from atlas.robot.differential_drive import Pose
from atlas.sensors.lidar import Lidar
from atlas.slam.scan_matcher import ScanMatcher


def _scene():
    world = demo_world()
    grid = OccupancyGrid.from_world(world, resolution=0.25)
    field = DistanceField.from_grid(grid)
    lidar = Lidar(max_range=10.0, range_sigma=0.0, seed=0)
    return world, lidar, field


def test_matcher_recovers_a_displaced_pose():
    world, lidar, field = _scene()
    truth = Pose(3.0, 3.0, 0.4)
    scan = lidar.scan(world, truth)
    matcher = ScanMatcher(field)
    prior = Pose(3.45, 2.70, 0.52)

    result = matcher.match(scan, prior)

    assert math.hypot(result.pose.x - truth.x, result.pose.y - truth.y) < 0.12
    assert abs(result.pose.theta - truth.theta) < 0.12
    assert result.evaluations > 0
    assert result.score < matcher.score(scan, prior)


def test_score_is_lower_at_the_true_pose():
    world, lidar, field = _scene()
    truth = Pose(3.0, 3.0, 0.4)
    scan = lidar.scan(world, truth)
    matcher = ScanMatcher(field)

    assert matcher.score(scan, truth) < 0.5
    assert matcher.score(scan, truth) < matcher.score(scan, Pose(4.5, 4.5, 0.4))
    assert matcher.score(scan, Pose(4.5, 4.5, 0.4)) > 0.2


def test_invalid_matcher_configuration_is_rejected():
    _, _, field = _scene()

    with pytest.raises(ValueError):
        ScanMatcher(field, xy_window=-1.0)
    with pytest.raises(ValueError):
        ScanMatcher(field, theta_step=0.0)
    with pytest.raises(ValueError):
        ScanMatcher(field, max_beams=0)
    with pytest.raises(ValueError):
        ScanMatcher(field, max_distance=0.0)
