import numpy as np
import pytest

from atlas.perception import DynamicObstacleMap
from atlas.planning import OccupancyGrid
from atlas.robot.differential_drive import Pose
from atlas.sensors import Lidar
from atlas.simulation.world import RectObstacle, World


def lidar():
    return Lidar(max_range=10.0, fov=2 * np.pi,
                 angle_increment=np.pi / 180, range_sigma=0.0)


def single_ray():
    fov = 0.01
    return Lidar(max_range=10.0, fov=fov, angle_increment=fov,
                 angle_offset=fov / 2, range_sigma=0.0)


def empty_grid():
    return OccupancyGrid.from_world(World(10, 10), resolution=0.25,
                                    inflate_radius=0.0)


def test_known_obstacles_are_not_reported_as_new():
    world = World(10, 10, [RectObstacle(4, 4, 5, 5)])
    grid = OccupancyGrid.from_world(world, resolution=0.25, inflate_radius=0.25)
    detector = DynamicObstacleMap(grid)
    scan = lidar().scan(world, Pose(2, 2, 0))
    for _ in range(5):
        assert detector.integrate(scan) == []
    assert detector.detected_count == 0


def test_world_walls_are_not_reported_as_new():
    world = World(10, 10)
    detector = DynamicObstacleMap(empty_grid())
    scan = lidar().scan(world, Pose(1, 1, 0))
    for _ in range(5):
        detector.integrate(scan)
    assert detector.detected_count == 0


def test_new_obstacle_is_confirmed_and_blocks_the_grid():
    known = World(10, 10)
    truth = World(10, 10, [RectObstacle(4, 4, 5, 5)])
    detector = DynamicObstacleMap(empty_grid(), clearance=0.25)
    scan = lidar().scan(truth, Pose(2, 2, 0))

    for _ in range(4):
        detector.integrate(scan)

    assert detector.detected_count > 0
    near_face = detector.grid.world_to_cell(4.1, 4.5)
    assert not detector.grid.is_free(near_face)
    assert not np.array_equal(detector.grid.blocked, detector.known.blocked)


def test_a_single_scan_does_not_confirm():
    truth = World(10, 10, [RectObstacle(5, 1, 6, 3)])
    detector = DynamicObstacleMap(empty_grid(), confirm_threshold=2.0,
                                  decay=0.8, tolerance=0, clearance=0.0)
    scan = single_ray().scan(truth, Pose(2, 2, 0))

    assert detector.integrate(scan) == []
    detector.integrate(scan)
    detected = detector.integrate(scan)
    assert detected


def test_clearance_expands_the_footprint():
    truth = World(10, 10, [RectObstacle(4, 4, 5, 5)])
    scan = lidar().scan(truth, Pose(2, 2, 0))

    tight = DynamicObstacleMap(empty_grid(), clearance=0.0)
    grown = DynamicObstacleMap(empty_grid(), clearance=0.5)
    for _ in range(4):
        tight.integrate(scan)
        grown.integrate(scan)

    assert grown.detected_count > tight.detected_count


def test_transient_obstacle_decays_away():
    truth = World(10, 10, [RectObstacle(4, 4, 5, 5)])
    clean = World(10, 10)
    detector = DynamicObstacleMap(empty_grid(), decay=0.8,
                                  unblock_threshold=0.5)
    seen = lidar().scan(truth, Pose(2, 2, 0))
    empty = lidar().scan(clean, Pose(2, 2, 0))

    for _ in range(4):
        detector.integrate(seen)
    assert detector.detected_count > 0

    for _ in range(20):
        detector.integrate(empty)

    assert detector.detected_count == 0
    assert np.array_equal(detector.grid.blocked, detector.known.blocked)


def test_reset_restores_the_known_map():
    truth = World(10, 10, [RectObstacle(4, 4, 5, 5)])
    detector = DynamicObstacleMap(empty_grid())
    scan = lidar().scan(truth, Pose(2, 2, 0))
    for _ in range(4):
        detector.integrate(scan)
    assert detector.detected_count > 0

    detector.reset()
    assert detector.detected_count == 0
    assert np.array_equal(detector.grid.blocked, detector.known.blocked)
    assert not np.any(detector.evidence)


def test_invalid_parameters_are_rejected():
    grid = empty_grid()
    with pytest.raises(ValueError, match="confirm_threshold"):
        DynamicObstacleMap(grid, confirm_threshold=0)
    with pytest.raises(ValueError, match="unblock_threshold"):
        DynamicObstacleMap(grid, confirm_threshold=2.0, unblock_threshold=2.0)
    with pytest.raises(ValueError, match="decay"):
        DynamicObstacleMap(grid, decay=1.0)
    with pytest.raises(ValueError, match="tolerance"):
        DynamicObstacleMap(grid, tolerance=-1)
    with pytest.raises(ValueError, match="clearance"):
        DynamicObstacleMap(grid, clearance=-0.1)
