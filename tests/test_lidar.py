import math

import numpy as np
import pytest

from atlas.robot.differential_drive import Pose
from atlas.sensors import Lidar
from atlas.simulation.world import RectObstacle, World

FORWARD = 180  # index of the 0 rad ray in a full 360-ray scan


def full_lidar(**kwargs):
    kwargs.setdefault("max_range", 50.0)
    return Lidar(fov=2 * math.pi, angle_increment=math.pi / 180, **kwargs)


def single_ray(angle=0.0, **kwargs):
    """A one-ray sensor pointing at `angle` in the body frame."""
    fov = 0.01
    kwargs.setdefault("max_range", 50.0)
    return Lidar(
        fov=fov,
        angle_increment=fov,
        angle_offset=angle + fov / 2,
        **kwargs,
    )


def test_full_scan_ray_count_and_angles():
    lidar = full_lidar()
    assert lidar.ray_count == 360
    assert lidar.angles[FORWARD] == pytest.approx(0.0)
    assert lidar.angles[0] == pytest.approx(-math.pi)
    assert lidar.angles[-1] == pytest.approx(math.pi - lidar.angle_increment)


def test_partial_fov_is_centred_on_heading():
    lidar = Lidar(fov=math.pi / 2, angle_increment=math.pi / 180)
    assert lidar.ray_count == 90
    assert lidar.angles[0] == pytest.approx(-math.pi / 4)
    assert lidar.angles[-1] < math.pi / 4


def test_ray_hits_the_east_wall():
    world = World(20, 20)
    scan = full_lidar(max_range=50).scan(world, Pose(5, 5, 0))
    assert scan.ranges[FORWARD] == pytest.approx(15.0)


def test_ray_hits_a_rectangular_obstacle():
    world = World(20, 20, [RectObstacle(10, 0, 11, 20)])
    scan = full_lidar().scan(world, Pose(5, 5, 0))
    assert scan.ranges[FORWARD] == pytest.approx(5.0)


def test_points_at_uses_the_given_pose():
    world = World(20, 20, [RectObstacle(10, 0, 11, 20)])
    scan = full_lidar().scan(world, Pose(5, 5, 0))
    same = scan.points_at(Pose(5, 5, 0))
    assert np.allclose(same, scan.points())
    shifted = scan.points_at(Pose(5.1, 5.0, 0))
    assert np.allclose(shifted - same, [0.1, 0.0], atol=1e-9)


def test_nearest_obstacle_wins():
    world = World(20, 20, [
        RectObstacle(12, 0, 13, 20),
        RectObstacle(8, 0, 9, 20),
    ])
    scan = full_lidar().scan(world, Pose(5, 5, 0))
    assert scan.ranges[FORWARD] == pytest.approx(3.0)


def test_obstacle_behind_the_robot_is_ignored():
    world = World(20, 20, [RectObstacle(1, 0, 2, 20)])
    scan = full_lidar().scan(world, Pose(5, 5, 0))
    assert scan.ranges[FORWARD] == pytest.approx(15.0)


def test_heading_changes_which_wall_is_hit():
    scan = full_lidar().scan(World(20, 20), Pose(5, 5, math.pi / 2))
    assert scan.ranges[FORWARD] == pytest.approx(15.0)  # north wall


def test_left_ray_hits_the_south_wall():
    scan = full_lidar().scan(World(20, 20), Pose(5, 5, 0))
    # index 90 is -90 deg, pointing south
    assert scan.ranges[90] == pytest.approx(5.0)


def test_max_range_caps_distance_and_marks_miss():
    scan = full_lidar(max_range=3.0).scan(World(20, 20), Pose(5, 5, 0))
    assert scan.ranges[FORWARD] == pytest.approx(3.0)
    assert not scan.hit_mask[FORWARD]
    assert scan.hits < scan.ray_count


def test_ray_that_misses_everything_reports_max_range():
    scan = full_lidar(max_range=3.0).scan(World(20, 20), Pose(5, 5, 0))
    assert scan.ranges[FORWARD] == scan.max_range
    assert not scan.hit_mask[FORWARD]


def test_rectangle_beside_the_ray_is_not_hit():
    world = World(20, 20, [RectObstacle(10, 10, 12, 12)])
    scan = full_lidar().scan(world, Pose(5, 5, 0))  # ray at y = 5
    assert scan.ranges[FORWARD] == pytest.approx(15.0)


def test_sensor_mount_offset_moves_the_origin():
    world = World(20, 20)
    scan = single_ray(mount_offset=(0.5, 0.0)).scan(world, Pose(5, 5, 0))
    assert scan.origin == pytest.approx((5.5, 5.0))
    assert scan.ranges[0] == pytest.approx(14.5)


def test_angle_offset_rotates_the_sensor():
    world = World(20, 20)
    lidar = single_ray(angle=math.pi / 2)
    scan = lidar.scan(world, Pose(5, 5, 0))
    assert scan.angles[0] == pytest.approx(math.pi / 2)
    assert scan.ranges[0] == pytest.approx(15.0)  # north wall


def test_scan_points_are_in_world_coordinates():
    world = World(20, 20, [RectObstacle(10, 0, 11, 20)])
    scan = full_lidar().scan(world, Pose(5, 5, 0))
    points = scan.points()
    assert points.shape == (scan.hits, 2)
    assert points.shape[0] > 0
    assert np.all(np.isfinite(points))
    # every reported point is on a surface; check the forward ray's point
    forward = scan.world_angles[FORWARD]
    expected = np.array([5 + scan.ranges[FORWARD] * math.cos(forward),
                         5 + scan.ranges[FORWARD] * math.sin(forward)])
    assert np.min(np.linalg.norm(points - expected, axis=1)) < 1e-9


def test_zero_noise_is_exact():
    world = World(20, 20, [RectObstacle(10, 0, 11, 20)])
    clean = full_lidar().scan(world, Pose(5, 5, 0))
    noisy = full_lidar(range_sigma=0.0, seed=1).scan(world, Pose(5, 5, 0))
    assert np.array_equal(clean.ranges, noisy.ranges)


def test_noise_is_gaussian_with_configured_sigma():
    world = World(20, 20)
    lidar = single_ray(range_sigma=0.05, seed=7)
    samples = np.array([lidar.scan(world, Pose(5, 5, 0)).ranges[0] for _ in range(600)])
    assert samples.mean() == pytest.approx(15.0, abs=0.01)
    assert samples.std() == pytest.approx(0.05, abs=0.01)


def test_seeded_scans_are_reproducible():
    world = World(20, 20, [RectObstacle(10, 0, 11, 20)])
    a = full_lidar(range_sigma=0.05, seed=42).scan(world, Pose(5, 5, 0.3))
    b = full_lidar(range_sigma=0.05, seed=42).scan(world, Pose(5, 5, 0.3))
    assert np.array_equal(a.ranges, b.ranges)


def test_noise_changes_hit_ranges():
    world = World(20, 20, [RectObstacle(10, 0, 11, 20)])
    clean = full_lidar().scan(world, Pose(5, 5, 0))
    noisy = full_lidar(range_sigma=0.02, seed=3).scan(world, Pose(5, 5, 0))
    assert not np.allclose(clean.ranges, noisy.ranges)


def test_noise_never_exceeds_max_range():
    world = World(20, 20)
    scan = full_lidar(max_range=3.0, range_sigma=1.0, seed=0).scan(
        world, Pose(5, 5, 0)
    )
    assert np.all(scan.ranges >= 0)
    assert np.all(scan.ranges <= scan.max_range)


def test_invalid_parameters_are_rejected():
    with pytest.raises(ValueError, match="max_range"):
        Lidar(max_range=0)
    with pytest.raises(ValueError, match="fov"):
        Lidar(fov=0)
    with pytest.raises(ValueError, match="fov"):
        Lidar(fov=2 * math.pi + 0.1)
    with pytest.raises(ValueError, match="angle_increment"):
        Lidar(angle_increment=0)
    with pytest.raises(ValueError, match="range_sigma"):
        Lidar(range_sigma=-0.1)
    with pytest.raises(ValueError, match="mount_offset"):
        Lidar(mount_offset=(1.0, 2.0, 3.0))
