import math

import numpy as np
import pytest

from atlas.geometry import (
    angle_difference,
    deg_to_rad,
    euclidean_distance,
    rad_to_deg,
    robot_to_world,
    rotation_matrix,
    world_to_robot,
    wrap_angle,
)


def test_distance_3_4_5():
    assert euclidean_distance((0, 0), (3, 4)) == pytest.approx(5.0)


def test_distance_is_symmetric_and_zero_for_same_point():
    assert euclidean_distance((1, 2), (4, 6)) == pytest.approx(euclidean_distance((4, 6), (1, 2)))
    assert euclidean_distance((2, 2), (2, 2)) == 0.0


def test_degree_radian_roundtrip():
    assert deg_to_rad(180) == pytest.approx(math.pi)
    assert rad_to_deg(math.pi / 2) == pytest.approx(90)
    assert rad_to_deg(deg_to_rad(37.5)) == pytest.approx(37.5)


@pytest.mark.parametrize(
    "angle, expected",
    [
        (0.0, 0.0),
        (math.pi / 2, math.pi / 2),
        (3 * math.pi / 2, -math.pi / 2),
        (-3 * math.pi / 2, math.pi / 2),
        (2 * math.pi, 0.0),
        (5 * math.pi, -math.pi),
    ],
)
def test_wrap_angle(angle, expected):
    assert wrap_angle(angle) == pytest.approx(expected, abs=1e-9)


def test_wrap_angle_always_in_range():
    for a in np.linspace(-20, 20, 401):
        w = wrap_angle(a)
        assert -math.pi <= w < math.pi + 1e-12


def test_angle_difference_takes_shortest_way():
    # from 170 deg to -170 deg is +20 deg, not -340 deg
    d = angle_difference(deg_to_rad(-170), deg_to_rad(170))
    assert d == pytest.approx(deg_to_rad(20))


def test_rotation_matrix_90_degrees():
    r = rotation_matrix(math.pi / 2)
    assert r @ np.array([1.0, 0.0]) == pytest.approx(np.array([0.0, 1.0]))


def test_rotation_matrix_is_orthogonal():
    r = rotation_matrix(0.7)
    assert r.T @ r == pytest.approx(np.eye(2))
    assert np.linalg.det(r) == pytest.approx(1.0)


def test_robot_to_world_example():
    # robot at (2, 1) facing +y; a point 1 m in front of it is at (2, 2)
    pose = (2.0, 1.0, math.pi / 2)
    assert robot_to_world((1.0, 0.0), pose) == pytest.approx(np.array([2.0, 2.0]))


def test_world_robot_roundtrip():
    pose = (3.0, -2.0, 0.9)
    p = np.array([1.5, 4.0])
    assert robot_to_world(world_to_robot(p, pose), pose) == pytest.approx(p)
