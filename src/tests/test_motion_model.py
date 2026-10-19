import math

import pytest

from atlas.robot.differential_drive import DifferentialDriveRobot, Pose


def run(robot, v, w, total_time, steps, method):
    dt = total_time / steps
    for _ in range(steps):
        robot.step(v, w, dt, method)


def test_straight_line():
    r = DifferentialDriveRobot(pose=Pose(0, 0, 0))
    run(r, 1.0, 0.0, 1.0, 10, "euler")
    assert r.pose.x == pytest.approx(1.0)
    assert r.pose.y == pytest.approx(0.0, abs=1e-12)


def test_heading_decides_direction():
    r = DifferentialDriveRobot(pose=Pose(0, 0, math.pi / 2))
    run(r, 1.0, 0.0, 1.0, 10, "euler")
    assert r.pose.x == pytest.approx(0.0, abs=1e-9)
    assert r.pose.y == pytest.approx(1.0)


def test_rotate_in_place():
    r = DifferentialDriveRobot(pose=Pose(2, 3, 0))
    run(r, 0.0, 1.0, 1.0, 100, "midpoint")
    assert (r.pose.x, r.pose.y) == (2, 3)
    assert r.pose.theta == pytest.approx(1.0)


def test_exact_circle_closes():
    r = DifferentialDriveRobot(pose=Pose(5, 3, 0))
    run(r, 0.5, 0.5, 4 * math.pi, 100, "exact")
    assert r.pose.x == pytest.approx(5, abs=1e-9)
    assert r.pose.y == pytest.approx(3, abs=1e-9)


def test_euler_error_is_larger_than_midpoint_error():
    def max_error_vs_exact(method):
        total, steps = 4 * math.pi, 100
        dt = total / steps
        a = DifferentialDriveRobot(pose=Pose(5, 3, 0))
        b = DifferentialDriveRobot(pose=Pose(5, 3, 0))
        worst = 0.0
        for _ in range(steps):
            a.step(0.5, 0.5, dt, method)
            b.step(0.5, 0.5, dt, "exact")
            worst = max(worst, math.hypot(a.pose.x - b.pose.x, a.pose.y - b.pose.y))
        return worst

    euler = max_error_vs_exact("euler")
    midpoint = max_error_vs_exact("midpoint")
    assert euler > 1e-2
    assert midpoint < euler / 10


def test_wheel_speed_roundtrip():
    r = DifferentialDriveRobot()
    wl, wr = r.twist_to_wheel_speeds(0.4, 0.8)
    v, w = r.wheel_speeds_to_twist(wl, wr)
    assert (v, w) == (pytest.approx(0.4), pytest.approx(0.8))


def test_opposite_wheels_means_pure_rotation():
    r = DifferentialDriveRobot()
    v, w = r.wheel_speeds_to_twist(-5.0, 5.0)
    assert v == pytest.approx(0.0)
    assert w > 0


def test_velocity_is_clipped():
    r = DifferentialDriveRobot(pose=Pose(0, 0, 0))
    r.step(10.0, 0.0, 1.0, "euler")
    assert r.pose.x == pytest.approx(r.max_linear_speed)


def test_unknown_method_raises():
    with pytest.raises(ValueError):
        DifferentialDriveRobot().step(1, 0, 0.1, "magic")