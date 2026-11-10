import math

import pytest

from atlas.robot.differential_drive import DifferentialDriveRobot, Pose
from atlas.robot.odometry import Odometry


def test_straight_line_accumulates_exactly():
    odom = Odometry()
    odom.reset(Pose(1.0, 2.0, math.pi / 2))
    for _ in range(20):
        odom.update_twist(0.5, 0.0, 0.05)
    assert odom.pose.x == pytest.approx(1.0, abs=1e-9)
    assert odom.pose.y == pytest.approx(2.5, abs=1e-9)
    assert odom.pose.theta == pytest.approx(math.pi / 2)


def test_arc_matches_exact_unicycle_integration():
    robot = DifferentialDriveRobot(pose=Pose(1.0, 2.0, 0.5))
    odom = Odometry()
    odom.reset(Pose(1.0, 2.0, 0.5))
    for _ in range(50):
        robot.step(0.4, 0.6, 0.05, method="exact")
        odom.update_twist(0.4, 0.6, 0.05)
    assert odom.pose.x == pytest.approx(robot.pose.x)
    assert odom.pose.y == pytest.approx(robot.pose.y)
    assert odom.pose.theta == pytest.approx(robot.pose.theta)


def test_pure_rotation_keeps_position():
    odom = Odometry()
    odom.reset(Pose(3.0, 4.0, 0.0))
    for _ in range(10):
        odom.update_twist(0.0, 1.0, 0.05)
    assert odom.pose.x == pytest.approx(3.0)
    assert odom.pose.y == pytest.approx(4.0)
    assert odom.pose.theta == pytest.approx(0.5)


def test_wheel_speeds_round_trip_with_unit_scales():
    odom = Odometry()
    robot = DifferentialDriveRobot()
    v, w = robot.wheel_speeds_to_twist(8.0, 12.0)
    assert odom.wheel_speeds_to_twist(8.0, 12.0) == pytest.approx((v, w))


def test_right_wheel_scale_bias_adds_counter_clockwise_yaw():
    odom = Odometry(right_scale=1.1)
    v, w = odom.wheel_speeds_to_twist(10.0, 10.0)
    assert v > 0
    assert w > 0  # right wheel over-reports, so the estimate turns left


def test_left_wheel_scale_bias_adds_clockwise_yaw():
    odom = Odometry(left_scale=1.1)
    _, w = odom.wheel_speeds_to_twist(10.0, 10.0)
    assert w < 0


def test_update_wheels_matches_update_twist():
    a = Odometry(left_scale=0.98, right_scale=1.02)
    b = Odometry(left_scale=0.98, right_scale=1.02)
    a.update_wheels(9.0, 11.0, 0.05)
    v, w = b.wheel_speeds_to_twist(9.0, 11.0)
    b.update_twist(v, w, 0.05)
    assert a.pose.x == pytest.approx(b.pose.x)
    assert a.pose.y == pytest.approx(b.pose.y)
    assert a.pose.theta == pytest.approx(b.pose.theta)


def test_heading_is_wrapped():
    odom = Odometry()
    odom.reset(Pose(0.0, 0.0, 3.0))
    odom.update_twist(0.0, 1.0, 1.0)
    assert -math.pi <= odom.pose.theta < math.pi


def test_reset_sets_the_pose():
    odom = Odometry()
    odom.update_twist(1.0, 0.0, 0.1)
    odom.reset(Pose(5.0, 6.0, 0.2))
    assert odom.pose == Pose(5.0, 6.0, 0.2)
    odom.reset()
    assert odom.pose == Pose()


def test_invalid_parameters_are_rejected():
    with pytest.raises(ValueError, match="wheel_radius"):
        Odometry(wheel_radius=0)
    with pytest.raises(ValueError, match="wheel_separation"):
        Odometry(wheel_separation=-1)
    with pytest.raises(ValueError, match="wheel scales"):
        Odometry(left_scale=0)
    with pytest.raises(ValueError, match="wheel scales"):
        Odometry(right_scale=-0.5)


def test_non_positive_dt_is_rejected():
    with pytest.raises(ValueError, match="dt"):
        Odometry().update_twist(1.0, 0.0, 0.0)
