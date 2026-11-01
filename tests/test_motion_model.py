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


# ---- forward kinematics from wheel velocities ----

def test_equal_wheel_speeds_drive_straight():
    r = DifferentialDriveRobot(pose=Pose(0, 0, 0))
    # v = r * omega = 0.05 * 10 = 0.5 m/s for 2 s -> 1 m along +x
    for _ in range(100):
        r.step_wheels(10.0, 10.0, 0.02, "exact")
    assert r.pose.x == pytest.approx(1.0)
    assert r.pose.y == pytest.approx(0.0, abs=1e-9)
    assert r.pose.theta == pytest.approx(0.0, abs=1e-9)


def test_opposite_wheel_speeds_spin_in_place():
    r = DifferentialDriveRobot(pose=Pose(4, 4, 0))
    for _ in range(100):
        r.step_wheels(-5.0, 5.0, 0.01, "exact")
    assert (r.pose.x, r.pose.y) == (4, 4)
    # omega = r(wr - wl) / L = 0.05 * 10 / 0.30
    assert r.pose.theta == pytest.approx(0.05 * 10 / 0.30)


def test_wheel_step_matches_equivalent_twist_step():
    a = DifferentialDriveRobot(pose=Pose(1, 2, 0.3))
    b = DifferentialDriveRobot(pose=Pose(1, 2, 0.3))
    v, w = a.wheel_speeds_to_twist(4.0, 7.0)
    for _ in range(50):
        a.step_wheels(4.0, 7.0, 0.02, "midpoint")
        b.step(v, w, 0.02, "midpoint")
    assert (a.pose.x, a.pose.y, a.pose.theta) == pytest.approx(
        (b.pose.x, b.pose.y, b.pose.theta)
    )


def test_inverse_kinematics_of_pure_forward_motion():
    r = DifferentialDriveRobot(wheel_radius=0.1, wheel_separation=0.5)
    omega_l, omega_r = r.twist_to_wheel_speeds(0.3, 0.0)
    assert omega_l == pytest.approx(3.0)
    assert omega_r == pytest.approx(3.0)


def test_positive_angular_velocity_turns_left():
    r = DifferentialDriveRobot(pose=Pose(0, 0, 0))
    r.step_wheels(5.0, 8.0, 0.1, "exact")  # right wheel faster -> CCW
    assert r.pose.theta > 0
    assert r.pose.y > 0


def test_simulator_can_be_driven_by_wheel_speeds():
    from atlas.simulation.simulator import Simulator
    from atlas.simulation.world import World

    robot = DifferentialDriveRobot(pose=Pose(1, 1, 0))
    sim = Simulator(World(10, 10), robot, dt=0.1, method="exact")
    for _ in range(10):
        assert sim.step_wheels(10.0, 10.0)
    # v = 0.05 * 10 = 0.5 m/s for 1 s
    assert robot.pose.x == pytest.approx(1.5)
    assert sim.trajectory()[-1, 0] == pytest.approx(1.0)


# ---- parameter validation ----

def test_zero_wheel_radius_is_rejected():
    with pytest.raises(ValueError, match="wheel_radius"):
        DifferentialDriveRobot(wheel_radius=0.0)


def test_zero_wheel_separation_is_rejected():
    with pytest.raises(ValueError, match="wheel_separation"):
        DifferentialDriveRobot(wheel_separation=0.0)


def test_negative_body_radius_is_rejected():
    with pytest.raises(ValueError, match="body_radius"):
        DifferentialDriveRobot(body_radius=-0.1)
