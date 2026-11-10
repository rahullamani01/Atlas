import math

import pytest

from atlas.control import PID, PathFollower, Twist
from atlas.planning import OccupancyGrid, a_star
from atlas.robot.differential_drive import DifferentialDriveRobot, Pose
from atlas.simulation.simulator import Simulator
from atlas.simulation.world import RectObstacle, World


def straight_path(length=10.0):
    return [(0.0, 0.0), (length, 0.0)]


def test_aligned_robot_drives_straight():
    follower = PathFollower()
    command = follower.compute(Pose(0, 0, 0), straight_path(), 0.05)
    assert command.v == pytest.approx(follower.max_linear_speed)
    assert command.w == pytest.approx(0.0, abs=1e-9)


def test_robot_left_of_path_turns_right():
    follower = PathFollower()
    command = follower.compute(Pose(0, 1, 0), straight_path(), 0.05)
    assert command.w < 0  # clockwise: toward -y


def test_robot_right_of_path_turns_left():
    follower = PathFollower()
    command = follower.compute(Pose(0, -1, 0), straight_path(), 0.05)
    assert command.w > 0


def test_carrot_behind_robot_causes_rotation_in_place():
    follower = PathFollower()
    command = follower.compute(Pose(0, 0, math.pi), straight_path(), 0.05)
    assert command.v == 0.0
    assert command.w < 0


def test_goal_within_tolerance_stops_and_reports():
    follower = PathFollower(goal_tolerance=0.1)
    command = follower.compute(Pose(10.0, 0.0, 0.0), straight_path(), 0.05)
    assert command == Twist(0.0, 0.0)
    assert follower.goal_reached


def test_goal_flag_clears_when_moving_away():
    follower = PathFollower(goal_tolerance=0.1)
    follower.compute(Pose(10.0, 0.0, 0.0), straight_path(), 0.05)
    assert follower.goal_reached
    follower.compute(Pose(5.0, 0.0, 0.0), straight_path(), 0.05)
    assert not follower.goal_reached


def test_speed_and_angular_limits_are_respected():
    follower = PathFollower(max_linear_speed=0.2, max_angular_speed=0.7)
    command = follower.compute(Pose(0, 1, 0), straight_path(), 0.05)
    assert 0 <= command.v <= 0.2
    assert abs(command.w) <= 0.7


def test_goal_tolerance_validation():
    with pytest.raises(ValueError, match="goal_tolerance"):
        PathFollower(goal_tolerance=0)


def test_invalid_parameters_are_rejected():
    with pytest.raises(ValueError, match="lookahead"):
        PathFollower(lookahead=0)
    with pytest.raises(ValueError, match="max_linear_speed"):
        PathFollower(max_linear_speed=0)
    with pytest.raises(ValueError, match="max_angular_speed"):
        PathFollower(max_angular_speed=0)
    with pytest.raises(ValueError, match="distance_gain"):
        PathFollower(distance_gain=0)
    with pytest.raises(ValueError, match="min_speed"):
        PathFollower(min_speed=-0.1)
    with pytest.raises(ValueError, match="min_speed"):
        PathFollower(min_speed=1.0, max_linear_speed=0.5)


def test_empty_path_is_rejected():
    with pytest.raises(ValueError, match="path"):
        PathFollower().compute(Pose(), [], 0.05)


def test_non_positive_dt_is_rejected():
    with pytest.raises(ValueError, match="dt"):
        PathFollower().compute(Pose(), straight_path(), 0.0)


def test_lookahead_interpolates_along_the_path():
    follower = PathFollower(lookahead=1.5)
    path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
    assert follower.lookahead_point(Pose(0, 0, 0), path) == pytest.approx((1.5, 0.0))


def test_lookahead_starts_from_nearest_waypoint():
    follower = PathFollower(lookahead=0.5)
    path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
    assert follower.lookahead_point(Pose(1.9, 0, 0), path) == pytest.approx((2.5, 0.0))


def test_lookahead_clamps_to_the_goal():
    follower = PathFollower(lookahead=100.0)
    path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    assert follower.lookahead_point(Pose(0, 0, 0), path) == (2.0, 0.0)


def test_duplicate_waypoints_are_skipped():
    follower = PathFollower(lookahead=0.5)
    path = [(0.0, 0.0), (0.0, 0.0), (1.0, 0.0)]
    assert follower.lookahead_point(Pose(0, 0, 0), path) == (0.5, 0.0)


def test_reset_clears_controller_state():
    follower = PathFollower()
    follower.compute(Pose(10.0, 0.0, 0.0), straight_path(), 0.05)
    follower.reset()
    assert not follower.goal_reached
    assert follower.heading_pid.integral == 0


# ---- closed-loop integration with the Phase 2 map ----

def demo_world():
    return World(20, 20, [
        RectObstacle(6, 0, 7, 14),
        RectObstacle(10, 9, 17, 10),
        RectObstacle(13, 2, 15, 4),
    ])


def segment_distance(point, a, b):
    px, py = point
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    length_squared = dx * dx + dy * dy
    t = 0.0 if length_squared == 0 else max(
        0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_squared)
    )
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def plan_demo_path(inflate=0.25):
    grid = OccupancyGrid.from_world(demo_world(), resolution=0.25, inflate_radius=inflate)
    result = a_star(grid, grid.world_to_cell(1, 1), grid.world_to_cell(18, 18))
    assert result.found
    return grid, [grid.cell_to_world(cell) for cell in result.path]


def drive(path, heading_pid, bias=0.0, max_steps=6000):
    start = path[0]
    look = min(5, len(path) - 1)
    heading = math.atan2(path[look][1] - start[1], path[look][0] - start[0])
    world = demo_world()
    robot = DifferentialDriveRobot(pose=Pose(start[0], start[1], heading))
    follower = PathFollower(heading_pid=heading_pid)
    sim = Simulator(world, robot, dt=0.05, method="midpoint")

    cross_track = []
    for _ in range(max_steps):
        command = follower.compute(robot.pose, path, sim.dt)
        ok = sim.step(command.v, command.w + bias)
        cross_track.append(
            min(
                segment_distance((robot.pose.x, robot.pose.y), a, b)
                for a, b in zip(path, path[1:])
            )
        )
        if not ok or follower.goal_reached:
            break

    return sim, robot, follower, cross_track


def test_closed_loop_reaches_goal_without_collision():
    _, path = plan_demo_path()
    sim, robot, follower, cross_track = drive(path, PID(kp=2.0, ki=0.2, kd=0.05))

    assert sim.collision_count == 0
    assert follower.goal_reached
    final_error = math.hypot(robot.pose.x - path[-1][0], robot.pose.y - path[-1][1])
    assert final_error < 0.15
    rms = math.sqrt(sum(e * e for e in cross_track) / len(cross_track))
    assert rms < 0.1


def test_pid_rejects_wheel_bias_better_than_p_only():
    _, path = plan_demo_path()
    bias = 0.2  # rad/s of unmodelled yaw disturbance

    _, _, _, p_track = drive(path, PID(kp=2.0), bias=bias)
    _, _, _, pid_track = drive(path, PID(kp=2.0, ki=0.2, kd=0.05), bias=bias)

    p_rms = math.sqrt(sum(e * e for e in p_track) / len(p_track))
    pid_rms = math.sqrt(sum(e * e for e in pid_track) / len(pid_track))
    assert pid_rms < 0.9 * p_rms
