import math

import numpy as np
import pytest

from atlas.control import Twist
from atlas.perception import DynamicObstacleMap
from atlas.recovery import (
    NavState,
    NavigationSupervisor,
    nearest_free_cell,
)
from atlas.planning import OccupancyGrid, a_star
from atlas.robot.differential_drive import DifferentialDriveRobot, Pose
from atlas.sensors import Lidar
from atlas.simulation.simulator import Simulator
from atlas.simulation.world import RectObstacle, World

DT = 0.05


def demo_world():
    return World(20, 20, [
        RectObstacle(6, 0, 7, 14),
        RectObstacle(10, 9, 17, 10),
        RectObstacle(13, 2, 15, 4),
    ])


def demo_grid(inflate=0.30):
    return OccupancyGrid.from_world(demo_world(), resolution=0.25,
                                    inflate_radius=inflate)


def planned_setup():
    grid = demo_grid()
    goal = grid.world_to_cell(18.125, 18.125)
    result = a_star(grid, grid.world_to_cell(1.125, 1.125), goal)
    assert result.found
    path = [grid.cell_to_world(c) for c in result.path]
    heading = math.atan2(path[5][1] - path[0][1], path[5][0] - path[0][0])
    robot = DifferentialDriveRobot(pose=Pose(path[0][0], path[0][1], heading))
    return grid, goal, robot


# ---- nearest free cell ----

def test_nearest_free_cell_returns_the_same_cell():
    grid = OccupancyGrid(10, 10)
    assert nearest_free_cell(grid, (3, 4)) == (3, 4)


def test_nearest_free_cell_finds_a_neighbour():
    grid = OccupancyGrid(10, 10)
    for dc in (-1, 0, 1):
        for dr in (-1, 0, 1):
            grid.set_blocked((5 + dc, 5 + dr))
    cell = nearest_free_cell(grid, (5, 5))
    assert cell is not None
    assert grid.is_free(cell)
    assert max(abs(cell[0] - 5), abs(cell[1] - 5)) == 2


def test_nearest_free_cell_returns_none_when_boxed_in():
    blocked = np.ones((5, 5), dtype=bool)
    blocked[0, 0] = False
    grid = OccupancyGrid(5, 5, blocked=blocked)
    assert nearest_free_cell(grid, (2, 2), max_radius=1) is None
    assert nearest_free_cell(grid, (2, 2), max_radius=3) == (0, 0)


# ---- planning ----

def test_plan_sets_a_path_and_counts():
    grid, goal, robot = planned_setup()
    supervisor = NavigationSupervisor(grid, goal)
    assert supervisor.plan(robot.pose)
    assert supervisor.path
    assert supervisor.plan_count == 1
    assert supervisor.replan_count == 0
    assert supervisor.state is NavState.DRIVING


def test_goal_cell_must_be_free():
    grid = demo_grid()
    blocked = grid.world_to_cell(6.5, 7.0)
    assert not grid.is_free(blocked)
    with pytest.raises(ValueError, match="goal_cell"):
        NavigationSupervisor(grid, blocked)


# ---- closed-loop recovery ----

def test_detection_replans_and_reaches_the_goal():
    grid, goal, robot = planned_setup()
    world = demo_world()
    sim = Simulator(world, robot, dt=DT, method="midpoint")
    lidar = Lidar(max_range=10.0, range_sigma=0.02, seed=3)
    detector = DynamicObstacleMap(grid, clearance=0.30)
    supervisor = NavigationSupervisor(grid, goal, detector=detector)
    assert supervisor.plan(robot.pose)

    for step in range(3000):
        if step == 80:
            world.obstacles.append(RectObstacle(2.32, 5.7, 2.92, 6.3))
        command = supervisor.step(robot.pose, lidar.scan(world, robot.pose), DT)
        ok = sim.step(command.v, command.w)
        assert ok, f"collision at step {step}"
        if supervisor.goal_reached or supervisor.failed:
            break

    assert supervisor.goal_reached
    assert not supervisor.failed
    assert supervisor.replan_count >= 1
    assert supervisor.detected_cells
    assert math.hypot(robot.pose.x - 18.125, robot.pose.y - 18.125) < 0.2


def test_no_route_scans_backs_up_and_stops_safely():
    grid, goal, robot = planned_setup()
    # A static wall the supervisor knows about, splitting the map.
    grid.blocked[:, 48:52] = True
    supervisor = NavigationSupervisor(
        grid, goal, scan_steps=3, backup_steps=2,
        scan_speed=1.0, backup_speed=-0.1,
    )
    assert not supervisor.plan(robot.pose)

    world = demo_world()
    lidar = Lidar(max_range=10.0, range_sigma=0.0)
    commands = []
    for _ in range(1 + supervisor.scan_steps + supervisor.backup_steps + 2):
        commands.append(supervisor.step(
            robot.pose, lidar.scan(world, robot.pose), DT
        ))

    assert supervisor.failed
    assert supervisor.recovering or supervisor.failed
    kinds = [event.kind for event in supervisor.events]
    assert "no_path" in kinds
    assert "scanning" in kinds
    assert "backing_up" in kinds
    assert "failed" in kinds
    assert any(command.v < 0 for command in commands)
    assert commands[-1] == Twist(0.0, 0.0)


def test_stall_triggers_recovery():
    grid, goal, robot = planned_setup()
    supervisor = NavigationSupervisor(grid, goal, stall_window=5,
                                     stall_distance=0.1, scan_steps=3)
    assert supervisor.plan(robot.pose)

    world = demo_world()
    lidar = Lidar(max_range=10.0, range_sigma=0.0)

    # Feed the same pose every step: the robot makes no progress.
    for _ in range(7):
        supervisor.step(robot.pose, lidar.scan(world, robot.pose), DT)

    assert supervisor.state is NavState.SCANNING
    assert any(event.kind == "scanning" for event in supervisor.events)


# ---- validation ----

def test_non_positive_dt_is_rejected():
    grid, goal, robot = planned_setup()
    supervisor = NavigationSupervisor(grid, goal)
    supervisor.plan(robot.pose)
    with pytest.raises(ValueError, match="dt"):
        supervisor.step(robot.pose, Lidar().scan(demo_world(), robot.pose), 0.0)


def test_invalid_parameters_are_rejected():
    grid = demo_grid()
    goal = grid.world_to_cell(18.125, 18.125)
    with pytest.raises(ValueError, match="scan_steps"):
        NavigationSupervisor(grid, goal, scan_steps=0)
    with pytest.raises(ValueError, match="scan_speed"):
        NavigationSupervisor(grid, goal, scan_speed=0)
    with pytest.raises(ValueError, match="backup_steps"):
        NavigationSupervisor(grid, goal, backup_steps=0)
    with pytest.raises(ValueError, match="backup_speed"):
        NavigationSupervisor(grid, goal, backup_speed=0.1)
    with pytest.raises(ValueError, match="stall_window"):
        NavigationSupervisor(grid, goal, stall_window=0)
    with pytest.raises(ValueError, match="stall_distance"):
        NavigationSupervisor(grid, goal, stall_distance=-1)
