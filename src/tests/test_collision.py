import pytest

from atlas.robot.differential_drive import DifferentialDriveRobot, Pose
from atlas.simulation.simulator import Simulator
from atlas.simulation.world import RectObstacle, World


def test_circle_rect_far_apart():
    assert not RectObstacle(4, 4, 6, 6).collides_with_circle(1, 1, 0.5)


def test_circle_rect_overlap():
    assert RectObstacle(4, 4, 6, 6).collides_with_circle(3.8, 5, 0.5)


def test_circle_near_corner_does_not_hit():
    # bounding boxes overlap, but the corner is farther than the radius
    assert not RectObstacle(4, 4, 6, 6).collides_with_circle(3.6, 3.6, 0.5)


def test_world_boundaries():
    w = World(10, 10)
    assert w.in_collision(0.1, 5, 0.2)
    assert w.in_collision(9.9, 5, 0.2)
    assert not w.in_collision(5, 5, 0.2)


def test_simulator_stops_at_obstacle():
    world = World(10, 10, [RectObstacle(5, 0, 6, 10)])
    robot = DifferentialDriveRobot(pose=Pose(2, 5, 0))
    sim = Simulator(world, robot, dt=0.05, method="exact")
    for _ in range(100):
        sim.step(1.0, 0.0)
    assert sim.collided
    assert robot.pose.x <= 5 - robot.body_radius + 1e-9


def test_trajectory_is_recorded():
    sim = Simulator(World(10, 10), DifferentialDriveRobot(pose=Pose(5, 5, 0)), dt=0.1)
    for _ in range(20):
        sim.step(0.5, 0.0)
    traj = sim.trajectory()
    assert traj.shape == (21, 4)
    assert traj[-1, 0] == pytest.approx(2.0)