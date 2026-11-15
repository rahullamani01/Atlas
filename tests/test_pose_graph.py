"""Geometry and least-squares tests for the Phase 11 pose graph."""
import math

import pytest

from atlas.geometry import compose_pose, relative_pose, wrap_angle
from atlas.robot.differential_drive import Pose
from atlas.slam.pose_graph import PoseGraph


def _square(count: int = 41) -> list[tuple[float, float, float]]:
    """Counter-clockwise square loop with a heading that follows the edges."""
    corners = [
        (0.0, 0.0, 0.0),
        (4.0, 0.0, math.pi / 2),
        (4.0, 4.0, math.pi),
        (0.0, 4.0, -math.pi / 2),
        (0.0, 0.0, 0.0),
    ]
    poses = []
    for index in range(count):
        t = index / (count - 1) * 4.0
        side = min(int(t), 3)
        fraction = t - side
        x0, y0, h0 = corners[side]
        x1, y1, h1 = corners[side + 1]
        turn = (h1 - h0 + math.pi) % (2.0 * math.pi) - math.pi
        poses.append(
            (x0 + (x1 - x0) * fraction, y0 + (y1 - y0) * fraction,
             h0 + turn * fraction)
        )
    return poses


def test_relative_and_compose_are_inverse():
    a = (1.0, 2.0, 0.4)
    b = (3.0, -1.0, 1.1)

    moved = compose_pose(a, relative_pose(a, b))

    assert moved == pytest.approx(b)


def test_consistent_chain_stays_put():
    truth = _square()
    graph = PoseGraph()
    for x, y, theta in truth:
        graph.add_node(Pose(x, y, theta))
    for index in range(1, len(truth)):
        graph.add_edge(index - 1, index, relative_pose(truth[index - 1], truth[index]))

    before = [Pose(p.x, p.y, p.theta) for p in graph.poses]
    graph.optimize(iterations=5)

    for start, end in zip(before, graph.poses):
        assert math.hypot(start.x - end.x, start.y - end.y) < 1e-9
        assert abs(wrap_angle(start.theta - end.theta)) < 1e-9


def test_loop_closure_removes_synthetic_drift():
    truth = _square()
    graph = PoseGraph()
    for index, (x, y, theta) in enumerate(truth):
        graph.add_node(Pose(x + 0.01 * index, y + 0.005 * index, theta * 1.01))
    for index in range(1, len(truth)):
        graph.add_edge(index - 1, index, relative_pose(truth[index - 1], truth[index]))

    def ate() -> float:
        errors = [
            math.hypot(pose.x - true[0], pose.y - true[1])
            for pose, true in zip(graph.poses, truth)
        ]
        return math.sqrt(sum(error * error for error in errors) / len(errors))

    before = ate()
    graph.add_edge(
        0, len(truth) - 1, relative_pose(truth[0], truth[-1]), weight=5.0,
        kind="loop",
    )
    history = graph.optimize(iterations=20)

    assert before > 0.2
    assert ate() < 0.01
    assert history[-1] < history[0]


def test_invalid_graph_input_is_rejected():
    graph = PoseGraph()
    graph.add_node(Pose())

    with pytest.raises(IndexError):
        graph.add_edge(0, 1, (0.0, 0.0, 0.0))
    with pytest.raises(ValueError):
        graph.add_edge(0, 0, (0.0, 0.0, 0.0))
    with pytest.raises(ValueError):
        graph.add_edge(0, 0, (0.0, 0.0, 0.0), weight=0.0)
    with pytest.raises(ValueError):
        graph.add_edge(0, 0, (0.0, float("nan"), 0.0))
    with pytest.raises(ValueError):
        graph.optimize(iterations=0)
