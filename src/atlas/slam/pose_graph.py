"""Pose-graph back-end for 2D SLAM (Phase 11).

A pose graph stores robot poses as nodes and relative-pose measurements as
edges. Consecutive nodes are tied by odometry, which drifts; a loop closure adds
an edge measured against the map when the robot revisits an old place. Because
the loop edge is drift-free, least squares spreads its correction over the whole
chain and the trajectory straightens.

The optimiser is Gauss-Newton on the three coordinates of every free node. The
first node is held fixed to remove the gauge freedom. Jacobians are computed by
finite differences: graphs here hold a few hundred nodes, so the simpler code is
the right trade.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from atlas.geometry import relative_pose, wrap_angle
from atlas.robot.differential_drive import Pose

_EPS = 1e-6


@dataclass(frozen=True)
class GraphEdge:
    """One relative-pose constraint between two nodes."""

    i: int
    j: int
    measurement: tuple[float, float, float]
    weight: float = 1.0
    kind: str = "odometry"


@dataclass
class PoseGraph:
    """Nodes, constraints and a Gauss-Newton solver."""

    poses: list[Pose] = field(default_factory=list)
    edges: list[GraphEdge] = field(default_factory=list)

    def add_node(self, pose: Pose) -> int:
        """Append a pose and return its index."""
        self.poses.append(Pose(pose.x, pose.y, pose.theta))
        return len(self.poses) - 1

    def add_edge(
        self,
        i: int,
        j: int,
        measurement: tuple[float, float, float],
        weight: float = 1.0,
        kind: str = "odometry",
    ) -> GraphEdge:
        """Connect nodes ``i`` and ``j`` with a relative-pose measurement."""
        if not (0 <= i < len(self.poses) and 0 <= j < len(self.poses)):
            raise IndexError("edge endpoints must be existing nodes")
        if i == j:
            raise ValueError("an edge needs two distinct nodes")
        if weight <= 0:
            raise ValueError("weight must be positive")
        if len(measurement) != 3 or not all(
            np.isfinite(value) for value in measurement
        ):
            raise ValueError("measurement must be three finite values")
        edge = GraphEdge(i, j, tuple(measurement), weight, kind)
        self.edges.append(edge)
        return edge

    @staticmethod
    def _error(
        measurement: tuple[float, float, float],
        xi: Pose,
        xj: Pose,
    ) -> np.ndarray:
        predicted = relative_pose((xi.x, xi.y, xi.theta), (xj.x, xj.y, xj.theta))
        return np.array(
            relative_pose(measurement, predicted), dtype=float
        )

    def residual(self, edge: GraphEdge) -> np.ndarray:
        """Three-vector error of one edge at the current poses."""
        return self._error(edge.measurement, self.poses[edge.i], self.poses[edge.j])

    def cost(self) -> float:
        """Weighted sum of squared edge residuals."""
        total = 0.0
        for edge in self.edges:
            error = self.residual(edge)
            total += edge.weight * float(error @ error)
        return total

    def _jacobian(self, edge: GraphEdge) -> np.ndarray:
        """Finite-difference Jacobian of the edge error, shape (3, 6)."""
        xi, xj = self.poses[edge.i], self.poses[edge.j]
        base = self._error(edge.measurement, xi, xj)
        jacobian = np.empty((3, 6))

        for column in range(6):
            plus_i, plus_j = (
                Pose(xi.x, xi.y, xi.theta),
                Pose(xj.x, xj.y, xj.theta),
            )
            target = plus_i if column < 3 else plus_j
            axis = column % 3
            if axis == 0:
                target.x += _EPS
            elif axis == 1:
                target.y += _EPS
            else:
                target.theta = wrap_angle(target.theta + _EPS)
            shifted = self._error(edge.measurement, plus_i, plus_j)
            jacobian[:, column] = (shifted - base) / _EPS
        return jacobian

    def optimize(
        self,
        iterations: int = 20,
        damping: float = 1e-6,
    ) -> list[float]:
        """Run Gauss-Newton and return the cost after each iteration.

        The first element is the cost before any update, so a caller can see
        whether the loop closure actually improved the graph.
        """
        if iterations < 1:
            raise ValueError("iterations must be positive")
        if len(self.poses) < 2:
            return [self.cost()]

        history = [self.cost()]
        free = len(self.poses) - 1

        for _ in range(iterations):
            size = 3 * free
            hessian = np.zeros((size, size))
            gradient = np.zeros(size)

            for edge in self.edges:
                jacobian = self._jacobian(edge)
                error = self.residual(edge)

                columns: list[int] = []
                for node in (edge.i, edge.j):
                    if node == 0:
                        columns.extend([-1, -1, -1])
                    else:
                        base = 3 * (node - 1)
                        columns.extend([base, base + 1, base + 2])

                for a in range(6):
                    if columns[a] < 0:
                        continue
                    gradient[columns[a]] += edge.weight * jacobian[:, a] @ error
                    for b in range(6):
                        if columns[b] < 0:
                            continue
                        hessian[columns[a], columns[b]] += (
                            edge.weight * jacobian[:, a] @ jacobian[:, b]
                        )

            step = np.linalg.solve(
                hessian + damping * np.eye(size), -gradient
            )

            for node in range(1, len(self.poses)):
                base = 3 * (node - 1)
                pose = self.poses[node]
                pose.x += step[base]
                pose.y += step[base + 1]
                pose.theta = wrap_angle(pose.theta + step[base + 2])

            history.append(self.cost())

        return history
