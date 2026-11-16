"""Offline pose-graph SLAM over a recorded run (Phase 11).

The live loop still uses the particle filter for control. This back-end takes
the same run's wheel odometry, scans and estimated poses, builds a pose graph
on the odometry chain, verifies loop closures against the prior map, optimises
the graph, and rebuilds the occupancy map from the corrected poses.

Why odometry nodes: the filter already removes local drift for navigation, so a
graph built from filter poses would have nothing to correct. Odometry is the
drifting input a SLAM back-end is meant to fix, and comparing its error before
and after the closure is the honest test of the graph.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from atlas.geometry import relative_pose
from atlas.localization.likelihood_field import DistanceField
from atlas.mapping.occupancy_map import LogOddsMap
from atlas.planning.grid import OccupancyGrid
from atlas.robot.differential_drive import Pose
from atlas.sensors.lidar import LidarScan
from atlas.slam.loop_closure import LoopCloser
from atlas.slam.pose_graph import PoseGraph
from atlas.slam.scan_matcher import ScanMatcher


@dataclass(frozen=True)
class LoopEvent:
    """An accepted constraint: current node tied to an older one."""

    node: int
    old: int
    score: float


@dataclass
class PoseGraphSlam:
    """Collect a run, close loops, optimise, and rebuild the map."""

    prior: OccupancyGrid
    node_stride: int = 5
    check_interval: int = 5
    loop_radius: float = 2.5
    min_gap: int = 20
    max_score: float = 0.25
    max_candidates: int = 2
    max_loops: int = 10
    loop_weight: float = 5.0

    graph: PoseGraph = field(default_factory=PoseGraph)
    node_odometry: list[Pose] = field(default_factory=list)
    node_estimates: list[Pose] = field(default_factory=list)
    node_truth: list[Pose] = field(default_factory=list)
    node_scans: list[LidarScan] = field(default_factory=list)
    loops: list[LoopEvent] = field(default_factory=list)
    cost_history: list[float] = field(default_factory=list)
    _checked_until: int = field(default=0, init=False, repr=False)

    def __post_init__(self) -> None:
        if self.node_stride < 1:
            raise ValueError("node_stride must be positive")
        if self.check_interval < 1:
            raise ValueError("check_interval must be positive")
        if self.loop_radius <= 0:
            raise ValueError("loop_radius must be positive")
        if self.min_gap < 1:
            raise ValueError("min_gap must be positive")
        if self.max_loops < 0:
            raise ValueError("max_loops must be non-negative")
        self._matcher = ScanMatcher(
            DistanceField.from_grid(self.prior, boundary_only=False),
            xy_window=0.60,
            theta_window=0.20,
            xy_step=0.20,
            theta_step=0.08,
            max_beams=45,
        )
        self._closer = LoopCloser(
            self._matcher,
            radius=self.loop_radius,
            min_gap=self.min_gap,
            max_score=self.max_score,
            max_candidates=self.max_candidates,
        )

    def observe(
        self,
        step: int,
        odometry: Pose,
        estimate: Pose,
        truth: Pose,
        scan: LidarScan,
    ) -> None:
        """Record one control step; every ``node_stride`` steps becomes a node."""
        if step % self.node_stride != 0:
            return
        index = self.graph.add_node(odometry)
        self.node_odometry.append(Pose(odometry.x, odometry.y, odometry.theta))
        self.node_estimates.append(Pose(estimate.x, estimate.y, estimate.theta))
        self.node_truth.append(Pose(truth.x, truth.y, truth.theta))
        self.node_scans.append(scan)

        if index == 0:
            return
        self.graph.add_edge(
            index - 1,
            index,
            relative_pose(
                (self.node_odometry[index - 1].x,
                 self.node_odometry[index - 1].y,
                 self.node_odometry[index - 1].theta),
                (self.node_odometry[index].x,
                 self.node_odometry[index].y,
                 self.node_odometry[index].theta),
            ),
        )

    def close_loops(self) -> None:
        """Verify candidate closures and add them to the graph.

        Only nodes added since the previous call are examined, so calling this
        repeatedly from a live loop does not duplicate edges.
        """
        nodes = len(self.graph.poses)
        for index in range(max(self.min_gap, self._checked_until), nodes):
            if index % self.check_interval != 0:
                continue
            if len(self.loops) >= self.max_loops:
                break
            found = self._closer.detect(
                self.node_scans[index],
                self.node_estimates[index],
                self.node_estimates,
                self.graph.poses,
                index,
            )
            for closure in found:
                self.graph.add_edge(
                    closure.old,
                    index,
                    closure.measurement,
                    weight=self.loop_weight,
                    kind="loop",
                )
                self.loops.append(
                    LoopEvent(index, closure.old, closure.score)
                )
        self._checked_until = nodes

    def solve(self, iterations: int = 20, detect: bool = True) -> None:
        """Optionally close loops, then optimise and record the cost history."""
        if detect:
            self.close_loops()
        self.cost_history = self.graph.optimize(iterations=iterations)

    def map_from(self, poses: list[Pose]) -> LogOddsMap:
        """Build a map by replaying node scans at the given poses."""
        if len(poses) != len(self.node_scans):
            raise ValueError("one pose per node is required")
        mapping = LogOddsMap(self.prior, max_beams=90)
        for scan, pose in zip(self.node_scans, poses):
            mapping.integrate(scan, pose)
        return mapping

    def node_errors(self, poses: list[Pose]) -> list[float]:
        """Euclidean node error against truth."""
        return [
            math.hypot(pose.x - truth.x, pose.y - truth.y)
            for pose, truth in zip(poses, self.node_truth)
        ]

    def corrected(self) -> list[Pose]:
        """Copy the current (optimised) graph poses."""
        return [
            Pose(pose.x, pose.y, pose.theta) for pose in self.graph.poses
        ]
