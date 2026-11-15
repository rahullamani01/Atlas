"""Loop-closure detection for the Phase 11 pose graph.

A closure is proposed when the estimated trajectory comes back near an older
node. The candidate is verified by scan matching the current scan against the
prior map; a low residual means the map, not the drifting odometry, can place
the robot. The accepted edge stores the relative pose between the old node and
the matched pose, so the optimiser can pull the chain back onto the map.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from atlas.geometry import relative_pose
from atlas.robot.differential_drive import Pose
from atlas.sensors.lidar import LidarScan
from atlas.slam.scan_matcher import MatchResult, ScanMatcher


@dataclass(frozen=True)
class LoopClosure:
    """An accepted constraint between node ``old`` and the current node."""

    old: int
    measurement: tuple[float, float, float]
    score: float


class LoopCloser:
    """Propose and verify loop closures from trajectory proximity."""

    def __init__(
        self,
        matcher: ScanMatcher,
        radius: float = 2.5,
        min_gap: int = 20,
        max_score: float = 0.25,
        max_candidates: int = 2,
    ) -> None:
        if radius <= 0:
            raise ValueError("radius must be positive")
        if min_gap < 1:
            raise ValueError("min_gap must be positive")
        if max_score <= 0:
            raise ValueError("max_score must be positive")
        if max_candidates < 1:
            raise ValueError("max_candidates must be positive")
        self.matcher = matcher
        self.radius = radius
        self.min_gap = min_gap
        self.max_score = max_score
        self.max_candidates = max_candidates

    def candidates(self, estimates: list[Pose], index: int) -> list[int]:
        """Older node indices within ``radius`` of the current estimate."""
        if not 0 <= index < len(estimates):
            raise IndexError("index out of range")
        current = estimates[index]
        near: list[tuple[float, int]] = []
        for old in range(0, index - self.min_gap + 1):
            distance = math.hypot(
                current.x - estimates[old].x,
                current.y - estimates[old].y,
            )
            if distance <= self.radius:
                near.append((distance, old))
        near.sort()
        return [old for _, old in near[: self.max_candidates]]

    def detect(
        self,
        scan: LidarScan,
        estimate: Pose,
        estimates: list[Pose],
        nodes: list[Pose],
        index: int,
    ) -> list[LoopClosure]:
        """Return verified closures tying the current node to older ones."""
        candidates = self.candidates(estimates, index)
        if not candidates:
            return []
        result: MatchResult = self.matcher.match(scan, estimate)
        if result.score > self.max_score:
            return []
        found: list[LoopClosure] = []
        for old in candidates:
            measurement = relative_pose(
                (nodes[old].x, nodes[old].y, nodes[old].theta),
                (result.pose.x, result.pose.y, result.pose.theta),
            )
            found.append(LoopClosure(old, measurement, result.score))
        return found
