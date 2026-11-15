"""Correlative scan matching against a distance field (Phase 11).

Given a scan and a starting pose, the matcher scores a grid of nearby poses by
the mean distance from projected hit points to the nearest mapped obstacle. It
searches coarsely, then refines around the best cell. The score is the same
quantity the particle filter uses for its measurement likelihood, so a good
match means the scan agrees with the map.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from atlas.localization.likelihood_field import DistanceField
from atlas.robot.differential_drive import Pose
from atlas.sensors.lidar import LidarScan
from atlas.geometry import wrap_angle


@dataclass(frozen=True)
class MatchResult:
    """Best pose found and its mean clipped endpoint distance (lower is better)."""

    pose: Pose
    score: float
    evaluations: int


class ScanMatcher:
    """Brute-force pose search scored by endpoint agreement with the map."""

    def __init__(
        self,
        field: DistanceField,
        xy_window: float = 1.25,
        theta_window: float = 0.35,
        xy_step: float = 0.25,
        theta_step: float = 0.10,
        max_beams: int = 60,
        max_distance: float = 1.0,
    ) -> None:
        if xy_window < 0 or theta_window < 0:
            raise ValueError("search windows must be non-negative")
        if xy_step <= 0 or theta_step <= 0:
            raise ValueError("search steps must be positive")
        if max_beams < 1:
            raise ValueError("max_beams must be positive")
        if max_distance <= 0:
            raise ValueError("max_distance must be positive")
        self.field = field
        self.xy_window = xy_window
        self.theta_window = theta_window
        self.xy_step = xy_step
        self.theta_step = theta_step
        self.max_beams = max_beams
        self.max_distance = max_distance

    def score(self, scan: LidarScan, pose: Pose) -> float:
        """Mean clipped distance from projected hits to mapped obstacles."""
        points = scan.points_at(pose)
        if len(points) == 0:
            return self.max_distance
        stride = max(1, math.ceil(len(points) / self.max_beams))
        distances = self.field.sample(points[::stride])
        return float(np.clip(distances, 0.0, self.max_distance).mean())

    def _search(
        self,
        scan: LidarScan,
        center: Pose,
        xy_window: float,
        theta_window: float,
        xy_step: float,
        theta_step: float,
    ) -> MatchResult:
        nx = int(round(xy_window / xy_step))
        nt = int(round(theta_window / theta_step))
        best = None
        best_score = math.inf
        evaluations = 0

        for dx in np.arange(-nx, nx + 1) * xy_step:
            for dy in np.arange(-nx, nx + 1) * xy_step:
                for dtheta in np.arange(-nt, nt + 1) * theta_step:
                    candidate = Pose(
                        center.x + float(dx),
                        center.y + float(dy),
                        wrap_angle(center.theta + float(dtheta)),
                    )
                    value = self.score(scan, candidate)
                    evaluations += 1
                    if value < best_score:
                        best_score = value
                        best = candidate
        assert best is not None
        return MatchResult(best, best_score, evaluations)

    def match(self, scan: LidarScan, prior: Pose) -> MatchResult:
        """Coarse search around ``prior``, then a fine search around the winner."""
        coarse = self._search(
            scan,
            prior,
            self.xy_window,
            self.theta_window,
            self.xy_step,
            self.theta_step,
        )
        fine = self._search(
            scan,
            coarse.pose,
            self.xy_step,
            self.theta_step,
            self.xy_step / 5.0,
            self.theta_step / 5.0,
        )
        if fine.score > coarse.score:
            return MatchResult(
                coarse.pose,
                coarse.score,
                coarse.evaluations + fine.evaluations,
            )
        return MatchResult(
            fine.pose,
            fine.score,
            coarse.evaluations + fine.evaluations,
        )
