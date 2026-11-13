"""Global relocalization after a kidnapped-robot event (Phase 9).

A particle filter started near the true pose tracks well, but it cannot recover
from an arbitrary displacement: every particle is in the wrong place and the
measurement update collapses onto whichever wrong hypothesis fits best. This
module supplies the missing behaviour:

1. ``LocalizationMonitor`` watches the map-consistency residual and reports a
   sustained failure, so a single odd scan does not trigger a search.
2. ``Relocalizer`` performs a coarse-to-fine scan match over the known map. A
   single scan can be ambiguous, so it keeps several pose hypotheses and
   accumulates their scores while the robot rotates in place. The best
   hypothesis then seeds the particle filter.

Everything here uses the known static map. The live dynamic map is deliberately
excluded: a global search should not depend on obstacles discovered from an
estimate that is known to be wrong.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from atlas.geometry import wrap_angle
from atlas.localization.likelihood_field import DistanceField
from atlas.planning.grid import OccupancyGrid
from atlas.robot.differential_drive import Pose
from atlas.sensors.lidar import LidarScan


@dataclass(frozen=True)
class PoseHypothesis:
    """One candidate global pose and its mean map residual."""

    pose: Pose
    score: float


class LocalizationMonitor:
    """Declare the filter lost after a run of inconsistent scans."""

    def __init__(self, residual_limit: float = 0.5, patience: int = 8) -> None:
        if residual_limit <= 0:
            raise ValueError("residual_limit must be positive")
        if patience < 1:
            raise ValueError("patience must be positive")
        self.residual_limit = residual_limit
        self.patience = patience
        self.bad_steps = 0
        self.triggered = False

    def update(self, residual: float) -> bool:
        """Record one residual. Returns True exactly once when the filter is lost."""
        if residual > self.residual_limit:
            self.bad_steps += 1
        else:
            self.bad_steps = 0

        if not self.triggered and self.bad_steps >= self.patience:
            self.triggered = True
            return True
        return False

    def reset(self) -> None:
        """Rearm the monitor after a successful relocalization."""
        self.bad_steps = 0
        self.triggered = False


class Relocalizer:
    """Coarse-to-fine global pose search with multi-hypothesis confirmation."""

    def __init__(
        self,
        grid: OccupancyGrid,
        max_beams: int = 60,
        coarse_stride: int = 4,
        heading_step_deg: float = 10.0,
        max_candidates: int = 4,
        coarse_separation: float = 1.5,
        refine_distance: float = 0.5,
        refine_position_step: float = 0.25,
        refine_heading_deg: float = 10.0,
        refine_heading_step_deg: float = 5.0,
        max_distance: float = 1.0,
        wall_tolerance: float = 0.10,
    ) -> None:
        if max_beams < 1:
            raise ValueError("max_beams must be positive")
        if coarse_stride < 1:
            raise ValueError("coarse_stride must be positive")
        if not 0.0 < heading_step_deg <= 180.0:
            raise ValueError("heading_step_deg must be in (0, 180]")
        if max_candidates < 1:
            raise ValueError("max_candidates must be positive")
        if coarse_separation <= 0:
            raise ValueError("coarse_separation must be positive")
        if refine_distance < 0 or refine_position_step <= 0:
            raise ValueError("refinement distances must be non-negative")
        if refine_heading_deg < 0 or refine_heading_step_deg <= 0:
            raise ValueError("refinement headings must be non-negative")
        if max_distance <= 0:
            raise ValueError("max_distance must be positive")
        if wall_tolerance < 0:
            raise ValueError("wall_tolerance must be non-negative")

        self.grid = grid
        self.max_beams = max_beams
        self.coarse_stride = coarse_stride
        self.heading_step = math.radians(heading_step_deg)
        self.max_candidates = max_candidates
        self.coarse_separation = coarse_separation
        self.refine_distance = refine_distance
        self.refine_position_step = refine_position_step
        self.refine_heading = math.radians(refine_heading_deg)
        self.refine_heading_step = math.radians(refine_heading_step_deg)
        self.max_distance = max_distance
        self.wall_tolerance = wall_tolerance
        self.field = DistanceField.from_grid(
            grid, include_boundary=True, boundary_only=True
        )

        self._poses: list[Pose] = []
        self._accumulated = np.zeros(0)
        self._observations = 0

    # ---- search ----

    def search(self, scan: LidarScan) -> list[PoseHypothesis]:
        """Rank global poses for one scan and keep the best few."""
        body = self._body_points(scan)
        coarse = self._coarse_candidates(body)
        refined = self._refine(body, coarse)
        self._poses = [pose for pose, _ in refined]
        self._accumulated = np.asarray([score for _, score in refined])
        self._observations = 1
        return [
            PoseHypothesis(pose, score)
            for pose, score in zip(self._poses, self._accumulated)
        ]

    def observe(self, scan: LidarScan, dtheta: float = 0.0) -> None:
        """Score the current scan, accumulate it, then rotate the hypotheses.

        ``dtheta`` is the rotation the robot is about to perform. Scores are
        measured before the rotation, so the accumulated evidence always belongs
        to the pose the scan was taken at.
        """
        if not self._poses:
            raise RuntimeError("call search() before observe()")
        body = self._body_points(scan)
        if len(body) == 0:
            if dtheta:
                self._poses = [
                    Pose(pose.x, pose.y, wrap_angle(pose.theta + dtheta))
                    for pose in self._poses
                ]
            return
        scoring = self._scores(body, self._poses)
        self._accumulated = self._accumulated + scoring
        self._observations += 1
        if dtheta:
            self._poses = [
                Pose(pose.x, pose.y, wrap_angle(pose.theta + dtheta))
                for pose in self._poses
            ]

    def best(self) -> PoseHypothesis:
        """Hypothesis with the lowest mean residual over all observations."""
        if not self._poses:
            raise RuntimeError("call search() before best()")
        index = int(np.argmin(self._accumulated))
        return PoseHypothesis(
            self._poses[index],
            float(self._accumulated[index] / self._observations),
        )

    def reset(self) -> None:
        """Forget the current candidate bank."""
        self._poses = []
        self._accumulated = np.zeros(0)
        self._observations = 0

    @property
    def hypothesis_count(self) -> int:
        return len(self._poses)

    @property
    def observations(self) -> int:
        return self._observations

    # ---- internals ----

    def _body_points(self, scan: LidarScan) -> np.ndarray:
        valid = np.flatnonzero(scan.hit_mask)
        if len(valid) == 0:
            return np.zeros((0, 2))
        if len(valid) > self.max_beams:
            picks = np.linspace(
                0, len(valid) - 1, self.max_beams
            ).round().astype(int)
            valid = valid[picks]
        body = np.column_stack((
            scan.ranges[valid] * np.cos(scan.angles[valid]),
            scan.ranges[valid] * np.sin(scan.angles[valid]),
        ))
        return body + np.asarray(scan.body_origin)

    def _coarse_candidates(self, body: np.ndarray) -> list[Pose]:
        if len(body) == 0:
            return []
        free = np.argwhere(~self.grid.blocked)
        if len(free) == 0:
            return []
        stride = self.coarse_stride
        free = free[
            (free[:, 0] % stride == 0) & (free[:, 1] % stride == 0)
        ]
        if len(free) == 0:
            return []

        resolution = self.grid.resolution
        ox, oy = self.grid.origin
        xs = ox + (free[:, 1] + 0.5) * resolution
        ys = oy + (free[:, 0] + 0.5) * resolution
        count = int(round(2.0 * math.pi / self.heading_step))
        thetas = -math.pi + np.arange(count) * self.heading_step

        columns, headings = np.meshgrid(
            np.arange(len(xs)), np.arange(count), indexing="ij"
        )
        columns = columns.ravel()
        headings = headings.ravel()
        poses = np.column_stack((
            xs[columns], ys[columns], thetas[headings]
        ))
        scores = self._scores(body, poses)

        picked: list[Pose] = []
        for index in np.argsort(scores):
            candidate = poses[index]
            if all(
                math.hypot(
                    candidate[0] - other.x, candidate[1] - other.y
                ) > self.coarse_separation
                for other in picked
            ):
                picked.append(
                    Pose(float(candidate[0]), float(candidate[1]),
                         float(candidate[2]))
                )
            if len(picked) >= self.max_candidates:
                break
        return picked

    def _refine(
        self, body: np.ndarray, candidates: list[Pose]
    ) -> list[tuple[Pose, float]]:
        if not candidates or len(body) == 0:
            return []

        steps = int(round(self.refine_distance / self.refine_position_step))
        offsets = np.arange(-steps, steps + 1) * self.refine_position_step
        steps = int(round(self.refine_heading / self.refine_heading_step))
        headings = np.arange(-steps, steps + 1) * self.refine_heading_step

        best: list[tuple[Pose, float]] = []
        for candidate in candidates:
            poses = np.asarray([
                (
                    candidate.x + dx,
                    candidate.y + dy,
                    wrap_angle(candidate.theta + dtheta),
                )
                for dx in offsets
                for dy in offsets
                for dtheta in headings
            ])
            scores = self._scores(body, poses)
            index = int(np.argmin(scores))
            best.append((
                Pose(float(poses[index, 0]), float(poses[index, 1]),
                     float(wrap_angle(poses[index, 2]))),
                float(scores[index]),
            ))

        best.sort(key=lambda item: item[1])
        distinct: list[tuple[Pose, float]] = []
        for pose, score in best:
            if all(
                math.hypot(pose.x - other.x, pose.y - other.y) > 1.0
                for other, _ in distinct
            ):
                distinct.append((pose, score))
        return distinct[: self.max_candidates]

    def _scores(self, body: np.ndarray, poses) -> np.ndarray:
        """Mean clipped residual for each pose, with world walls as obstacles.

        Endpoints that leave the map are penalised by how far outside they
        land, minus a small wall tolerance. Clamping them to the boundary would
        reward hypotheses whose beams pass through walls, which makes a wrong
        pose look better than the true one.
        """
        poses = np.asarray([
            (pose.x, pose.y, pose.theta) if isinstance(pose, Pose) else pose
            for pose in poses
        ])
        x, y, theta = poses[:, 0], poses[:, 1], poses[:, 2]
        cos_t, sin_t = np.cos(theta), np.sin(theta)
        world_x = (
            x[:, None] + cos_t[:, None] * body[:, 0]
            - sin_t[:, None] * body[:, 1]
        )
        world_y = (
            y[:, None] + sin_t[:, None] * body[:, 0]
            + cos_t[:, None] * body[:, 1]
        )

        width, height = self.grid.width, self.grid.height
        ox, oy = self.grid.origin
        resolution = self.grid.resolution
        xmin, ymin = ox, oy
        xmax = ox + width * resolution
        ymax = oy + height * resolution
        inside = (
            (world_x >= xmin) & (world_x <= xmax)
            & (world_y >= ymin) & (world_y <= ymax)
        )

        distances = np.empty(world_x.shape)
        distances[inside] = self.field.sample(
            np.column_stack((world_x[inside], world_y[inside]))
        )
        outside_x = np.maximum(
            np.maximum(xmin - world_x, world_x - xmax), 0.0
        )
        outside_y = np.maximum(
            np.maximum(ymin - world_y, world_y - ymax), 0.0
        )
        outside = np.hypot(outside_x, outside_y) - self.wall_tolerance
        distances[~inside] = np.maximum(outside[~inside], 0.0)

        distances = np.minimum(distances, self.max_distance)
        return distances.mean(axis=1)
