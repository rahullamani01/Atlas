"""Detect unexpected obstacles from LiDAR against a known map (Phase 6).

The planner owns a known, static occupancy grid. This module maintains a copy
of that grid plus evidence for cells that LiDAR returns say are occupied but the
known map says are free. Confirmed cells are added to the planning grid with a
clearance margin, so the planner can route around them.

Evidence decays every update. A permanent obstacle is re-observed continuously
and stays blocked; a transient return that is never seen again fades out. Cells
that the known map already marks as occupied (including inflation) are ignored,
so walls and existing obstacles do not become "new" detections.
"""
from __future__ import annotations

import math

import numpy as np

from atlas.planning.grid import Cell, OccupancyGrid
from atlas.robot.differential_drive import Pose
from atlas.sensors.lidar import LidarScan


def _dilate(mask: np.ndarray, radius: int) -> np.ndarray:
    """Grow a boolean mask by a square neighbourhood of `radius` cells."""
    if radius <= 0:
        return mask.copy()
    height, width = mask.shape
    padded = np.pad(mask, radius, mode="constant", constant_values=False)
    grown = np.zeros_like(mask)
    for dr in range(-radius, radius + 1):
        for dc in range(-radius, radius + 1):
            grown |= padded[
                radius + dr: radius + dr + height,
                radius + dc: radius + dc + width,
            ]
    return grown


class DynamicObstacleMap:
    """A planning grid that grows as unexpected LiDAR returns are confirmed."""

    def __init__(
        self,
        known_grid: OccupancyGrid,
        confirm_threshold: float = 2.0,
        unblock_threshold: float = 0.5,
        decay: float = 0.8,
        tolerance: int = 1,
        clearance: float = 0.25,
    ) -> None:
        if confirm_threshold <= 0:
            raise ValueError("confirm_threshold must be positive")
        if not 0 <= unblock_threshold < confirm_threshold:
            raise ValueError(
                "unblock_threshold must be non-negative and below confirm_threshold"
            )
        if not 0 <= decay < 1:
            raise ValueError("decay must be in [0, 1)")
        if tolerance < 0:
            raise ValueError("tolerance must be non-negative")
        if clearance < 0:
            raise ValueError("clearance must be non-negative")

        self.known = known_grid
        self.grid = OccupancyGrid(
            known_grid.width,
            known_grid.height,
            known_grid.resolution,
            known_grid.origin,
            known_grid.blocked.copy(),
        )
        self.confirm_threshold = float(confirm_threshold)
        self.unblock_threshold = float(unblock_threshold)
        self.decay = float(decay)
        self.tolerance = int(tolerance)
        self.clearance = float(clearance)
        self.clearance_cells = max(
            1, int(math.ceil(clearance / known_grid.resolution))
        ) if clearance > 0 else 0

        self.evidence = np.zeros((known_grid.height, known_grid.width))
        self.confirmed = np.zeros((known_grid.height, known_grid.width), dtype=bool)
        self.dynamic = np.zeros((known_grid.height, known_grid.width), dtype=bool)
        self._known_near = _dilate(known_grid.blocked, self.tolerance)
        # World walls are implicit in the planning grid, so LiDAR returns from
        # them land in the outer free cells. With an estimated pose a return can
        # land one tolerance band further in, so mark `tolerance + 1` cells as
        # known. This is much thinner than the clearance used for real
        # obstacles, and it keeps boundary walls from becoming false detections.
        band = self.tolerance + 1
        self._known_near[:band, :] = True
        self._known_near[-band:, :] = True
        self._known_near[:, :band] = True
        self._known_near[:, -band:] = True

    def integrate(self, scan: LidarScan, pose: Pose | None = None) -> list[Cell]:
        """Fade old evidence, add this scan's returns, and update the grid.

        ``pose`` overrides the sensor pose stored in the scan. Pass the current
        pose estimate when the scan was produced from ground truth, so map
        consistency is evaluated in the estimated frame.

        Returns the list of ``(col, row)`` cells that became blocked this call.
        """
        height, width = self.evidence.shape
        self.evidence *= self.decay

        points = scan.points() if pose is None else scan.points_at(pose)
        if points.size:
            ox, oy = self.grid.origin
            resolution = self.grid.resolution
            cols = np.floor((points[:, 0] - ox) / resolution).astype(int)
            rows = np.floor((points[:, 1] - oy) / resolution).astype(int)
            valid = (
                (rows >= 0) & (rows < height) & (cols >= 0) & (cols < width)
            )
            unexpected = np.zeros(len(points), dtype=bool)
            unexpected[valid] = ~self._known_near[rows[valid], cols[valid]]
            if np.any(unexpected):
                np.add.at(
                    self.evidence,
                    (rows[unexpected], cols[unexpected]),
                    1.0,
                )

        confirmed = self.evidence >= self.confirm_threshold
        released = self.evidence < self.unblock_threshold
        self.confirmed = (self.confirmed | confirmed) & ~released
        desired = _dilate(self.confirmed, self.clearance_cells) & ~self.known.blocked
        newly_blocked = desired & ~self.grid.blocked

        self.dynamic = desired
        self.grid.blocked[:] = self.known.blocked | desired

        rows, cols = np.nonzero(newly_blocked)
        return [(int(col), int(row)) for row, col in zip(rows, cols)]

    def reset(self) -> None:
        """Forget every dynamic detection and restore the known map."""
        self.evidence.fill(0.0)
        self.confirmed.fill(False)
        self.dynamic.fill(False)
        self.grid.blocked[:] = self.known.blocked

    @property
    def detected_count(self) -> int:
        return int(np.count_nonzero(self.dynamic))
