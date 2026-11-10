"""Distance-to-obstacle field for LiDAR measurement likelihood (Phase 5).

The field stores, for every grid cell, the approximate Euclidean distance in
metres to the nearest blocked cell. It is precomputed once per map with a
two-pass chamfer transform, so a particle filter can score a scan endpoint with
a single array lookup.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from atlas.planning.grid import OccupancyGrid


@dataclass(frozen=True)
class DistanceField:
    distances: np.ndarray      # metres, shape (height, width)
    resolution: float
    origin: tuple[float, float]
    include_boundary: bool = True

    @classmethod
    def from_grid(
        cls,
        grid: OccupancyGrid,
        include_boundary: bool = True,
        boundary_only: bool = False,
    ) -> "DistanceField":
        """Build the field with an 8-connected chamfer distance transform.

        `include_boundary` treats the world walls as obstacles analytically.
        `boundary_only` places sources only on the free-facing edge of
        obstacles, so a point pushed deep inside an obstacle is penalised
        instead of also scoring zero.
        """
        height, width = grid.blocked.shape
        free = ~grid.blocked
        sources = grid.blocked.copy()

        if boundary_only:
            adjacent_free = np.zeros_like(grid.blocked)
            adjacent_free[1:, :] |= free[:-1, :]
            adjacent_free[:-1, :] |= free[1:, :]
            adjacent_free[:, 1:] |= free[:, :-1]
            adjacent_free[:, :-1] |= free[:, 1:]
            sources = grid.blocked & adjacent_free

        inf = float("inf")
        distance = np.where(sources, 0.0, inf)
        diagonal = math.sqrt(2.0)

        for row in range(height):
            for col in range(width):
                best = distance[row, col]
                if row > 0:
                    best = min(best, distance[row - 1, col] + 1.0)
                    if col > 0:
                        best = min(best, distance[row - 1, col - 1] + diagonal)
                    if col + 1 < width:
                        best = min(best, distance[row - 1, col + 1] + diagonal)
                if col > 0:
                    best = min(best, distance[row, col - 1] + 1.0)
                distance[row, col] = best

        for row in range(height - 1, -1, -1):
            for col in range(width - 1, -1, -1):
                best = distance[row, col]
                if row + 1 < height:
                    best = min(best, distance[row + 1, col] + 1.0)
                    if col > 0:
                        best = min(best, distance[row + 1, col - 1] + diagonal)
                    if col + 1 < width:
                        best = min(best, distance[row + 1, col + 1] + diagonal)
                if col + 1 < width:
                    best = min(best, distance[row, col + 1] + 1.0)
                distance[row, col] = best

        return cls(
            distance * grid.resolution,
            grid.resolution,
            grid.origin,
            include_boundary,
        )

    @property
    def _bounds(self) -> tuple[float, float, float, float]:
        xmin, ymin = self.origin
        height, width = self.distances.shape
        return xmin, ymin, xmin + width * self.resolution, ymin + height * self.resolution

    def _wall_distance(self, x: float, y: float) -> float:
        xmin, ymin, xmax, ymax = self._bounds
        if not (xmin <= x <= xmax and ymin <= y <= ymax):
            return math.inf
        return min(x - xmin, xmax - x, y - ymin, ymax - y)

    def distance_at(self, x: float, y: float) -> float:
        """Distance to the nearest obstacle or world wall."""
        ox, oy = self.origin
        col = math.floor((x - ox) / self.resolution)
        row = math.floor((y - oy) / self.resolution)
        if not (0 <= row < self.distances.shape[0] and 0 <= col < self.distances.shape[1]):
            return math.inf
        distance = float(self.distances[row, col])
        if self.include_boundary:
            distance = min(distance, self._wall_distance(x, y))
        return distance

    def sample(self, points: np.ndarray) -> np.ndarray:
        """Vectorised lookup for an (N, 2) array of world points."""
        points = np.atleast_2d(np.asarray(points, dtype=float))
        ox, oy = self.origin
        cols = np.floor((points[:, 0] - ox) / self.resolution).astype(int)
        rows = np.floor((points[:, 1] - oy) / self.resolution).astype(int)
        height, width = self.distances.shape
        valid = (rows >= 0) & (rows < height) & (cols >= 0) & (cols < width)

        result = np.full(len(points), math.inf)
        result[valid] = self.distances[rows[valid], cols[valid]]

        if self.include_boundary:
            xmin, ymin, xmax, ymax = self._bounds
            xs, ys = points[:, 0], points[:, 1]
            inside = (xs >= xmin) & (xs <= xmax) & (ys >= ymin) & (ys <= ymax)
            wall = np.minimum.reduce(
                (xs - xmin, xmax - xs, ys - ymin, ymax - ys)
            )
            wall = np.where(inside, np.maximum(wall, 0.0), math.inf)
            result = np.minimum(result, wall)

        return result
