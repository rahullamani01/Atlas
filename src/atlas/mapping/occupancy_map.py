"""Occupancy mapping and map repair from LiDAR (Phase 10).

The earlier phases assume the occupancy grid is known and correct. This module
builds a map from range measurements instead. Each cell stores log-odds of being
occupied; a beam carves free space along its length and adds occupancy evidence
at its endpoint. Because free observations also lower the log-odds of cells that
were previously occupied, the same update both grows the map and repairs stale
cells: a phantom obstacle fades as the robot sees through it, and a real
obstacle that was missing from the prior is added.

The sensor model is deliberately simple. It is an inverse sensor model applied
per beam, not a learned or ray-cast-through-uncertainty model. That keeps the
update vectorisable and makes the repair behaviour easy to reason about.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from atlas.planning.grid import Cell, OccupancyGrid
from atlas.robot.differential_drive import Pose
from atlas.sensors.lidar import LidarScan

_EPS = 1e-9


@dataclass(frozen=True)
class MapUpdate:
    """Cells whose occupancy changed during one integration step."""

    changed: tuple[Cell, ...] = ()
    added: tuple[Cell, ...] = ()
    removed: tuple[Cell, ...] = ()

    @property
    def empty(self) -> bool:
        return not self.changed


def surface_cells(blocked: np.ndarray) -> np.ndarray:
    """Return blocked cells that touch free space.

    Mapping observes obstacle *surfaces*, not obstacle interiors. Comparing a
    mapped grid with a rasterised solid obstacle directly therefore understates
    accuracy. Restricting the comparison to surface cells makes the prior and
    the mapped grid comparable.
    """
    blocked = np.asarray(blocked, dtype=bool)
    padded = np.pad(blocked, 1, mode="constant", constant_values=True)
    interior = (
        padded[:-2, 1:-1]
        & padded[2:, 1:-1]
        & padded[1:-1, :-2]
        & padded[1:-1, 2:]
    )
    return blocked & ~interior


def map_error(
    predicted: np.ndarray,
    truth: np.ndarray,
    mask: np.ndarray | None = None,
) -> int:
    """Symmetric difference between two obstacle surfaces, optionally masked.

    ``mask`` should select cells the robot actually observed. Without it,
    obstacles the robot never approached count as errors for both maps, which
    makes repair progress look smaller than it is.
    """
    predicted_surface = surface_cells(predicted)
    truth_surface = surface_cells(truth)
    if mask is not None:
        predicted_surface = predicted_surface & mask
        truth_surface = truth_surface & mask
    return int(np.count_nonzero(predicted_surface ^ truth_surface))


class LogOddsMap:
    """A log-odds occupancy grid updated from LiDAR scans at estimated poses."""

    def __init__(
        self,
        prior: OccupancyGrid,
        prior_occupied_log_odds: float = 1.5,
        prior_free_log_odds: float = -0.75,
        hit_log_odds: float = 0.85,
        miss_log_odds: float = -0.45,
        min_log_odds: float = -4.0,
        max_log_odds: float = 4.0,
        occupied_threshold: float = 0.0,
        max_range: float = 10.0,
        max_beams: int = 120,
        free_margin: float = 0.15,
        known: np.ndarray | None = None,
    ) -> None:
        if prior_occupied_log_odds <= prior_free_log_odds:
            raise ValueError(
                "prior occupied log-odds must exceed prior free log-odds"
            )
        if hit_log_odds <= 0:
            raise ValueError("hit_log_odds must be positive")
        if miss_log_odds >= 0:
            raise ValueError("miss_log_odds must be negative")
        if min_log_odds >= max_log_odds:
            raise ValueError("min_log_odds must be below max_log_odds")
        if not min_log_odds <= occupied_threshold <= max_log_odds:
            raise ValueError("occupied_threshold must lie inside the clamp range")
        if max_range <= 0:
            raise ValueError("max_range must be positive")
        if max_beams < 1:
            raise ValueError("max_beams must be positive")
        if free_margin < 0:
            raise ValueError("free_margin must be non-negative")

        self.prior = prior
        self.width = prior.width
        self.height = prior.height
        self.resolution = prior.resolution
        self.origin = prior.origin
        self.hit_log_odds = float(hit_log_odds)
        self.miss_log_odds = float(miss_log_odds)
        self.min_log_odds = float(min_log_odds)
        self.max_log_odds = float(max_log_odds)
        self.occupied_threshold = float(occupied_threshold)
        self.max_range = float(max_range)
        self.max_beams = int(max_beams)
        self.free_margin = float(free_margin)

        if known is None:
            self.known = np.ones((self.height, self.width), dtype=bool)
        else:
            self.known = np.asarray(known, dtype=bool)
            if self.known.shape != (self.height, self.width):
                raise ValueError(
                    f"known shape {self.known.shape} does not match "
                    f"({self.height}, {self.width})"
                )

        self.log_odds = np.where(
            prior.blocked, prior_occupied_log_odds, prior_free_log_odds
        )
        self.log_odds = np.where(self.known, self.log_odds, 0.0)
        self.observed = np.zeros((self.height, self.width), dtype=bool)

    # ---- properties ----

    @property
    def occupancy(self) -> np.ndarray:
        """Boolean occupancy under the current evidence."""
        return self.known & (self.log_odds > self.occupied_threshold)

    @property
    def known_ratio(self) -> float:
        return float(np.mean(self.known))

    @property
    def observed_ratio(self) -> float:
        return float(np.mean(self.observed))

    def to_grid(self, inflate_radius: float = 0.0) -> OccupancyGrid:
        """Return a planning grid. Unknown cells are treated as free.

        Optimistic treatment of unknown space is intentional: the planner should
        not refuse to move simply because the robot has not looked there yet.
        Observed obstacles are inflated by ``inflate_radius`` for clearance.
        """
        grid = OccupancyGrid(
            self.width,
            self.height,
            self.resolution,
            self.origin,
            self.occupancy,
        )
        return grid.inflated(inflate_radius) if inflate_radius > 0 else grid

    # ---- mapping ----

    def integrate(self, scan: LidarScan, pose: Pose) -> MapUpdate:
        """Apply one scan at ``pose`` and return the occupancy changes."""
        if scan.ray_count == 0:
            return MapUpdate()

        before = self.occupancy
        beams = np.arange(scan.ray_count)
        if len(beams) > self.max_beams:
            beams = beams[
                np.linspace(0, len(beams) - 1, self.max_beams).round().astype(int)
            ]

        cos_t, sin_t = math.cos(pose.theta), math.sin(pose.theta)
        mount_x, mount_y = scan.body_origin
        origin_x = pose.x + cos_t * mount_x - sin_t * mount_y
        origin_y = pose.y + sin_t * mount_x + cos_t * mount_y

        free_indices: list[np.ndarray] = []
        hit_indices: list[int] = []
        size = self.width * self.height

        for index in beams:
            angle = pose.theta + float(scan.angles[index])
            distance = float(scan.ranges[index])
            hit = distance < scan.max_range - _EPS

            free_length = distance - self.free_margin if hit else distance
            if free_length > 0:
                indices = self._ray_indices(
                    origin_x, origin_y, angle, free_length
                )
                if indices.size:
                    free_indices.append(indices)

            if hit:
                end_x = origin_x + distance * math.cos(angle)
                end_y = origin_y + distance * math.sin(angle)
                col, row = self._world_cell(end_x, end_y)
                if 0 <= col < self.width and 0 <= row < self.height:
                    hit_indices.append(row * self.width + col)

        flat = self.log_odds.reshape(-1)
        known_flat = self.known.reshape(-1)
        observed_flat = self.observed.reshape(-1)

        if free_indices:
            indices = np.concatenate(free_indices)
            counts = np.bincount(indices, minlength=size)
            touched = counts > 0
            flat[touched] += self.miss_log_odds * counts[touched]
            known_flat[touched] = True
            observed_flat[touched] = True

        if hit_indices:
            indices = np.asarray(hit_indices, dtype=int)
            counts = np.bincount(indices, minlength=size)
            touched = counts > 0
            flat[touched] += self.hit_log_odds * counts[touched]
            known_flat[touched] = True
            observed_flat[touched] = True

        np.clip(flat, self.min_log_odds, self.max_log_odds, out=flat)

        after = self.occupancy
        changed = before ^ after
        added = ~before & after
        removed = before & ~after

        return MapUpdate(
            changed=self._cells(changed),
            added=self._cells(added),
            removed=self._cells(removed),
        )

    def _ray_indices(
        self,
        origin_x: float,
        origin_y: float,
        angle: float,
        length: float,
    ) -> np.ndarray:
        step = 0.5 * self.resolution
        count = int(math.floor(length / step)) + 1
        if count < 1:
            return np.zeros(0, dtype=int)
        t = np.arange(count, dtype=float) * step
        xs = origin_x + t * math.cos(angle)
        ys = origin_y + t * math.sin(angle)
        cols = np.floor((xs - self.origin[0]) / self.resolution).astype(int)
        rows = np.floor((ys - self.origin[1]) / self.resolution).astype(int)
        valid = (
            (cols >= 0)
            & (cols < self.width)
            & (rows >= 0)
            & (rows < self.height)
        )
        if not np.any(valid):
            return np.zeros(0, dtype=int)
        return np.unique(rows[valid] * self.width + cols[valid])

    def _world_cell(self, x: float, y: float) -> Cell:
        col = math.floor((x - self.origin[0]) / self.resolution)
        row = math.floor((y - self.origin[1]) / self.resolution)
        return int(col), int(row)

    def _cells(self, mask: np.ndarray) -> tuple[Cell, ...]:
        rows, cols = np.nonzero(mask)
        return tuple(
            (int(col), int(row)) for row, col in zip(rows, cols)
        )
