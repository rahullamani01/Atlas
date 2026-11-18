"""Next-best-view exploration on an unknown map (Phase 13).

The robot starts with an empty prior, so every cell is unknown. Exploration
chooses where to drive next by scoring a coarse lattice of candidate viewpoints
on how much unknown area the LiDAR would reveal from each one, divided by the
A* cost of getting there. Unknown cells are treated as free while planning, so
the robot is willing to drive into space it has not seen yet. A target that
turns out to be unreachable is blacklisted by the runner instead of aborting
the run.

Scoring is deliberately simple and deterministic. A denser lattice or a
continuous information-gain field would find slightly better viewpoints, but
the coarse version is cheap enough to re-run after every arrival and its
decisions are easy to explain: the best score wins and a tie keeps the first
candidate.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np

from atlas.mapping.occupancy_map import LogOddsMap
from atlas.planning import OccupancyGrid, a_star
from atlas.planning.grid import Cell
from atlas.recovery import nearest_free_cell

DEFAULT_SPACING = 8
DEFAULT_RADIUS = 40
DEFAULT_MIN_GAIN = 40


@dataclass(frozen=True)
class Viewpoint:
    """A scored target cell: how much it would reveal for what travel cost."""

    cell: Cell
    gain: int
    cost: float
    score: float


def unknown_area(
    unknown: np.ndarray, cell: Cell, radius: int = DEFAULT_RADIUS
) -> int:
    """Count unknown cells inside a disk of ``radius`` centred on ``cell``."""
    if radius < 0:
        raise ValueError("radius must be non-negative")
    unknown = np.asarray(unknown, dtype=bool)
    col, row = cell
    height, width = unknown.shape
    if not (0 <= row < height and 0 <= col < width):
        return 0
    r = int(radius)
    r0, r1 = max(0, row - r), min(height, row + r + 1)
    c0, c1 = max(0, col - r), min(width, col + r + 1)
    rows = np.arange(r0, r1)[:, None] - row
    cols = np.arange(c0, c1)[None, :] - col
    disk = rows * rows + cols * cols <= r * r
    return int(np.count_nonzero(unknown[r0:r1, c0:c1] & disk))


def candidate_cells(
    grid: OccupancyGrid,
    spacing: int = DEFAULT_SPACING,
    *,
    visited: Sequence[Cell] = (),
    blacklist: Sequence[Cell] = (),
    start: Cell | None = None,
) -> list[Cell]:
    """Free lattice points that are not near an excluded target.

    The lattice is offset by half a spacing so it does not sit on the map
    border. ``start`` removes the cell the robot already occupies, which would
    otherwise win with zero travel cost.
    """
    if spacing < 1:
        raise ValueError("spacing must be positive")
    excluded = list(visited) + list(blacklist)
    cells: list[Cell] = []
    for row in range(spacing // 2, grid.height, spacing):
        for col in range(spacing // 2, grid.width, spacing):
            cell = (col, row)
            if start is not None and cell == start:
                continue
            if not grid.is_free(cell):
                continue
            if any(
                math.hypot(col - other[0], row - other[1]) < spacing
                for other in excluded
            ):
                continue
            cells.append(cell)
    return cells


def select_viewpoint(
    mapping: LogOddsMap,
    grid: OccupancyGrid,
    start: Cell,
    *,
    spacing: int = DEFAULT_SPACING,
    radius: int = DEFAULT_RADIUS,
    min_gain: int = DEFAULT_MIN_GAIN,
    visited: Sequence[Cell] = (),
    blacklist: Sequence[Cell] = (),
) -> Viewpoint | None:
    """Return the reachable viewpoint with the best gain-per-travel score.

    ``grid`` is the planner's optimistic view of ``mapping`` (unknown cells are
    free). ``start`` must be free on that grid. ``visited`` and ``blacklist``
    are cell coordinates that should not be proposed again.
    """
    if min_gain < 0:
        raise ValueError("min_gain must be non-negative")
    unknown = ~mapping.known
    best: Viewpoint | None = None
    for cell in candidate_cells(
        grid,
        spacing,
        visited=visited,
        blacklist=blacklist,
        start=start,
    ):
        target = nearest_free_cell(grid, cell)
        if target is None:
            continue
        gain = unknown_area(unknown, target, radius)
        if gain < min_gain:
            continue
        result = a_star(grid, start, target)
        if not result.found:
            continue
        score = gain / (1.0 + result.cost)
        if best is None or score > best.score:
            best = Viewpoint(target, gain, float(result.cost), score)
    return best
