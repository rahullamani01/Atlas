"""Occupancy grid environment for planning (Phase 2).

Cell indexing: ``blocked[row, col]``. Column ``col`` maps to world x, row ``row``
to world y. A cell is free when it is in bounds and not blocked.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterator

import numpy as np

from atlas.simulation.world import World

Cell = tuple[int, int]

# (dcol, drow, step cost) for the two supported neighbourhoods.
_MOVES_4 = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0))
_MOVES_8 = _MOVES_4 + (
    (1, 1, math.sqrt(2.0)),
    (1, -1, math.sqrt(2.0)),
    (-1, 1, math.sqrt(2.0)),
    (-1, -1, math.sqrt(2.0)),
)


@dataclass
class OccupancyGrid:
    """A rectangular occupancy grid over a world region.

    Attributes
    ----------
    width, height:
        Grid size in cells.
    resolution:
        Side length of one cell in metres.
    origin:
        World coordinates of the grid's lower-left corner.
    blocked:
        Boolean array of shape ``(height, width)``; True means occupied.
    """

    width: int
    height: int
    resolution: float = 1.0
    origin: tuple[float, float] = (0.0, 0.0)
    blocked: np.ndarray | None = field(default=None, repr=False)

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("grid width and height must be positive")
        if self.resolution <= 0:
            raise ValueError("resolution must be positive")
        if self.blocked is None:
            self.blocked = np.zeros((self.height, self.width), dtype=bool)
        else:
            self.blocked = np.asarray(self.blocked, dtype=bool)
        if self.blocked.shape != (self.height, self.width):
            raise ValueError(
                f"blocked shape {self.blocked.shape} does not match "
                f"({self.height}, {self.width})"
            )

    # ---- geometry ----

    def in_bounds(self, cell: Cell) -> bool:
        col, row = cell
        return 0 <= col < self.width and 0 <= row < self.height

    def world_to_cell(self, x: float, y: float) -> Cell:
        """World point -> cell containing it (may be out of bounds)."""
        ox, oy = self.origin
        return (
            math.floor((x - ox) / self.resolution),
            math.floor((y - oy) / self.resolution),
        )

    def cell_to_world(self, cell: Cell) -> tuple[float, float]:
        """Cell -> world coordinates of its centre."""
        col, row = cell
        ox, oy = self.origin
        return (
            ox + (col + 0.5) * self.resolution,
            oy + (row + 0.5) * self.resolution,
        )

    # ---- occupancy ----

    def is_free(self, cell: Cell) -> bool:
        if not self.in_bounds(cell):
            return False
        col, row = cell
        return not bool(self.blocked[row, col])

    def set_blocked(self, cell: Cell, value: bool = True) -> None:
        if not self.in_bounds(cell):
            raise ValueError(f"cell {cell} is outside the grid")
        col, row = cell
        self.blocked[row, col] = value

    def free_count(self) -> int:
        return int(self.width * self.height - np.count_nonzero(self.blocked))

    def free_ratio(self) -> float:
        return self.free_count() / (self.width * self.height)

    # ---- construction from a simulated world ----

    @classmethod
    def from_world(
        cls,
        world: World,
        resolution: float = 1.0,
        inflate_radius: float = 0.0,
        origin: tuple[float, float] = (0.0, 0.0),
    ) -> "OccupancyGrid":
        """Rasterise a `World` and optionally inflate obstacles.

        A cell is blocked when its centre lies inside a rectangular obstacle.
        `inflate_radius` then grows every obstacle by that radius, which is how
        robot body size is accounted for during planning.
        """
        width = math.ceil(world.width / resolution)
        height = math.ceil(world.height / resolution)
        ox, oy = origin
        blocked = np.zeros((height, width), dtype=bool)

        for row in range(height):
            y = oy + (row + 0.5) * resolution
            for col in range(width):
                x = ox + (col + 0.5) * resolution
                blocked[row, col] = any(
                    o.xmin <= x <= o.xmax and o.ymin <= y <= o.ymax
                    for o in world.obstacles
                )

        grid = cls(width, height, resolution, origin, blocked)
        return grid.inflated(inflate_radius) if inflate_radius > 0 else grid

    def inflated(self, radius: float) -> "OccupancyGrid":
        """Return a copy with blocked cells grown by `radius` metres.

        The growth is deliberately conservative: cells whose centres are within
        one extra half-cell of the radius are also blocked, so a path can never
        pass closer than `radius` to an obstacle.
        """
        if radius < 0:
            raise ValueError("inflation radius must be non-negative")
        grown = self.blocked.copy()
        if radius == 0:
            return OccupancyGrid(
                self.width, self.height, self.resolution, self.origin, grown
            )

        reach = math.ceil((radius + 0.5 * self.resolution) / self.resolution)
        offsets = [
            (dc, dr)
            for dc in range(-reach, reach + 1)
            for dr in range(-reach, reach + 1)
            if math.hypot(dc, dr) * self.resolution
            <= radius + 0.5 * self.resolution + 1e-12
        ]

        rows, cols = np.nonzero(self.blocked)
        for row, col in zip(rows, cols):
            for dc, dr in offsets:
                c, r = col + dc, row + dr
                if 0 <= c < self.width and 0 <= r < self.height:
                    grown[r, c] = True

        return OccupancyGrid(self.width, self.height, self.resolution, self.origin, grown)

    # ---- graph interface ----

    def neighbors(
        self,
        cell: Cell,
        connectivity: int = 8,
        allow_corner_cutting: bool = False,
    ) -> Iterator[tuple[Cell, float]]:
        """Yield ``(neighbour, step_cost)`` for free neighbours of `cell`.

        With 8-connectivity, a diagonal move is rejected when either adjacent
        orthogonal cell is blocked unless `allow_corner_cutting` is True. This
        prevents a circular robot from squeezing through a wall corner.
        """
        if connectivity not in (4, 8):
            raise ValueError("connectivity must be 4 or 8")
        moves = _MOVES_4 if connectivity == 4 else _MOVES_8
        col, row = cell

        for dc, dr, step in moves:
            neighbour = (col + dc, row + dr)
            if not self.is_free(neighbour):
                continue
            if connectivity == 8 and dc != 0 and dr != 0 and not allow_corner_cutting:
                if not (self.is_free((col + dc, row)) and self.is_free((col, row + dr))):
                    continue
            yield neighbour, step
