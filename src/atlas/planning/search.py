"""Graph search over occupancy grids: Dijkstra and A* (Phase 2).

Both algorithms return a `SearchResult`. Dijkstra is A* with a zero heuristic;
A* is optimal when its heuristic is admissible (and, with the closed set used
here, consistent).
"""
from __future__ import annotations

import heapq
import itertools
import math
from dataclasses import dataclass, field
from typing import Callable

from atlas.planning.grid import Cell, OccupancyGrid

Heuristic = Callable[[Cell, Cell], float]


@dataclass
class SearchResult:
    """Outcome of a search.

    ``path`` includes both start and goal and is empty when no route exists.
    ``expanded_cells`` is the order in which cells were closed, useful for
    visualising how much of the map each algorithm explored.
    """

    path: list[Cell] = field(default_factory=list)
    cost: float = math.inf
    expanded: int = 0
    expanded_cells: list[Cell] = field(default_factory=list)

    @property
    def found(self) -> bool:
        return bool(self.path)


# ---- heuristics ----

def zero(_a: Cell, _b: Cell) -> float:
    return 0.0


def manhattan(a: Cell, b: Cell) -> float:
    """admissible only for 4-connected grids"""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def euclidean(a: Cell, b: Cell) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def octile(a: Cell, b: Cell) -> float:
    """Exact shortest distance on an 8-connected uniform grid."""
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    return (dx + dy) + (math.sqrt(2.0) - 2.0) * min(dx, dy)


HEURISTICS: dict[str, Heuristic] = {
    "zero": zero,
    "manhattan": manhattan,
    "euclidean": euclidean,
    "octile": octile,
}


# ---- search ----

def _check_endpoint(grid: OccupancyGrid, cell: Cell, name: str) -> None:
    if not grid.in_bounds(cell):
        raise ValueError(f"{name} {cell} is outside the grid")
    if not grid.is_free(cell):
        raise ValueError(f"{name} {cell} is blocked")


def _reconstruct(came_from: dict[Cell, Cell], start: Cell, goal: Cell) -> list[Cell]:
    path = [goal]
    while path[-1] != start:
        path.append(came_from[path[-1]])
    path.reverse()
    return path


def _search(
    grid: OccupancyGrid,
    start: Cell,
    goal: Cell,
    heuristic: Heuristic,
    connectivity: int,
    allow_corner_cutting: bool,
) -> SearchResult:
    _check_endpoint(grid, start, "start")
    _check_endpoint(grid, goal, "goal")

    if start == goal:
        return SearchResult([start], 0.0, 1, [start])

    counter = itertools.count()
    # (f, h, tie_breaker, cell): preferring lower h breaks f-ties toward goal.
    open_heap: list[tuple[float, float, int, Cell]] = [
        (heuristic(start, goal), heuristic(start, goal), next(counter), start)
    ]
    came_from: dict[Cell, Cell] = {}
    g_score: dict[Cell, float] = {start: 0.0}
    closed: set[Cell] = set()
    expanded_cells: list[Cell] = []

    while open_heap:
        _, _, _, cell = heapq.heappop(open_heap)
        if cell in closed:
            continue
        closed.add(cell)
        expanded_cells.append(cell)

        if cell == goal:
            path = _reconstruct(came_from, start, goal)
            return SearchResult(path, g_score[goal], len(closed), expanded_cells)

        for neighbour, step in grid.neighbors(
            cell, connectivity, allow_corner_cutting
        ):
            if neighbour in closed:
                continue
            tentative = g_score[cell] + step
            if tentative < g_score.get(neighbour, math.inf):
                came_from[neighbour] = cell
                g_score[neighbour] = tentative
                h = heuristic(neighbour, goal)
                heapq.heappush(
                    open_heap, (tentative + h, h, next(counter), neighbour)
                )

    return SearchResult([], math.inf, len(closed), expanded_cells)


def dijkstra(
    grid: OccupancyGrid,
    start: Cell,
    goal: Cell,
    connectivity: int = 8,
    allow_corner_cutting: bool = False,
) -> SearchResult:
    """Uniform-cost search. Optimal for any non-negative step costs."""
    return _search(grid, start, goal, zero, connectivity, allow_corner_cutting)


def a_star(
    grid: OccupancyGrid,
    start: Cell,
    goal: Cell,
    heuristic: str | Heuristic = "octile",
    connectivity: int = 8,
    allow_corner_cutting: bool = False,
) -> SearchResult:
    """A* search. ``heuristic`` is a name from HEURISTICS or a callable.

    Raises ValueError for ``manhattan`` with 8-connectivity, where it can
    overestimate and would break optimality.
    """
    if isinstance(heuristic, str):
        if heuristic not in HEURISTICS:
            raise ValueError(
                f"unknown heuristic {heuristic!r}; choose from {sorted(HEURISTICS)}"
            )
        if heuristic == "manhattan" and connectivity == 8:
            raise ValueError("manhattan is inadmissible with 8-connectivity")
        heuristic_fn = HEURISTICS[heuristic]
    else:
        heuristic_fn = heuristic

    return _search(
        grid, start, goal, heuristic_fn, connectivity, allow_corner_cutting
    )
