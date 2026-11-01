import math

import pytest

from atlas.planning.grid import OccupancyGrid
from atlas.planning.search import (
    SearchResult,
    a_star,
    dijkstra,
    euclidean,
    manhattan,
    octile,
    zero,
)


def open_grid(width=20, height=20):
    return OccupancyGrid(width, height, resolution=1.0)


def assert_valid_path(grid, result, start, goal, connectivity=8):
    assert result.found
    assert result.path[0] == start
    assert result.path[-1] == goal
    assert len(set(result.path)) == len(result.path)
    for cell in result.path:
        assert grid.is_free(cell)
    moves = {
        (dc, dr)
        for dc in (-1, 0, 1)
        for dr in (-1, 0, 1)
        if (dc, dr) != (0, 0)
        and (connectivity == 8 or dc == 0 or dr == 0)
    }
    for a, b in zip(result.path, result.path[1:]):
        assert (b[0] - a[0], b[1] - a[1]) in moves


def test_straight_line_four_connected():
    grid = open_grid(10, 3)
    result = dijkstra(grid, (0, 0), (9, 0), connectivity=4)
    assert_valid_path(grid, result, (0, 0), (9, 0), connectivity=4)
    assert result.cost == pytest.approx(9.0)
    assert len(result.path) == 10


def test_a_star_matches_dijkstra_cost():
    grid = open_grid(15, 15)
    grid.blocked[3:12, 7] = True
    d = dijkstra(grid, (0, 0), (14, 14))
    a = a_star(grid, (0, 0), (14, 14))
    assert d.found and a.found
    assert a.cost == pytest.approx(d.cost)


def test_a_star_expands_fewer_cells_than_dijkstra():
    grid = open_grid(25, 25)
    d = dijkstra(grid, (0, 0), (24, 24), connectivity=4)
    a = a_star(grid, (0, 0), (24, 24), "manhattan", connectivity=4)
    assert a.cost == pytest.approx(d.cost)
    assert a.expanded < d.expanded


def test_diagonal_move_costs_sqrt_two():
    grid = open_grid(2, 2)
    result = a_star(grid, (0, 0), (1, 1))
    assert result.path == [(0, 0), (1, 1)]
    assert result.cost == pytest.approx(math.sqrt(2))


def test_path_detours_around_wall_with_gap():
    grid = open_grid(20, 20)
    grid.blocked[0:16, 10] = True  # wall with a gap at rows 16..19
    result = a_star(grid, (2, 2), (17, 2))
    assert_valid_path(grid, result, (2, 2), (17, 2))
    assert result.cost > 15  # must climb to the gap, not walk straight
    assert any(col == 10 and row >= 16 for col, row in result.path)


def test_unreachable_goal_is_reported():
    grid = open_grid(10, 10)
    # fully enclose the goal cell
    for cell in [(7, 5), (9, 5), (8, 4), (8, 6)]:
        grid.set_blocked(cell)
    result = a_star(grid, (0, 0), (8, 5))
    assert not result.found
    assert result.path == []
    assert result.cost == math.inf
    assert result.expanded > 0


def test_start_equals_goal():
    grid = open_grid(5, 5)
    result = a_star(grid, (2, 2), (2, 2))
    assert result.found
    assert result.path == [(2, 2)]
    assert result.cost == 0
    assert result.expanded == 1


def test_goal_inside_obstacle_is_rejected():
    grid = open_grid(5, 5)
    grid.set_blocked((4, 4))
    with pytest.raises(ValueError, match="goal"):
        dijkstra(grid, (0, 0), (4, 4))


def test_start_outside_grid_is_rejected():
    with pytest.raises(ValueError, match="start"):
        dijkstra(open_grid(5, 5), (5, 5), (0, 0))


def test_corner_cutting_can_change_reachability():
    grid = OccupancyGrid(3, 3)
    grid.set_blocked((1, 0))
    grid.set_blocked((0, 1))

    blocked = a_star(grid, (0, 0), (1, 1))
    assert not blocked.found

    cutting = a_star(grid, (0, 0), (1, 1), allow_corner_cutting=True)
    assert cutting.found
    assert cutting.path == [(0, 0), (1, 1)]


def test_inflation_can_disconnect_a_corridor():
    grid = OccupancyGrid(9, 5)
    grid.blocked[:, 4] = True
    grid.set_blocked((4, 2), False)  # one-cell gap in the wall
    start, goal = (0, 2), (8, 2)

    assert dijkstra(grid, start, goal).found
    assert not dijkstra(grid.inflated(0.6), start, goal).found


def test_search_result_reports_found():
    assert SearchResult([(0, 0)], 0.0).found
    assert not SearchResult().found


def test_manhattan_with_eight_connectivity_is_rejected():
    with pytest.raises(ValueError, match="inadmissible"):
        a_star(open_grid(4, 4), (0, 0), (3, 3), "manhattan", connectivity=8)


def test_unknown_heuristic_is_rejected():
    with pytest.raises(ValueError, match="unknown heuristic"):
        a_star(open_grid(4, 4), (0, 0), (3, 3), "chebyshev")


def test_custom_heuristic_callable_is_supported():
    grid = open_grid(10, 10)
    result = a_star(grid, (0, 0), (9, 9), heuristic=euclidean)
    assert result.found
    assert result.cost == pytest.approx(octile((0, 0), (9, 9)))


def test_octile_is_exact_for_open_eight_connected_grids():
    start, goal = (0, 0), (13, 7)
    result = a_star(open_grid(20, 20), start, goal)
    assert result.cost == pytest.approx(octile(start, goal))


def test_heuristics_are_admissible_on_the_goal():
    a, b = (0, 0), (5, 3)
    assert zero(a, b) == 0
    assert manhattan(a, b) == 8
    assert euclidean(a, b) == pytest.approx(math.hypot(5, 3))
    assert octile(a, b) == pytest.approx(5 + 3 + (math.sqrt(2) - 2) * 3)
    # manhattan is only admissible for 4-connected motion, and there it is exact
    for heuristic in (euclidean, octile):
        assert heuristic(a, b) <= octile(a, b)

