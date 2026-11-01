import numpy as np
import pytest

from atlas.planning.grid import OccupancyGrid
from atlas.simulation.world import RectObstacle, World


def test_world_cell_roundtrip_uses_cell_centres():
    grid = OccupancyGrid(10, 5, resolution=0.5, origin=(1.0, 2.0))
    cell = (3, 2)
    x, y = grid.cell_to_world(cell)
    assert grid.world_to_cell(x, y) == cell
    assert (x, y) == (1.0 + 3.5 * 0.5, 2.0 + 2.5 * 0.5)


def test_world_to_cell_outside_is_reported_by_in_bounds():
    grid = OccupancyGrid(4, 4, resolution=1.0)
    assert grid.world_to_cell(-0.1, 0.0) == (-1, 0)
    assert not grid.in_bounds(grid.world_to_cell(-0.1, 0.0))
    assert grid.in_bounds(grid.world_to_cell(3.9, 3.9))


def test_out_of_bounds_cells_are_never_free():
    grid = OccupancyGrid(4, 4)
    assert not grid.is_free((-1, 0))
    assert not grid.is_free((4, 0))
    assert not grid.is_free((0, 4))


def test_set_blocked_roundtrip_and_bounds_check():
    grid = OccupancyGrid(4, 4)
    grid.set_blocked((1, 2))
    assert not grid.is_free((1, 2))
    grid.set_blocked((1, 2), False)
    assert grid.is_free((1, 2))
    with pytest.raises(ValueError):
        grid.set_blocked((4, 4))


def test_from_world_rasterises_obstacles():
    world = World(4, 4, [RectObstacle(1, 1, 2, 2)])
    grid = OccupancyGrid.from_world(world, resolution=1.0)
    assert (grid.width, grid.height) == (4, 4)
    assert not grid.is_free((1, 1))
    assert grid.is_free((0, 0))
    assert grid.is_free((3, 3))


def test_from_world_rounds_up_partial_cells():
    world = World(2.1, 1.1)
    grid = OccupancyGrid.from_world(world, resolution=0.5)
    assert (grid.width, grid.height) == (5, 3)


def test_inflation_grows_every_blocked_cell():
    grid = OccupancyGrid(7, 7, resolution=1.0)
    grid.set_blocked((3, 3))
    grown = grid.inflated(0.6)
    for cell in [(2, 3), (4, 3), (3, 2), (3, 4)]:
        assert not grown.is_free(cell)
    assert grown.is_free((2, 2))  # diagonal is farther than the radius
    assert grid.is_free((2, 3))  # original is unchanged


def test_inflation_of_zero_is_a_copy():
    grid = OccupancyGrid(3, 3)
    grid.set_blocked((1, 1))
    grown = grid.inflated(0.0)
    assert np.array_equal(grown.blocked, grid.blocked)
    grown.set_blocked((0, 0))
    assert grid.is_free((0, 0))


def test_inflation_closes_a_narrow_corridor():
    grid = OccupancyGrid(9, 5)
    grid.blocked[:, 4] = True
    grown = grid.inflated(0.6)
    assert grid.is_free((3, 2))
    assert not grown.is_free((3, 2))
    assert not grown.is_free((5, 2))


def test_negative_inflation_is_rejected():
    with pytest.raises(ValueError):
        OccupancyGrid(3, 3).inflated(-0.1)


def test_free_ratio():
    grid = OccupancyGrid(4, 4)
    grid.set_blocked((0, 0))
    grid.set_blocked((1, 1))
    assert grid.free_count() == 14
    assert grid.free_ratio() == pytest.approx(14 / 16)


def test_four_connected_neighbours():
    grid = OccupancyGrid(5, 5)
    assert len(list(grid.neighbors((2, 2), connectivity=4))) == 4
    assert len(list(grid.neighbors((0, 0), connectivity=4))) == 2


def test_eight_connected_neighbours_include_diagonals():
    grid = OccupancyGrid(5, 5)
    neighbours = {cell for cell, cost in grid.neighbors((2, 2), connectivity=8)}
    assert len(neighbours) == 8
    assert (3, 3) in neighbours


def test_diagonal_neighbour_cost_is_sqrt_two():
    grid = OccupancyGrid(5, 5)
    costs = dict(grid.neighbors((0, 0), connectivity=8))
    assert costs[(1, 1)] == pytest.approx(2 ** 0.5)


def test_corner_cutting_is_rejected_by_default():
    grid = OccupancyGrid(3, 3)
    grid.set_blocked((2, 1))
    neighbours = {cell for cell, _ in grid.neighbors((1, 1), connectivity=8)}
    assert (2, 2) not in neighbours

    cutting = {
        cell
        for cell, _ in grid.neighbors(
            (1, 1), connectivity=8, allow_corner_cutting=True
        )
    }
    assert (2, 2) in cutting


def test_invalid_neighbourhood_is_rejected():
    grid = OccupancyGrid(3, 3)
    with pytest.raises(ValueError):
        list(grid.neighbors((1, 1), connectivity=6))


def test_invalid_grid_parameters():
    with pytest.raises(ValueError):
        OccupancyGrid(0, 3)
    with pytest.raises(ValueError):
        OccupancyGrid(3, 3, resolution=0)
    with pytest.raises(ValueError):
        OccupancyGrid(3, 3, blocked=np.zeros((2, 3), dtype=bool))
