import math

import numpy as np
import pytest

from atlas.localization import DistanceField
from atlas.planning import OccupancyGrid
from atlas.simulation.world import RectObstacle, World


def make_grid(resolution=0.5):
    world = World(10, 10, [RectObstacle(4, 4, 6, 6)])
    return OccupancyGrid.from_world(world, resolution=resolution, inflate_radius=0.0)


def test_field_preserves_grid_geometry():
    grid = make_grid()
    field = DistanceField.from_grid(grid)
    assert field.resolution == grid.resolution
    assert field.origin == grid.origin
    assert field.distances.shape == grid.blocked.shape


def test_distance_increases_away_from_an_obstacle():
    field = DistanceField.from_grid(make_grid(), include_boundary=False)
    near = field.distance_at(3.5, 5.0)   # one cell left of the block
    far = field.distance_at(1.0, 5.0)
    assert near < far


def test_boundary_cells_are_zero_distance():
    field = DistanceField.from_grid(make_grid(), include_boundary=False)
    assert field.distance_at(4.25, 4.75) == pytest.approx(0.0)


def test_world_wall_distance_is_included_by_default():
    field = DistanceField.from_grid(make_grid(), include_boundary=True)
    # 0.25 m from the west wall, far from the central block
    assert field.distance_at(0.25, 1.0) == pytest.approx(0.25)


def test_world_wall_can_be_excluded():
    field = DistanceField.from_grid(make_grid(), include_boundary=False)
    assert field.distance_at(0.25, 1.0) > 2.0


def test_boundary_only_penalises_obstacle_interiors():
    grid = make_grid()
    solid = DistanceField.from_grid(grid, include_boundary=False, boundary_only=False)
    shell = DistanceField.from_grid(grid, include_boundary=False, boundary_only=True)
    assert solid.distance_at(5.0, 5.0) == pytest.approx(0.0)  # blocked cell
    assert shell.distance_at(5.0, 5.0) >= 0.5                 # interior, not a source


def test_sample_matches_distance_at_at_cell_centres():
    field = DistanceField.from_grid(make_grid())
    points = np.array([[0.25, 0.25], [1.25, 2.75], [5.25, 5.25]])
    vectorised = field.sample(points)
    direct = np.array([field.distance_at(x, y) for x, y in points])
    assert np.allclose(vectorised, direct)


def test_sample_accepts_a_single_point():
    field = DistanceField.from_grid(make_grid())
    value = field.sample(np.array([2.25, 3.25]))
    assert value.shape == (1,)
    assert value[0] == pytest.approx(field.distance_at(2.25, 3.25))


def test_points_outside_the_grid_are_infinite():
    field = DistanceField.from_grid(make_grid())
    assert field.distance_at(-1.0, 5.0) == math.inf
    assert np.isinf(field.sample(np.array([[-1.0, 5.0]]))).all()


def test_distances_are_non_negative_and_finite_inside():
    field = DistanceField.from_grid(make_grid())
    values = field.sample(np.array([[0.1, 0.1], [9.9, 9.9], [5.0, 1.0]]))
    assert np.all(values >= 0)
    assert np.all(np.isfinite(values))
