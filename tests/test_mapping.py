"""Unit tests for occupancy mapping and map repair (Phase 10)."""
import math

import numpy as np
import pytest

from atlas.localization import ParticleFilter
from atlas.mapping import (
    LogOddsMap,
    MappingNavigator,
    map_error,
    route_length,
    surface_cells,
)
from atlas.planning import OccupancyGrid
from atlas.robot.differential_drive import Pose
from atlas.sensors.lidar import LidarScan


def _prior(cells=()):
    grid = OccupancyGrid(8, 8, 1.0, (0.0, 0.0))
    for cell in cells:
        grid.set_blocked(cell)
    return grid


def _beam(origin, angle, distance, max_range=10.0):
    """A one-ray scan for a deterministic mapping update."""
    return LidarScan(
        ranges=np.array([distance], dtype=float),
        angles=np.array([angle], dtype=float),
        world_angles=np.array([angle], dtype=float),
        origin=(origin[0], origin[1]),
        max_range=max_range,
    )


def test_log_odds_map_starts_from_the_prior():
    prior = _prior([(2, 2)])
    mapping = LogOddsMap(prior)

    assert np.array_equal(mapping.occupancy, prior.blocked)
    assert mapping.known_ratio == 1.0
    assert mapping.observed_ratio == 0.0
    assert mapping.log_odds[2, 2] == pytest.approx(1.5)
    assert mapping.log_odds[0, 0] == pytest.approx(-0.75)


def test_unknown_cells_are_free_and_unobserved():
    known = np.zeros((8, 8), dtype=bool)
    mapping = LogOddsMap(_prior([(2, 2)]), known=known)

    assert not mapping.occupancy.any()
    assert mapping.known_ratio == 0.0
    assert mapping.log_odds[2, 2] == 0.0
    assert mapping.to_grid(0.0).free_count() == 64


def test_free_rays_remove_a_phantom_obstacle():
    mapping = LogOddsMap(_prior([(2, 2)]))
    scan = _beam((0.5, 0.5), math.pi / 4.0, 10.0)

    for _ in range(3):
        update = mapping.integrate(scan, Pose(0.5, 0.5, 0.0))
        assert (2, 2) not in update.removed

    update = mapping.integrate(scan, Pose(0.5, 0.5, 0.0))

    assert (2, 2) in update.removed
    assert not mapping.occupancy[2, 2]
    assert mapping.log_odds[2, 2] < 0.0


def test_endpoint_hits_add_a_missing_obstacle():
    mapping = LogOddsMap(_prior())
    scan = _beam((0.5, 0.5), 0.0, 2.5)

    update = mapping.integrate(scan, Pose(0.5, 0.5, 0.0))

    assert (3, 0) in update.added
    assert mapping.occupancy[0, 3]
    assert mapping.log_odds[0, 3] > 0.0


def test_surface_cells_returns_the_boundary_not_the_interior():
    blocked = np.zeros((5, 5), dtype=bool)
    blocked[1:4, 1:4] = True

    surface = surface_cells(blocked)

    assert np.count_nonzero(surface) == 8
    assert not surface[2, 2]
    assert surface[1, 1]
    assert surface[2, 1]


def test_map_error_compares_surfaces_and_honours_the_mask():
    truth = np.zeros((5, 5), dtype=bool)
    truth[1:4, 1:4] = True
    empty = np.zeros((5, 5), dtype=bool)

    assert map_error(truth, truth) == 0
    assert map_error(empty, truth) == 8

    mask = np.zeros((5, 5), dtype=bool)
    mask[1, 1] = True
    assert map_error(empty, truth, mask) == 1
    assert map_error(truth, truth, mask) == 0


@pytest.mark.parametrize(
    "kwargs, message",
    [
        ({"hit_log_odds": 0.0}, "hit_log_odds"),
        ({"miss_log_odds": 0.0}, "miss_log_odds"),
        ({"min_log_odds": 1.0, "max_log_odds": 0.0}, "min_log_odds"),
        ({"occupied_threshold": 5.0}, "occupied_threshold"),
        ({"max_range": 0.0}, "max_range"),
        ({"max_beams": 0}, "max_beams"),
        ({"free_margin": -1.0}, "free_margin"),
        ({"known": np.zeros((2, 2), dtype=bool)}, "shape"),
    ],
)
def test_log_odds_map_rejects_invalid_configuration(kwargs, message):
    with pytest.raises(ValueError, match=message):
        LogOddsMap(_prior(), **kwargs)


def test_mapping_navigator_plans_and_keeps_a_path_on_replan_failure():
    mapping = LogOddsMap(_prior())
    navigator = MappingNavigator(mapping, goal_cell=(6, 6), inflation=0.0)

    assert not navigator.goal_reached
    assert navigator.plan(Pose(1.5, 1.5, 0.0))
    assert navigator.path
    assert navigator.plan_count == 1

    before = list(navigator.path)
    mapping.log_odds[6, 6] = 1.0
    navigator.map_version += 1

    assert not navigator.replan(Pose(1.5, 1.5, 0.0))
    assert navigator.path == before


def test_mapping_navigator_integrates_scans_into_the_map():
    mapping = LogOddsMap(_prior())
    navigator = MappingNavigator(mapping, goal_cell=(6, 6), inflation=0.0)
    assert navigator.plan(Pose(1.5, 1.5, 0.0))

    scan = _beam((1.5, 1.5), 0.0, 2.5)
    navigator.step(Pose(1.5, 1.5, 0.0), scan, 0.05)

    assert navigator.map_updates == 1
    assert navigator.map_version == 1
    assert (4, 1) in navigator.added_cells


def test_mapping_navigator_rejects_invalid_configuration():
    mapping = LogOddsMap(_prior())
    with pytest.raises(ValueError, match="inflation"):
        MappingNavigator(mapping, (5, 5), inflation=-0.1)
    with pytest.raises(ValueError, match="replan_interval"):
        MappingNavigator(mapping, (5, 5), replan_interval=0)


def test_route_length_sums_segments():
    path = [(0.0, 0.0), (3.0, 4.0), (3.0, 6.0)]
    assert route_length(path) == pytest.approx(7.0)
    assert route_length([]) == 0.0
    assert route_length([(1.0, 1.0)]) == 0.0


def test_replace_grid_rebuilds_the_likelihood_field():
    open_grid = OccupancyGrid(8, 8, 1.0, (0.0, 0.0))
    pf = ParticleFilter(open_grid, n_particles=20, seed=0)
    before = pf.field.distance_at(4.5, 4.5)

    blocked = OccupancyGrid(8, 8, 1.0, (0.0, 0.0))
    blocked.set_blocked((4, 4))
    pf.replace_grid(blocked)

    assert pf.grid is blocked
    assert pf.field.distance_at(4.5, 4.5) == pytest.approx(0.0)
    assert pf.field.distance_at(4.5, 4.5) < before
