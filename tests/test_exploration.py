"""Tests for next-best-view exploration (Phase 13)."""
import math
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from atlas.evaluation import (
    ExplorationMetrics,
    get_scenario,
    simulate_exploration,
)
from atlas.evaluation.scenarios import demo_world, empty_world
from atlas.exploration import (
    candidate_cells,
    select_viewpoint,
    unknown_area,
)
from atlas.planning import OccupancyGrid


def test_explore_scenario_starts_from_an_empty_map():
    scenario = get_scenario("explore")

    assert scenario.kind == "exploration"
    assert scenario.world is demo_world
    assert scenario.prior_world is empty_world
    assert scenario.prior_world().obstacles == []
    assert len(scenario.world().obstacles) > 0
    assert scenario.coverage_target == 0.95
    assert scenario.inflation == 0.30


def test_unknown_area_counts_only_the_disk():
    unknown = np.zeros((5, 5), dtype=bool)
    unknown[2, 2] = True

    assert unknown_area(unknown, (2, 2), radius=0) == 1
    assert unknown_area(unknown, (2, 2), radius=1) == 1
    assert unknown_area(unknown, (0, 0), radius=4) == 1
    assert unknown_area(unknown, (4, 4), radius=1) == 0
    assert unknown_area(unknown, (-3, 2), radius=1) == 0


def test_candidate_cells_skip_blocked_and_excluded_points():
    grid = OccupancyGrid(24, 24, resolution=1.0)
    grid.set_blocked((12, 4))

    cells = candidate_cells(grid, spacing=8, start=(4, 4))
    assert (4, 4) not in cells
    assert (12, 4) not in cells
    assert (20, 4) in cells

    excluded = candidate_cells(
        grid, spacing=8, visited=((4, 12),), blacklist=((20, 20),)
    )
    assert all(
        math.hypot(col - 4, row - 12) >= 8 for col, row in excluded
    )
    assert all(
        math.hypot(col - 20, row - 20) >= 8 for col, row in excluded
    )


def test_select_viewpoint_prefers_the_unknown_region():
    grid = OccupancyGrid(24, 24, resolution=1.0)
    known = np.ones((24, 24), dtype=bool)
    known[2:6, 18:22] = False
    mapping = SimpleNamespace(known=known)

    viewpoint = select_viewpoint(
        mapping, grid, (4, 4), spacing=8, radius=4, min_gain=1
    )

    assert viewpoint is not None
    assert viewpoint.cell == (20, 4)
    assert viewpoint.gain == 16
    assert viewpoint.cost == pytest.approx(16.0)
    assert viewpoint.score == pytest.approx(16.0 / 17.0)


def test_select_viewpoint_returns_none_when_nothing_is_unknown():
    grid = OccupancyGrid(24, 24, resolution=1.0)
    mapping = SimpleNamespace(known=np.ones((24, 24), dtype=bool))

    assert select_viewpoint(mapping, grid, (4, 4), min_gain=1) is None


def _short_scenario(**overrides):
    scenario = replace(
        get_scenario("explore"),
        max_steps=400,
        coverage_target=0.95,
    )
    return replace(scenario, **overrides)


def test_exploration_reaches_a_coverage_target_and_stops():
    scenario = _short_scenario(coverage_target=0.18, max_steps=200)
    trace = simulate_exploration(
        scenario, seed=0, particles=100, beams=24
    )
    metrics = trace.metrics

    assert metrics.completed
    assert not metrics.failed
    assert metrics.collisions == 0
    assert metrics.coverage >= scenario.coverage_target
    assert metrics.viewpoints >= 1
    assert metrics.steps < scenario.max_steps

    assert len(trace.trajectory) == len(trace.filter_errors) + 1
    assert len(trace.heading_errors) == len(trace.filter_errors)
    assert trace.coverage_history
    assert metrics.coverage > trace.coverage_history[0][1]
    assert trace.final_grid.blocked.shape == trace.truth_grid.blocked.shape
    assert trace.prior_grid.blocked.shape == trace.truth_grid.blocked.shape


def test_exploration_stops_at_the_step_budget_without_failing():
    scenario = _short_scenario()
    trace = simulate_exploration(
        scenario, seed=0, particles=100, beams=24
    )

    assert not trace.metrics.completed
    assert not trace.metrics.failed
    assert trace.metrics.collisions == 0
    assert trace.metrics.coverage > trace.coverage_history[0][1]
    assert trace.metrics.steps == scenario.max_steps
    assert trace.metrics.viewpoints >= 2


def test_exploration_is_reproducible_for_a_seed():
    scenario = _short_scenario(coverage_target=0.18, max_steps=200)
    first = simulate_exploration(scenario, seed=3, particles=100, beams=24)
    second = simulate_exploration(scenario, seed=3, particles=100, beams=24)

    assert first.metrics == second.metrics
    assert first.trajectory == second.trajectory
    assert first.viewpoints == second.viewpoints


def test_exploration_metrics_expose_a_flat_dictionary():
    metrics = ExplorationMetrics(
        completed=True,
        failed=False,
        collisions=0,
        duration=112.1,
        coverage=0.95,
        free_coverage=0.961,
        map_error=320,
        viewpoints=19,
        steps=2242,
        filter_rms=0.093,
        filter_heading_rms_deg=1.15,
    ).as_dict()

    assert metrics["completed"] == 1.0
    assert metrics["failed"] == 0.0
    assert metrics["coverage"] == 0.95
    assert metrics["viewpoints"] == 19
    assert metrics["steps"] == 2242
