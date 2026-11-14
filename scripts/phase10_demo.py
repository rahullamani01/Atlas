"""Phase 10 experiment: build a map and repair a wrong prior while driving.

The robot starts from a prior map that contains a phantom obstacle and omits a
real block. Scans are folded into a log-odds occupancy map at the particle-filter
estimate. Free rays erase the phantom, endpoints add the missing block, and the
planner replans when the repaired map invalidates the current route.
"""
from __future__ import annotations

from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

from atlas.evaluation import get_scenario, simulate_mapping
from atlas.planning import OccupancyGrid, a_star

DT = 0.05
MAP_ERROR_STRIDE = 10


def _route(scenario) -> list[tuple[float, float]]:
    """Plan on the prior map, for plotting the route the robot started with."""
    world = scenario.prior_world()
    grid = OccupancyGrid.from_world(
        world,
        resolution=scenario.resolution,
        inflate_radius=scenario.inflation,
    )
    start = grid.world_to_cell(*scenario.start)
    goal = grid.world_to_cell(*scenario.goal)
    result = a_star(grid, start, goal)
    if not result.found:
        raise SystemExit("no prior route to plot")
    return [grid.cell_to_world(cell) for cell in result.path]


def _centres(grid: OccupancyGrid, cells) -> tuple[list[float], list[float]]:
    xs, ys = [], []
    for col, row in cells:
        x, y = grid.cell_to_world((col, row))
        xs.append(x)
        ys.append(y)
    return xs, ys


def print_summary(trace) -> None:
    metrics = trace.metrics
    rows = [
        ("goal reached", "yes" if metrics.reached else "no"),
        ("failed", "yes" if metrics.failed else "no"),
        ("collisions", f"{metrics.collisions}"),
        ("duration [s]", f"{metrics.duration:.1f}"),
        ("replans", f"{metrics.replans}"),
        ("map updates", f"{metrics.map_updates}"),
        ("added cells", f"{metrics.added_cells}"),
        ("removed cells", f"{metrics.removed_cells}"),
        ("observed area [%]", f"{100.0 * metrics.observed_ratio:.1f}"),
        ("prior map error [cells]", f"{metrics.prior_map_error}"),
        ("final map error [cells]", f"{metrics.final_map_error}"),
        ("map error reduction [cells]", f"{metrics.map_error_reduction}"),
        ("map repair ratio", f"{metrics.map_repair_ratio:.2f}"),
        ("filter RMS [m]", f"{metrics.filter_rms:.3f}"),
        ("final error [m]", f"{metrics.final_error:.3f}"),
    ]
    print(f"\n{'metric':<32}{'value':>10}")
    print("-" * 42)
    for label, value in rows:
        print(f"{label:<32}{value:>10}")


def main() -> None:
    scenario = get_scenario("slam")
    trace = simulate_mapping(scenario, seed=0)
    print_summary(trace)

    route = _route(scenario)
    trajectory = np.asarray(trace.trajectory)
    prior = trace.prior_grid
    final = trace.final_grid
    truth = trace.truth_grid

    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    prior_ax, final_ax = axes[0]
    error_ax, filter_ax = axes[1]

    extent = (
        prior.origin[0],
        prior.origin[0] + prior.width * prior.resolution,
        prior.origin[1],
        prior.origin[1] + prior.height * prior.resolution,
    )

    # ---- prior map ----
    prior_ax.imshow(
        prior.blocked,
        origin="lower",
        extent=extent,
        cmap="Greys",
        vmin=0,
        vmax=1,
        alpha=0.75,
    )
    if route:
        prior_ax.plot(*zip(*route), color="#7f7f7f", linewidth=1.0,
                      label="prior route")
    prior_ax.plot(
        trajectory[:, 0],
        trajectory[:, 1],
        color="#1f77b4",
        linewidth=1.5,
        label="driven trajectory",
    )
    prior_ax.plot(
        trajectory[0, 0], trajectory[0, 1],
        marker="o", color="green", markersize=8, linestyle="none",
        label="start",
    )
    prior_ax.plot(*scenario.goal, marker="*", color="gold",
                  markeredgecolor="black", markersize=15, linestyle="none",
                  label="goal")
    prior_ax.set_title("Prior map (wrong: phantom present, block missing)")
    prior_ax.set_aspect("equal")
    prior_ax.set_xlim(*extent[:2])
    prior_ax.set_ylim(*extent[2:])
    prior_ax.grid(True, alpha=0.2)
    prior_ax.legend(loc="upper left", fontsize=7)

    # ---- repaired map ----
    final_ax.imshow(
        final.blocked,
        origin="lower",
        extent=extent,
        cmap="Greys",
        vmin=0,
        vmax=1,
        alpha=0.75,
    )
    prior_ax.imshow(
        np.ma.masked_where(~(truth.blocked & ~prior.blocked), truth.blocked),
        origin="lower",
        extent=extent,
        cmap="Greens",
        vmin=0,
        vmax=1,
        alpha=0.35,
    )
    final_ax.imshow(
        np.ma.masked_where(~(truth.blocked & ~prior.blocked), truth.blocked),
        origin="lower",
        extent=extent,
        cmap="Greens",
        vmin=0,
        vmax=1,
        alpha=0.35,
    )
    if trace.added_cells:
        xs, ys = _centres(final, trace.added_cells)
        final_ax.scatter(xs, ys, s=2.0, color="#2ca02c", alpha=0.5,
                         label="cells added")
    if trace.removed_cells:
        xs, ys = _centres(final, trace.removed_cells)
        final_ax.scatter(xs, ys, s=2.0, color="#d62728", alpha=0.5,
                         label="cells removed")
    final_ax.plot(
        trajectory[:, 0],
        trajectory[:, 1],
        color="#1f77b4",
        linewidth=1.5,
        label="driven trajectory",
    )
    final_ax.set_title("Repaired map (added / removed evidence)")
    final_ax.set_aspect("equal")
    final_ax.set_xlim(*extent[:2])
    final_ax.set_ylim(*extent[2:])
    final_ax.grid(True, alpha=0.2)
    final_ax.legend(loc="upper left", fontsize=7)

    replans = [event.step * DT for event in trace.events
               if event.kind == "replan"]

    # ---- map error timeline ----
    map_steps = np.arange(len(trace.map_errors)) * MAP_ERROR_STRIDE
    map_time = map_steps * DT
    error_ax.plot(map_time, trace.map_errors, color="#2ca02c",
                  marker=".", markersize=3)
    error_ax.axhline(
        trace.metrics.final_map_error,
        color="#7f7f7f",
        linestyle="--",
        linewidth=1.0,
        label="final map error",
    )
    for step in replans:
        error_ax.axvline(step, color="#d62728", alpha=0.15, linewidth=0.8)
    error_ax.set_xlabel("time [s]")
    error_ax.set_ylabel("surface error [cells]")
    error_ax.set_title("Map error over the run (observed cells only)")
    error_ax.grid(True, alpha=0.25)
    error_ax.legend(loc="upper right", fontsize=8)

    # ---- localization timeline ----
    filter_time = np.arange(len(trace.filter_errors)) * DT
    filter_ax.semilogy(filter_time, trace.filter_errors, color="#1f77b4")
    for step in replans:
        filter_ax.axvline(step, color="#d62728", alpha=0.15, linewidth=0.8)
    filter_ax.set_xlabel("time [s]")
    filter_ax.set_ylabel("position error [m]")
    filter_ax.set_title("Particle-filter error (red: replans)")
    filter_ax.grid(True, alpha=0.25, which="both")

    fig.suptitle("ATLAS Phase 10 — occupancy mapping and map repair")
    plt.tight_layout()

    out = Path(__file__).resolve().parents[1] / "experiments" / "plots"
    out.mkdir(parents=True, exist_ok=True)
    fig.savefig(out / "phase10_mapping.png", dpi=150)
    print(f"\nsaved plot -> {out / 'phase10_mapping.png'}")

    if matplotlib.get_backend().lower() not in {
        "agg", "pdf", "svg", "ps", "cairo"
    }:
        plt.show()


if __name__ == "__main__":
    main()
