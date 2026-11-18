"""Phase 13 experiment: explore an unknown map with next-best views.

The robot starts with an empty prior, so every cell is unknown and there is no
goal to plan to. It scores a coarse lattice of candidate viewpoints by the
unknown area within LiDAR range divided by the A* travel cost, drives to the
best one, and re-scores after each arrival. Unknown cells are free to the
planner, so the robot is willing to leave the observed region; failing targets
are blacklisted and replaced. The run stops when the observed fraction of the
map reaches the scenario's coverage target.
"""
from __future__ import annotations

from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

from atlas.evaluation import get_scenario, simulate_exploration


def print_summary(trace) -> None:
    metrics = trace.metrics
    rows = [
        ("completed", "yes" if metrics.completed else "no"),
        ("failed", "yes" if metrics.failed else "no"),
        ("collisions", f"{metrics.collisions}"),
        ("duration [s]", f"{metrics.duration:.1f}"),
        ("map coverage", f"{100.0 * metrics.coverage:.1f}%"),
        ("free-space coverage", f"{100.0 * metrics.free_coverage:.1f}%"),
        ("viewpoints", f"{metrics.viewpoints}"),
        ("steps", f"{metrics.steps}"),
        ("map error [cells]", f"{metrics.map_error}"),
        ("filter RMS [m]", f"{metrics.filter_rms:.3f}"),
        ("filter heading RMS [deg]", f"{metrics.filter_heading_rms_deg:.2f}"),
    ]
    print(f"\n{'metric':<30}{'value':>10}")
    print("-" * 40)
    for label, value in rows:
        print(f"{label:<30}{value:>10}")


def _extent(grid) -> tuple[float, float, float, float]:
    return (
        grid.origin[0],
        grid.origin[0] + grid.width * grid.resolution,
        grid.origin[1],
        grid.origin[1] + grid.height * grid.resolution,
    )


def _draw_map(axis, grid, title: str) -> None:
    extent = _extent(grid)
    axis.imshow(
        grid.blocked, origin="lower", extent=extent, cmap="Greys",
        vmin=0, vmax=1, alpha=0.8,
    )
    axis.set_title(title)
    axis.set_aspect("equal")
    axis.set_xlim(*extent[:2])
    axis.set_ylim(*extent[2:])
    axis.grid(True, alpha=0.2)


def main() -> None:
    scenario = get_scenario("explore")
    trace = simulate_exploration(scenario, seed=0)
    print_summary(trace)

    metrics = trace.metrics
    trajectory = np.asarray(trace.trajectory)
    coverage = np.asarray(trace.coverage_history)
    free_coverage = np.asarray(trace.truth_coverage_history)
    elapsed = np.arange(len(trace.filter_errors)) * scenario.dt
    viewpoints = np.asarray([
        trace.final_grid.cell_to_world((int(col), int(row)))
        for col, row in trace.viewpoints
    ])

    fig, axes = plt.subplots(2, 3, figsize=(19, 11))
    prior_ax, truth_ax, final_ax = axes[0]
    coverage_ax, error_ax, route_ax = axes[1]

    # ---- maps ----
    _draw_map(prior_ax, trace.prior_grid, "Prior map: every cell unknown")
    prior_ax.text(
        0.5, 0.5, "no prior knowledge", transform=prior_ax.transAxes,
        ha="center", va="center", fontsize=14, color="#777777",
    )
    _draw_map(truth_ax, trace.truth_grid, "True world (scoring only)")
    _draw_map(
        final_ax, trace.final_grid,
        f"Map after {metrics.steps} steps",
    )
    final_ax.plot(
        trajectory[:, 0], trajectory[:, 1], color="#1f77b4",
        linewidth=1.0, alpha=0.8, label="driven path",
    )
    if len(viewpoints):
        final_ax.plot(
            viewpoints[:, 0], viewpoints[:, 1], marker="x",
            color="#d62728", markersize=6, linestyle="none",
            label="viewpoints",
        )
    final_ax.plot(
        trajectory[0, 0], trajectory[0, 1], marker="o", color="green",
        markersize=8, linestyle="none", label="start",
    )
    final_ax.legend(loc="upper right", fontsize=7)
    final_ax.set_xlabel(
        f"map error vs truth: {metrics.map_error} cells"
    )

    # ---- coverage ----
    coverage_ax.plot(
        coverage[:, 0] * scenario.dt, 100.0 * coverage[:, 1],
        color="#1f77b4", linewidth=1.6,
        label="observed map fraction (drives stopping)",
    )
    coverage_ax.plot(
        free_coverage[:, 0] * scenario.dt, 100.0 * free_coverage[:, 1],
        color="#2ca02c", linewidth=1.6, linestyle="--",
        label="free-space coverage (scoring)",
    )
    coverage_ax.axhline(
        100.0 * scenario.coverage_target, color="#d62728",
        linestyle=":", linewidth=1.2, label="coverage target",
    )
    coverage_ax.set_xlabel("time [s]")
    coverage_ax.set_ylabel("coverage [%]")
    coverage_ax.set_title("Coverage grows until the target is reached")
    coverage_ax.grid(True, alpha=0.25)
    coverage_ax.legend(loc="lower right", fontsize=7)

    # ---- filter error ----
    errors = np.asarray(trace.filter_errors)
    headings = np.asarray(trace.heading_errors)
    error_ax.plot(
        elapsed, 100.0 * errors, color="#1f77b4", linewidth=1.1,
        label="position error [cm]",
    )
    error_ax.set_xlabel("time [s]")
    error_ax.set_ylabel("position error [cm]")
    error_ax.set_title("Localization error during exploration")
    error_ax.grid(True, alpha=0.25)
    heading_ax = error_ax.twinx()
    heading_ax.plot(
        elapsed, np.degrees(headings), color="#d62728", linewidth=0.9,
        alpha=0.8, label="heading error [deg]",
    )
    heading_ax.set_ylabel("heading error [deg]")
    lines = error_ax.get_lines() + heading_ax.get_lines()
    error_ax.legend(
        lines, [line.get_label() for line in lines],
        loc="upper right", fontsize=7,
    )

    # ---- trajectory by time ----
    points = route_ax.scatter(
        trajectory[1:, 0], trajectory[1:, 1], c=elapsed,
        cmap="viridis", s=4, label="driven path",
    )
    route_ax.plot(
        viewpoints[:, 0], viewpoints[:, 1], marker="x", color="#d62728",
        markersize=7, linestyle="none", label="viewpoints",
    )
    route_ax.set_title("Trajectory, coloured by time")
    route_ax.set_aspect("equal")
    route_ax.grid(True, alpha=0.2)
    route_ax.legend(loc="upper right", fontsize=7)
    colorbar = fig.colorbar(points, ax=route_ax, fraction=0.046, pad=0.04)
    colorbar.set_label("time [s]")

    fig.suptitle(
        "ATLAS Phase 13 — next-best-view exploration of an unknown map"
    )
    plt.tight_layout()

    out = Path(__file__).resolve().parents[1] / "experiments" / "plots"
    out.mkdir(parents=True, exist_ok=True)
    path = out / "phase13_exploration.png"
    fig.savefig(path, dpi=150)
    print(f"\nsaved plot -> {path}")

    if matplotlib.get_backend().lower() not in {
        "agg", "pdf", "svg", "ps", "cairo"
    }:
        plt.show()


if __name__ == "__main__":
    main()
