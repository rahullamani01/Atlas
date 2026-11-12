"""Phase 8 experiment: run the navigation loop on the estimated pose.

Phase 6 used ground-truth pose to isolate perception and recovery. This demo
closes the loop: the particle-filter estimate drives the supervisor, and the
same estimate interprets LiDAR returns against the dynamic map. A ground-truth
recovery run is printed beside it to show the cost of estimation.
"""
from __future__ import annotations

import math
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

from atlas.evaluation import get_scenario, run_recovery, simulate_integrated
from atlas.planning import OccupancyGrid, a_star

DT = 0.05


def initial_path(scenario):
    """Plan the scenario's route on its known map, for plotting only."""
    world = scenario.world()
    grid = OccupancyGrid.from_world(
        world, resolution=scenario.resolution, inflate_radius=scenario.inflation
    )
    start = grid.world_to_cell(*scenario.start)
    goal = grid.world_to_cell(*scenario.goal)
    result = a_star(grid, start, goal)
    if not result.found:
        raise SystemExit("no route to plot")
    return world, grid, goal, [grid.cell_to_world(cell) for cell in result.path]


def print_comparison(ground_truth, estimate):
    rows = [
        ("reached goal", lambda m: "yes" if m["reached"] else "no"),
        ("collisions", lambda m: f"{int(m['collisions'])}"),
        ("duration [s]", lambda m: f"{m['duration']:.2f}"),
        ("final error [m]", lambda m: f"{m['final_error']:.2f}"),
        ("detected cells", lambda m: f"{int(m['detected_cells'])}"),
        ("replans", lambda m: f"{int(m['replans'])}"),
    ]
    print(f"\n{'metric':<20}{'ground truth':>16}{'estimate':>14}")
    print("-" * 50)
    for label, getter in rows:
        print(f"{label:<20}{getter(ground_truth):>16}{getter(estimate):>14}")
    print(
        f"\nfilter position RMS {estimate['filter_rms']:.3f} m "
        f"vs odometry {estimate['odometry_rms']:.3f} m"
    )
    print(
        f"filter heading RMS {estimate['filter_heading_rms_deg']:.2f} deg"
    )


def main() -> None:
    integrated_scenario = get_scenario("integrated")
    ground_truth_scenario = get_scenario("recovery")

    trace = simulate_integrated(integrated_scenario, seed=0)
    ground_truth = run_recovery(ground_truth_scenario, seed=0).metrics
    estimate = trace.metrics.as_dict()
    print_comparison(ground_truth, estimate)

    world, grid, goal_cell, path = initial_path(integrated_scenario)
    trajectory = np.asarray(trace.trajectory)
    replan_times = [
        event.step * DT for event in trace.events if event.kind == "replan"
    ]

    fig = plt.figure(figsize=(16, 5.6))
    map_ax = fig.add_subplot(1, 3, 1)
    error_ax = fig.add_subplot(1, 3, 2)
    detect_ax = fig.add_subplot(1, 3, 3)

    extent = (0, world.width, 0, world.height)
    map_ax.imshow(
        grid.blocked,
        origin="lower",
        extent=extent,
        cmap="Greys",
        vmin=0,
        vmax=1,
        alpha=0.75,
    )
    map_ax.plot(
        *zip(*path), color="#7f7f7f", linewidth=1.2, label="initial path"
    )
    map_ax.plot(
        trajectory[:, 0],
        trajectory[:, 1],
        color="#1f77b4",
        linewidth=1.9,
        label="estimate-driven",
    )
    if trace.detected_cells:
        cells = [grid.cell_to_world(cell) for cell in trace.detected_cells]
        map_ax.scatter(
            *zip(*cells),
            marker="s",
            s=14,
            color="#d62728",
            alpha=0.55,
            label="detected obstacle",
        )
    block = integrated_scenario.inject_rect
    map_ax.add_patch(patches.Rectangle(
        (block[0], block[1]),
        block[2] - block[0],
        block[3] - block[1],
        fill=False,
        edgecolor="#d62728",
        linewidth=2.0,
        label="new block",
    ))
    map_ax.plot(*path[0], marker="o", color="green", markersize=8)
    map_ax.plot(
        *path[-1],
        marker="*",
        color="gold",
        markeredgecolor="black",
        markersize=15,
    )
    map_ax.set_xlim(0, world.width)
    map_ax.set_ylim(0, world.height)
    map_ax.set_aspect("equal")
    map_ax.set_title("Route and detour")
    map_ax.grid(True, alpha=0.2)
    map_ax.legend(loc="upper left", fontsize=8)

    time = np.arange(len(trace.filter_errors)) * DT
    error_ax.semilogy(
        time,
        trace.odometry_errors,
        color="#ff7f0e",
        label=f"odometry: RMS {estimate['odometry_rms']:.2f} m",
    )
    error_ax.semilogy(
        time,
        trace.filter_errors,
        color="#1f77b4",
        label=f"particle filter: RMS {estimate['filter_rms']:.3f} m",
    )
    if integrated_scenario.inject_step is not None:
        error_ax.axvline(
            integrated_scenario.inject_step * DT,
            color="#d62728",
            linestyle=":",
            label="obstacle injected",
        )
    for index, when in enumerate(replan_times):
        error_ax.axvline(
            when,
            color="#2ca02c",
            linestyle=":",
            alpha=0.8,
            label="replanned" if index == 0 else None,
        )
    error_ax.set_xlabel("time [s]")
    error_ax.set_ylabel("position error [m]")
    error_ax.set_title("Localization error")
    error_ax.grid(True, alpha=0.25, which="both")
    error_ax.legend(loc="lower right", fontsize=8)

    detect_time = np.arange(len(trace.detected_counts)) * DT
    detect_ax.step(
        detect_time,
        trace.detected_counts,
        where="post",
        color="#d62728",
    )
    if integrated_scenario.inject_step is not None:
        detect_ax.axvline(
            integrated_scenario.inject_step * DT,
            color="#d62728",
            linestyle=":",
        )
    for when in replan_times:
        detect_ax.axvline(when, color="#2ca02c", linestyle=":", alpha=0.8)
    detect_ax.set_xlabel("time [s]")
    detect_ax.set_ylabel("detected cells")
    detect_ax.set_title("Dynamic obstacle map growth")
    detect_ax.grid(True, alpha=0.25)

    fig.suptitle(
        "ATLAS Phase 8 — integrated autonomy on the estimated pose"
    )
    plt.tight_layout()

    out = Path(__file__).resolve().parents[1] / "experiments" / "plots"
    out.mkdir(parents=True, exist_ok=True)
    fig.savefig(out / "phase8_integrated.png", dpi=150)
    print(f"\nsaved plot -> {out / 'phase8_integrated.png'}")

    if matplotlib.get_backend().lower() not in {"agg", "pdf", "svg", "ps", "cairo"}:
        plt.show()


if __name__ == "__main__":
    main()
