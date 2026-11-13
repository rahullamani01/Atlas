"""Phase 9 experiment: detect a kidnapped robot and relocalize globally.

The robot drives normally, is teleported once without odometry observing the
move, notices that its scans no longer match the map, stops to rotate, and runs
a coarse-to-fine scan match. Several pose hypotheses are accumulated over the
rotation, the winner seeds the particle filter, and navigation resumes.
"""
from __future__ import annotations

import math
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

from atlas.evaluation import get_scenario, simulate_relocalization
from atlas.planning import OccupancyGrid, a_star

DT = 0.05
RESIDUAL_LIMIT = 0.5
RECOVERY_TOLERANCE = 0.5


def initial_path(scenario):
    """Plan the route on the known map, for plotting only."""
    world = scenario.world()
    grid = OccupancyGrid.from_world(
        world, resolution=scenario.resolution, inflate_radius=scenario.inflation
    )
    start = grid.world_to_cell(*scenario.start)
    goal = grid.world_to_cell(*scenario.goal)
    result = a_star(grid, start, goal)
    if not result.found:
        raise SystemExit("no route to plot")
    return world, grid, [grid.cell_to_world(cell) for cell in result.path]


def print_summary(trace) -> None:
    metrics = trace.metrics
    events = {event.kind: event.step for event in trace.events}
    rows = [
        ("kidnapped", "yes" if metrics.kidnapped else "no"),
        ("detection latency [s]", f"{metrics.detection_latency:.2f}"),
        ("relocalizations", f"{metrics.relocalizations}"),
        ("pose recovery [s]", f"{metrics.recovery_time:.2f}"),
        ("error at kidnap [m]", f"{metrics.error_at_kidnap:.2f}"),
        ("post-recovery RMS [m]", f"{metrics.post_recovery_rms:.3f}"),
        (
            "post-recovery heading [deg]",
            f"{metrics.post_recovery_heading_rms_deg:.2f}",
        ),
        ("final error [m]", f"{metrics.final_error:.3f}"),
        ("collisions", f"{metrics.collisions}"),
        ("goal reached", "yes" if metrics.reached else "no"),
    ]
    print(f"\n{'metric':<32}{'value':>10}")
    print("-" * 42)
    for label, value in rows:
        print(f"{label:<32}{value:>10}")
    print(
        f"\nevents: kidnap at {events.get('kidnap')}, "
        f"lost at {events.get('lost')}, "
        f"relocalized at {events.get('relocalized')}, "
        f"resumed at {events.get('resumed')}"
    )


def trajectory_index(step: int, kidnap_step: int | None, length: int) -> int:
    extra = 1 if kidnap_step is not None and step >= kidnap_step else 0
    return min(step + 1 + extra, length - 1)


def main() -> None:
    scenario = get_scenario("relocalization")
    trace = simulate_relocalization(scenario, seed=0)
    print_summary(trace)

    events = {event.kind: event.step for event in trace.events}
    kidnap_step = events["kidnap"]
    lost_step = events["lost"]
    relocalized_step = events["relocalized"]
    resumed_step = events["resumed"]

    world, grid, path = initial_path(scenario)
    trajectory = np.asarray(trace.trajectory)
    kidnap_index = kidnap_step + 1
    lost_index = trajectory_index(kidnap_step, kidnap_step, len(trajectory))
    resumed_index = trajectory_index(
        resumed_step, kidnap_step, len(trajectory)
    )

    fig = plt.figure(figsize=(16, 5.8))
    map_ax = fig.add_subplot(1, 3, 1)
    error_ax = fig.add_subplot(1, 3, 2)
    residual_ax = fig.add_subplot(1, 3, 3)

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
        *zip(*path), color="#7f7f7f", linewidth=1.2, label="planned route"
    )
    map_ax.plot(
        trajectory[:kidnap_index, 0],
        trajectory[:kidnap_index, 1],
        color="#1f77b4",
        linewidth=1.8,
        label="before kidnap",
    )
    map_ax.plot(
        [trajectory[kidnap_index - 1, 0], trajectory[kidnap_index, 0]],
        [trajectory[kidnap_index - 1, 1], trajectory[kidnap_index, 1]],
        color="#d62728",
        linestyle="--",
        linewidth=1.5,
        label="teleport",
    )
    map_ax.plot(
        trajectory[kidnap_index:resumed_index, 0],
        trajectory[kidnap_index:resumed_index, 1],
        color="#ff7f0e",
        linewidth=1.6,
        label="lost / relocalizing",
    )
    map_ax.plot(
        trajectory[resumed_index:, 0],
        trajectory[resumed_index:, 1],
        color="#2ca02c",
        linewidth=1.8,
        label="after recovery",
    )
    map_ax.plot(
        trajectory[kidnap_index, 0],
        trajectory[kidnap_index, 1],
        marker="X",
        color="#d62728",
        markersize=12,
        linestyle="none",
        label="kidnap point",
    )
    map_ax.plot(*path[0], marker="o", color="green", markersize=8)
    map_ax.plot(
        *path[-1],
        marker="*",
        color="gold",
        markeredgecolor="black",
        markersize=15,
    )
    map_ax.add_patch(patches.Rectangle(
        (scenario.kidnap_pose[0] - 0.2, scenario.kidnap_pose[1] - 0.2),
        0.4,
        0.4,
        fill=False,
        edgecolor="#d62728",
        linewidth=1.2,
    ))
    map_ax.set_xlim(0, world.width)
    map_ax.set_ylim(0, world.height)
    map_ax.set_aspect("equal")
    map_ax.set_title("Route, kidnap and recovery")
    map_ax.grid(True, alpha=0.2)
    map_ax.legend(loc="upper left", fontsize=7)

    time = np.arange(len(trace.filter_errors)) * DT
    error_ax.semilogy(time, trace.filter_errors, color="#1f77b4")
    error_ax.axhline(
        RECOVERY_TOLERANCE,
        color="#7f7f7f",
        linestyle="--",
        linewidth=1.0,
        label=f"{RECOVERY_TOLERANCE:.1f} m recovery tolerance",
    )
    for step, color, label in (
        (kidnap_step, "#d62728", "kidnap"),
        (lost_step, "#ff7f0e", "lost detected"),
        (relocalized_step, "#9467bd", "pose reacquired"),
        (resumed_step, "#2ca02c", "driving resumed"),
    ):
        error_ax.axvline(
            step * DT, color=color, linestyle=":", linewidth=1.2, label=label
        )
    error_ax.axvspan(
        lost_step * DT,
        resumed_step * DT,
        color="#ff7f0e",
        alpha=0.08,
    )
    error_ax.set_xlabel("time [s]")
    error_ax.set_ylabel("position error [m]")
    error_ax.set_title("Localization error")
    error_ax.grid(True, alpha=0.25, which="both")
    error_ax.legend(loc="lower right", fontsize=7)

    residual_ax.plot(time, trace.residuals, color="#d62728")
    residual_ax.axhline(
        RESIDUAL_LIMIT,
        color="#7f7f7f",
        linestyle="--",
        linewidth=1.0,
        label=f"{RESIDUAL_LIMIT:.1f} m monitor limit",
    )
    residual_ax.axvspan(
        lost_step * DT,
        resumed_step * DT,
        color="#ff7f0e",
        alpha=0.08,
    )
    for step, color in (
        (kidnap_step, "#d62728"),
        (lost_step, "#ff7f0e"),
        (relocalized_step, "#9467bd"),
        (resumed_step, "#2ca02c"),
    ):
        residual_ax.axvline(step * DT, color=color, linestyle=":", linewidth=1.2)
    residual_ax.set_xlabel("time [s]")
    residual_ax.set_ylabel("mean scan residual [m]")
    residual_ax.set_title("Map-consistency residual")
    residual_ax.grid(True, alpha=0.25)
    residual_ax.legend(loc="lower right", fontsize=8)

    fig.suptitle("ATLAS Phase 9 — kidnapped-robot relocalization")
    plt.tight_layout()

    out = Path(__file__).resolve().parents[1] / "experiments" / "plots"
    out.mkdir(parents=True, exist_ok=True)
    fig.savefig(out / "phase9_relocalization.png", dpi=150)
    print(f"\nsaved plot -> {out / 'phase9_relocalization.png'}")

    if matplotlib.get_backend().lower() not in {
        "agg", "pdf", "svg", "ps", "cairo"
    }:
        plt.show()


if __name__ == "__main__":
    main()
