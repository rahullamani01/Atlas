"""Phase 11 experiment: close a loop and correct the odometry trajectory.

The robot drives a route that returns near its start. Wheel odometry drifts
along the way, so a map built from those poses smears. The back-end builds a
pose graph on the odometry chain, verifies loop closures by scan matching
against the prior map, optimises the graph, and rebuilds the map from the
corrected poses.
"""
from __future__ import annotations

from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

from atlas.evaluation import get_scenario, simulate_slam


def print_summary(trace) -> None:
    metrics = trace.metrics
    rows = [
        ("goal reached", "yes" if metrics.reached else "no"),
        ("failed", "yes" if metrics.failed else "no"),
        ("collisions", f"{metrics.collisions}"),
        ("duration [s]", f"{metrics.duration:.1f}"),
        ("graph nodes", f"{metrics.nodes}"),
        ("loop closures", f"{metrics.loop_closures}"),
        ("odometry RMS [m]", f"{metrics.odometry_rms:.3f}"),
        ("slam RMS [m]", f"{metrics.slam_rms:.3f}"),
        ("slam final [m]", f"{metrics.slam_final:.3f}"),
        ("ATE reduction", f"{100.0 * metrics.ate_reduction:.1f}%"),
        ("graph cost before", f"{metrics.graph_cost_before:.2f}"),
        ("graph cost after", f"{metrics.graph_cost_after:.4f}"),
        ("prior map error [cells]", f"{metrics.prior_map_error}"),
        ("raw map error [cells]", f"{metrics.raw_map_error}"),
        ("slam map error [cells]", f"{metrics.slam_map_error}"),
        ("map error reduction [cells]", f"{metrics.map_error_reduction}"),
        ("filter RMS [m]", f"{metrics.filter_rms:.3f}"),
        ("final error [m]", f"{metrics.final_error:.3f}"),
    ]
    print(f"\n{'metric':<32}{'value':>10}")
    print("-" * 42)
    for label, value in rows:
        print(f"{label:<32}{value:>10}")


def _xy(poses) -> tuple[list[float], list[float]]:
    return [pose.x for pose in poses], [pose.y for pose in poses]


def main() -> None:
    scenario = get_scenario("loop")
    trace = simulate_slam(scenario, seed=0)
    print_summary(trace)

    trajectory = np.asarray(trace.trajectory)
    raw = trace.node_raw
    corrected = trace.node_corrected
    truth = trace.node_truth
    raw_errors = [
        float(np.hypot(pose.x - true.x, pose.y - true.y))
        for pose, true in zip(raw, truth)
    ]
    slam_errors = [
        float(np.hypot(pose.x - true.x, pose.y - true.y))
        for pose, true in zip(corrected, truth)
    ]

    fig, axes = plt.subplots(2, 2, figsize=(15, 11))
    route_ax, error_ax = axes[0]
    map_ax, cost_ax = axes[1]

    # ---- trajectories ----
    route_ax.plot(
        trajectory[:, 0], trajectory[:, 1],
        color="#1f77b4", linewidth=1.5, label="driven (truth)",
    )
    raw_x, raw_y = _xy(raw)
    corr_x, corr_y = _xy(corrected)
    route_ax.plot(
        raw_x, raw_y, color="#d62728", linewidth=1.0, linestyle="--",
        label="odometry nodes",
    )
    route_ax.plot(
        corr_x, corr_y, color="#2ca02c", linewidth=1.2,
        label="pose-graph nodes",
    )
    for loop in trace.loops:
        route_ax.plot(
            [corrected[loop.old].x, corrected[loop.node].x],
            [corrected[loop.old].y, corrected[loop.node].y],
            color="#9467bd", linewidth=0.6, alpha=0.35,
        )
    route_ax.plot(
        trajectory[0, 0], trajectory[0, 1], marker="o", color="green",
        markersize=8, linestyle="none", label="start",
    )
    route_ax.plot(
        *scenario.goal, marker="*", color="gold", markeredgecolor="black",
        markersize=15, linestyle="none", label="goal",
    )
    route_ax.set_title("Trajectory: odometry drift vs corrected graph")
    route_ax.set_aspect("equal")
    route_ax.grid(True, alpha=0.2)
    route_ax.legend(loc="best", fontsize=7)

    # ---- node error ----
    index = np.arange(len(raw_errors))
    error_ax.plot(index, raw_errors, color="#d62728", marker=".",
                  markersize=3, label="odometry")
    error_ax.plot(index, slam_errors, color="#2ca02c", marker=".",
                  markersize=3, label="pose graph")
    error_ax.set_xlabel("node index")
    error_ax.set_ylabel("position error [m]")
    error_ax.set_title("Absolute trajectory error per node")
    error_ax.grid(True, alpha=0.25)
    error_ax.legend(loc="upper left", fontsize=8)

    # ---- corrected map ----
    grid = trace.slam_grid
    extent = (
        grid.origin[0],
        grid.origin[0] + grid.width * grid.resolution,
        grid.origin[1],
        grid.origin[1] + grid.height * grid.resolution,
    )
    map_ax.imshow(
        grid.blocked, origin="lower", extent=extent, cmap="Greys",
        vmin=0, vmax=1, alpha=0.75,
    )
    map_ax.plot(
        trajectory[:, 0], trajectory[:, 1], color="#1f77b4", linewidth=1.0,
        alpha=0.7,
    )
    map_ax.set_title("Map rebuilt at the optimised poses")
    map_ax.set_aspect("equal")
    map_ax.set_xlim(*extent[:2])
    map_ax.set_ylim(*extent[2:])
    map_ax.grid(True, alpha=0.2)

    # ---- optimiser cost ----
    history = np.maximum(np.asarray(trace.cost_history, dtype=float), 1e-9)
    cost_ax.semilogy(np.arange(len(history)), history, color="#9467bd",
                     marker=".")
    cost_ax.set_xlabel("iteration")
    cost_ax.set_ylabel("weighted residual cost")
    cost_ax.set_title("Gauss-Newton convergence after loop closure")
    cost_ax.grid(True, alpha=0.25, which="both")

    fig.suptitle("ATLAS Phase 11 — pose-graph loop closure")
    plt.tight_layout()

    out = Path(__file__).resolve().parents[1] / "experiments" / "plots"
    out.mkdir(parents=True, exist_ok=True)
    fig.savefig(out / "phase11_loop_closure.png", dpi=150)
    print(f"\nsaved plot -> {out / 'phase11_loop_closure.png'}")

    if matplotlib.get_backend().lower() not in {
        "agg", "pdf", "svg", "ps", "cairo"
    }:
        plt.show()


if __name__ == "__main__":
    main()
