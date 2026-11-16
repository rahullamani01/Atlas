"""Phase 12 experiment: correct the pose graph while the robot keeps driving.

The robot drives the Phase 11 loop, but this time the route continues to a
second goal. When the return to the start area is verified as a loop closure,
the graph is optimised mid-run, the live map is rebuilt from the corrected
poses, odometry and the filter are reset to the corrected pose, and navigation
continues on the corrected map.
"""
from __future__ import annotations

from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

from atlas.evaluation import get_scenario, simulate_online_slam


def print_summary(trace) -> None:
    metrics = trace.metrics
    rows = [
        ("goal reached", "yes" if metrics.reached else "no"),
        ("failed", "yes" if metrics.failed else "no"),
        ("collisions", f"{metrics.collisions}"),
        ("duration [s]", f"{metrics.duration:.2f}"),
        ("graph nodes", f"{metrics.nodes}"),
        ("loop closures", f"{metrics.loop_closures}"),
        ("closure step", f"{metrics.closure_step}"),
        ("closure time [s]", f"{metrics.closure_time:.2f}"),
        ("correction [m]", f"{metrics.correction:.3f}"),
        ("pre-closure RMS [m]", f"{metrics.pre_closure_rms:.3f}"),
        ("post-closure RMS [m]", f"{metrics.post_closure_rms:.3f}"),
        ("ATE reduction", f"{100.0 * metrics.ate_reduction:.1f}%"),
        ("final SLAM RMS [m]", f"{metrics.slam_rms:.3f}"),
        ("final SLAM error [m]", f"{metrics.slam_final:.3f}"),
        ("graph cost before", f"{metrics.graph_cost_before:.2f}"),
        ("graph cost after", f"{metrics.graph_cost_after:.4f}"),
        ("raw map error [cells]", f"{metrics.raw_map_error}"),
        ("corrected map error [cells]", f"{metrics.slam_map_error}"),
        ("map error reduction [cells]", f"{metrics.map_error_reduction}"),
        ("filter RMS [m]", f"{metrics.filter_rms:.3f}"),
        ("final error [m]", f"{metrics.final_error:.3f}"),
    ]
    print(f"\n{'metric':<34}{'value':>10}")
    print("-" * 44)
    for label, value in rows:
        print(f"{label:<34}{value:>10}")


def _xy(poses) -> tuple[list[float], list[float]]:
    return [pose.x for pose in poses], [pose.y for pose in poses]


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
    scenario = get_scenario("online")
    trace = simulate_online_slam(scenario, seed=0)
    print_summary(trace)

    metrics = trace.metrics
    trajectory = np.asarray(trace.trajectory)
    pre = trace.node_pre
    post = trace.node_post
    truth = trace.node_truth
    closure_index = max(0, len(pre) - 1)

    pre_errors = [
        float(np.hypot(pose.x - true.x, pose.y - true.y))
        for pose, true in zip(pre, truth[: len(pre)])
    ]
    post_errors = [
        float(np.hypot(pose.x - true.x, pose.y - true.y))
        for pose, true in zip(post, truth)
    ]

    fig, axes = plt.subplots(2, 3, figsize=(19, 11))
    route_ax, error_ax, shift_ax = axes[0]
    raw_ax, slam_ax, cost_ax = axes[1]

    # ---- trajectory ----
    route_ax.plot(
        trajectory[:, 0], trajectory[:, 1], color="#1f77b4", linewidth=1.4,
        label="driven (truth)",
    )
    pre_x, pre_y = _xy(pre)
    post_x, post_y = _xy(post)
    route_ax.plot(
        pre_x, pre_y, color="#d62728", linewidth=1.0, linestyle="--",
        label="odometry nodes",
    )
    route_ax.plot(
        post_x, post_y, color="#2ca02c", linewidth=1.3,
        label="corrected nodes",
    )
    closure_xy = (post[closure_index].x, post[closure_index].y)
    route_ax.plot(
        closure_xy[0], closure_xy[1], marker="X", color="#9467bd",
        markersize=11, linestyle="none", label="online correction",
    )
    route_ax.annotate(
        "", xy=closure_xy,
        xytext=(pre[closure_index].x, pre[closure_index].y),
        arrowprops={"arrowstyle": "->", "color": "#9467bd", "lw": 1.6},
    )
    route_ax.plot(
        trajectory[0, 0], trajectory[0, 1], marker="o", color="green",
        markersize=8, linestyle="none", label="start",
    )
    for index, goal in enumerate((scenario.goal, *scenario.goal_sequence)):
        route_ax.plot(
            goal[0], goal[1], marker="*", color="gold",
            markeredgecolor="black", markersize=15, linestyle="none",
            label="goal" if index == 0 else "second goal",
        )
    route_ax.set_title("Trajectory: online correction and continued drive")
    route_ax.set_aspect("equal")
    route_ax.grid(True, alpha=0.2)
    route_ax.legend(loc="best", fontsize=7)

    # ---- node error ----
    error_ax.plot(
        np.arange(len(pre_errors)), pre_errors, color="#d62728",
        marker=".", markersize=3, label="odometry",
    )
    error_ax.plot(
        np.arange(len(post_errors)), post_errors, color="#2ca02c",
        marker=".", markersize=3, label="corrected graph",
    )
    error_ax.axvline(
        closure_index, color="#9467bd", linestyle=":", linewidth=1.2,
        label="correction",
    )
    error_ax.set_xlabel("node index")
    error_ax.set_ylabel("position error [m]")
    error_ax.set_title("Node error before and after the closure")
    error_ax.grid(True, alpha=0.25)
    error_ax.legend(loc="upper left", fontsize=8)

    # ---- correction shift ----
    shifts = [
        float(np.hypot(a.x - b.x, a.y - b.y))
        for a, b in zip(pre, post[: len(pre)])
    ]
    shift_ax.plot(
        np.arange(len(shifts)), shifts, color="#9467bd", marker=".",
        markersize=3,
    )
    shift_ax.axvline(
        closure_index, color="#9467bd", linestyle=":", linewidth=1.2,
    )
    shift_ax.set_xlabel("node index")
    shift_ax.set_ylabel("pose shift [m]")
    shift_ax.set_title("How far optimisation moved each node")
    shift_ax.grid(True, alpha=0.25)

    # ---- maps ----
    _draw_map(raw_ax, trace.raw_grid, "Map from odometry poses")
    _draw_map(slam_ax, trace.slam_grid, "Map after the online correction")
    raw_ax.set_xlabel(
        f"surface error vs truth: {metrics.raw_map_error} cells"
    )
    slam_ax.set_xlabel(
        f"surface error vs truth: {metrics.slam_map_error} cells"
    )

    # ---- optimiser cost ----
    history = np.maximum(np.asarray(trace.cost_history, dtype=float), 1e-9)
    cost_ax.semilogy(
        np.arange(len(history)), history, color="#9467bd", marker=".",
    )
    cost_ax.set_xlabel("iteration")
    cost_ax.set_ylabel("weighted residual cost")
    cost_ax.set_title("Gauss-Newton run at the online closure")
    cost_ax.grid(True, alpha=0.25, which="both")

    fig.suptitle(
        "ATLAS Phase 12 — online pose-graph SLAM: correct and keep driving"
    )
    plt.tight_layout()

    out = Path(__file__).resolve().parents[1] / "experiments" / "plots"
    out.mkdir(parents=True, exist_ok=True)
    path = out / "phase12_online_slam.png"
    fig.savefig(path, dpi=150)
    print(f"\nsaved plot -> {path}")

    if matplotlib.get_backend().lower() not in {
        "agg", "pdf", "svg", "ps", "cairo"
    }:
        plt.show()


if __name__ == "__main__":
    main()
