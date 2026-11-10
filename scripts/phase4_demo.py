"""Phase 4 experiment: LiDAR ray casting with and without range noise."""
from __future__ import annotations

import math
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Rectangle

from atlas.robot.differential_drive import Pose
from atlas.sensors import Lidar
from atlas.simulation.world import RectObstacle, World

POSES = [
    (Pose(2.0, 2.0, 0.0), "open corner"),
    (Pose(5.5, 12.0, math.pi / 2), "beside the long wall"),
    (Pose(12.0, 6.0, math.pi), "between walls"),
    (Pose(17.5, 17.5, -math.pi / 2), "far corner"),
]
MAX_RANGE = 8.0
SIGMA = 0.03


def build_world() -> World:
    return World(20, 20, [
        RectObstacle(6, 0, 7, 14),
        RectObstacle(10, 9, 17, 10),
        RectObstacle(13, 2, 15, 4),
    ])


def main() -> None:
    world = build_world()
    lidar = Lidar(
        max_range=MAX_RANGE,
        fov=2 * math.pi,
        angle_increment=math.pi / 180,
        range_sigma=SIGMA,
        seed=7,
    )
    clean_lidar = Lidar(
        max_range=MAX_RANGE,
        fov=2 * math.pi,
        angle_increment=math.pi / 180,
    )

    fig, axes = plt.subplots(2, 2, figsize=(12, 12))
    fig.suptitle(
        f"ATLAS Phase 4 — 360 degree LiDAR, {lidar.ray_count} rays, "
        f"max range {MAX_RANGE:.0f} m, noise sigma {SIGMA} m"
    )

    print(f"rays per scan: {lidar.ray_count}")
    for ax, (pose, label) in zip(axes.flat, POSES):
        clean = clean_lidar.scan(world, pose)
        noisy = lidar.scan(world, pose)

        both_hit = clean.hit_mask & noisy.hit_mask
        noise_rms = (
            float(np.sqrt(np.mean((noisy.ranges[both_hit] - clean.ranges[both_hit]) ** 2)))
            if np.any(both_hit)
            else float("nan")
        )

        for obstacle in world.obstacles:
            ax.add_patch(Rectangle(
                (obstacle.xmin, obstacle.ymin),
                obstacle.xmax - obstacle.xmin,
                obstacle.ymax - obstacle.ymin,
                color="#444444",
            ))

        clean_points = clean.points()
        noisy_points = noisy.points()
        origin = np.array(clean.origin)
        for point in clean_points:
            ax.plot(*zip(origin, point), color="#1f77b4", linewidth=0.25, alpha=0.35)
        ax.scatter(clean_points[:, 0], clean_points[:, 1], s=5,
                   color="#1f77b4", label="clean returns")
        ax.scatter(noisy_points[:, 0], noisy_points[:, 1], s=4,
                   color="#d62728", alpha=0.6, label="noisy returns")
        ax.add_patch(Circle((pose.x, pose.y), 0.2, color="green", zorder=4))
        ax.arrow(
            pose.x,
            pose.y,
            0.6 * math.cos(pose.theta),
            0.6 * math.sin(pose.theta),
            head_width=0.2,
            color="black",
            zorder=5,
        )

        ax.set_xlim(0, world.width)
        ax.set_ylim(0, world.height)
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.2)
        ax.set_title(
            f"{label}: {clean.hits}/{clean.ray_count} hits, "
            f"min {clean.ranges.min():.2f} m, noise RMS {noise_rms:.3f} m",
            fontsize=9,
        )
        ax.legend(loc="upper left", fontsize=8)

        print(
            f"{label:<20} pose ({pose.x:4.1f}, {pose.y:4.1f}, "
            f"{math.degrees(pose.theta):6.0f} deg)  "
            f"hits {clean.hits:3d}/{clean.ray_count}  "
            f"min {clean.ranges.min():5.2f} m  "
            f"noise RMS {noise_rms:.4f} m"
        )

    plt.tight_layout()
    out = Path(__file__).resolve().parents[1] / "experiments" / "plots"
    out.mkdir(parents=True, exist_ok=True)
    fig.savefig(out / "phase4_lidar.png", dpi=150)
    print(f"saved plot -> {out / 'phase4_lidar.png'}")

    if matplotlib.get_backend().lower() not in {"agg", "pdf", "svg", "ps", "cairo"}:
        plt.show()


if __name__ == "__main__":
    main()
