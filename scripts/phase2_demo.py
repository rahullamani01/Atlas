"""Phase 2 experiment: occupancy grid, Dijkstra vs A* on the same map."""
from __future__ import annotations

import math
import time
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

from atlas.planning.grid import OccupancyGrid
from atlas.planning.search import a_star, dijkstra
from atlas.simulation.world import RectObstacle, World


def build_world() -> World:
    world = World(20, 20, [
        RectObstacle(6, 0, 7, 14),     # vertical wall with a gap at the top
        RectObstacle(10, 9, 17, 10),   # horizontal wall
        RectObstacle(13, 2, 15, 4),    # free-standing block
    ])
    return world


def describe(name: str, result, elapsed: float, resolution: float) -> None:
    if not result.found:
        print(f"{name:>8}: no path found (expanded {result.expanded})")
        return
    length = result.cost * resolution
    print(
        f"{name:>8}: cost {result.cost:6.2f} cells = {length:5.2f} m | "
        f"expanded {result.expanded:5d} | {elapsed * 1000:6.1f} ms | "
        f"{len(result.path):4d} waypoints"
    )


def main() -> None:
    world = build_world()
    resolution = 0.25
    robot_radius = 0.20
    grid = OccupancyGrid.from_world(
        world, resolution=resolution, inflate_radius=robot_radius
    )

    start_world = (1.0, 1.0)
    goal_world = (18.0, 18.0)
    start = grid.world_to_cell(*start_world)
    goal = grid.world_to_cell(*goal_world)

    print(
        f"grid {grid.width}x{grid.height} @ {resolution} m/cell | "
        f"free space {grid.free_ratio():.1%} | inflation {robot_radius} m"
    )
    print(f"start {start} -> goal {goal}")

    # Dijkstra
    t0 = time.perf_counter()
    d = dijkstra(grid, start, goal)
    d_time = time.perf_counter() - t0
    describe("dijkstra", d, d_time, resolution)

    # A* with the exact 8-connected heuristic
    t0 = time.perf_counter()
    a = a_star(grid, start, goal, heuristic="octile")
    a_time = time.perf_counter() - t0
    describe("a*", a, a_time, resolution)

    if d.found and a.found:
        assert math.isclose(d.cost, a.cost)
        print(
            f"same optimal cost; A* expanded {d.expanded - a.expanded} fewer cells "
            f"({(1 - a.expanded / d.expanded):.0%} reduction)"
        )

    # ---- plot ----
    fig, axes = plt.subplots(1, 2, figsize=(12, 6.5), sharex=True, sharey=True)
    extent = (0, world.width, 0, world.height)

    for ax, result, title in ((axes[0], d, "Dijkstra"), (axes[1], a, "A* (octile)")):
        ax.imshow(
            grid.blocked,
            origin="lower",
            extent=extent,
            cmap="Greys",
            vmin=0,
            vmax=1,
            alpha=0.85,
        )
        if result.found:
            xs, ys = zip(*(grid.cell_to_world(cell) for cell in result.path))
            ax.plot(xs, ys, color="#1f77b4", linewidth=2.2, label="path")
        cells = result.expanded_cells
        if cells:
            ex, ey = zip(*(grid.cell_to_world(cell) for cell in cells))
            ax.scatter(ex, ey, s=3, color="#d62728", alpha=0.35, label="expanded")
        ax.add_patch(Circle(start_world, robot_radius, color="green", zorder=3))
        ax.plot(*goal_world, marker="*", markersize=16, color="gold", markeredgecolor="black", zorder=3)
        ax.set_title(f"{title}: {result.expanded} expanded, cost {result.cost:.2f} cells")
        ax.set_xlim(0, world.width)
        ax.set_ylim(0, world.height)
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.2)
        ax.legend(loc="upper left", framealpha=0.9)

    fig.suptitle("ATLAS Phase 2 — path planning on an inflated occupancy grid")
    plt.tight_layout()

    out = Path(__file__).resolve().parents[1] / "experiments" / "plots"
    out.mkdir(parents=True, exist_ok=True)
    fig.savefig(out / "phase2_planning.png", dpi=150)
    print(f"saved plot -> {out / 'phase2_planning.png'}")

    if matplotlib.get_backend().lower() not in {"agg", "pdf", "svg", "ps", "cairo"}:
        plt.show()


if __name__ == "__main__":
    main()
