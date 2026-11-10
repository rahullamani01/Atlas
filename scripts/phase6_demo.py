"""Phase 6 experiment: detect a new obstacle, replan, and recover.

A block is dropped onto the Phase 3 route after the robot has started. The
reactive run fuses LiDAR into a dynamic obstacle map, replans around the block
and reaches the goal. A blind baseline keeps following the stale path and
collides. Both runs use ground-truth pose so the experiment isolates perception
and recovery; Phase 5 already demonstrated control on the estimated pose.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

from atlas.control import PID, PathFollower
from atlas.perception import DynamicObstacleMap
from atlas.planning import OccupancyGrid, a_star
from atlas.recovery import NavigationSupervisor
from atlas.robot.differential_drive import DifferentialDriveRobot, Pose
from atlas.sensors import Lidar
from atlas.simulation.simulator import Simulator
from atlas.simulation.world import RectObstacle, World

DT = 0.05
INFLATION = 0.30          # 0.20 m body + 0.10 m margin for replanned corners
INJECT_STEP = 80
MAX_STEPS = 4000
BLOCK = RectObstacle(2.32, 5.7, 2.92, 6.3)


@dataclass
class Run:
    name: str
    trajectory: np.ndarray
    distance: np.ndarray
    reached: bool
    failed: bool
    collisions: int
    steps: int
    final_error: float
    detections: list[int] | None = None
    replans: int = 0
    replan_times: list[float] | None = None
    detected_cells: set | None = None
    map: DynamicObstacleMap | None = None

    @property
    def duration(self) -> float:
        return self.steps * DT


def base_world() -> World:
    return World(20, 20, [
        RectObstacle(6, 0, 7, 14),
        RectObstacle(10, 9, 17, 10),
        RectObstacle(13, 2, 15, 4),
    ])


def setup():
    world = base_world()
    grid = OccupancyGrid.from_world(world, resolution=0.25,
                                    inflate_radius=INFLATION)
    goal = grid.world_to_cell(18.125, 18.125)
    result = a_star(grid, grid.world_to_cell(1.125, 1.125), goal)
    if not result.found:
        raise SystemExit("no initial path; cannot run the Phase 6 experiment")
    path = [grid.cell_to_world(cell) for cell in result.path]
    heading = math.atan2(path[5][1] - path[0][1], path[5][0] - path[0][0])
    robot = DifferentialDriveRobot(pose=Pose(path[0][0], path[0][1], heading))
    return world, grid, goal, path, robot


def distance_to_goal(pose, goal_world) -> float:
    return math.hypot(pose.x - goal_world[0], pose.y - goal_world[1])


def run_reactive() -> Run:
    world, grid, goal, path, robot = setup()
    goal_world = grid.cell_to_world(goal)
    sim = Simulator(world, robot, dt=DT, method="midpoint")
    lidar = Lidar(max_range=10.0, range_sigma=0.02, seed=3)
    detector = DynamicObstacleMap(grid, clearance=INFLATION)
    supervisor = NavigationSupervisor(grid, goal, detector=detector)
    supervisor.plan(robot.pose)

    trajectory, distances, detections = [], [], []
    for step in range(MAX_STEPS):
        if step == INJECT_STEP:
            world.obstacles.append(BLOCK)
        command = supervisor.step(
            robot.pose, lidar.scan(world, robot.pose), DT
        )
        ok = sim.step(command.v, command.w)
        trajectory.append((robot.pose.x, robot.pose.y))
        distances.append(distance_to_goal(robot.pose, goal_world))
        detections.append(len(supervisor.detected_cells))
        if not ok or supervisor.goal_reached or supervisor.failed:
            break

    replan_times = [
        event.step * DT for event in supervisor.events if event.kind == "replan"
    ]
    return Run(
        name="reactive",
        trajectory=np.array(trajectory),
        distance=np.array(distances),
        reached=supervisor.goal_reached,
        failed=supervisor.failed,
        collisions=sim.collision_count,
        steps=len(trajectory),
        final_error=distance_to_goal(robot.pose, goal_world),
        detections=detections,
        replans=supervisor.replan_count,
        replan_times=replan_times,
        detected_cells=set(supervisor.detected_cells),
        map=supervisor.map,
    )


def run_blind() -> Run:
    world, grid, goal, path, robot = setup()
    goal_world = grid.cell_to_world(goal)
    sim = Simulator(world, robot, dt=DT, method="midpoint")
    follower = PathFollower(heading_pid=PID(kp=2.0, ki=0.2, kd=0.05))

    trajectory, distances = [], []
    for step in range(MAX_STEPS):
        if step == INJECT_STEP:
            world.obstacles.append(BLOCK)
        command = follower.compute(robot.pose, path, DT)
        ok = sim.step(command.v, command.w)
        trajectory.append((robot.pose.x, robot.pose.y))
        distances.append(distance_to_goal(robot.pose, goal_world))
        if not ok or follower.goal_reached:
            break

    return Run(
        name="blind",
        trajectory=np.array(trajectory),
        distance=np.array(distances),
        reached=follower.goal_reached,
        failed=not follower.goal_reached,
        collisions=sim.collision_count,
        steps=len(trajectory),
        final_error=distance_to_goal(robot.pose, goal_world),
    )


def print_table(runs: list[Run]) -> None:
    rows = [
        ("reached goal", lambda r: "yes" if r.reached else "no"),
        ("collisions", lambda r: str(r.collisions)),
        ("duration [s]", lambda r: f"{r.duration:.1f}"),
        ("final error [m]", lambda r: f"{r.final_error:.2f}"),
        ("detected cells", lambda r: str(len(r.detected_cells or ()))),
        ("replans", lambda r: str(r.replans)),
    ]
    width = max(12, max(len(r.name) for r in runs) + 2)
    header = f"{'metric':<20}" + "".join(f"{r.name:>{width}}" for r in runs)
    print(header)
    print("-" * len(header))
    for label, getter in rows:
        print(f"{label:<20}" + "".join(f"{getter(r):>{width}}" for r in runs))


def main() -> None:
    reactive = run_reactive()
    blind = run_blind()
    print(f"obstacle injected at t = {INJECT_STEP * DT:.1f} s: "
          f"({BLOCK.xmin}, {BLOCK.ymin}) to ({BLOCK.xmax}, {BLOCK.ymax})")
    print(f"planned with {INFLATION:.2f} m inflation, {1 / DT:.0f} Hz\n")
    print_table([reactive, blind])
    if reactive.replan_times:
        print(f"\nreplans at t = "
              f"{', '.join(f'{t:.2f}' for t in reactive.replan_times)} s")

    # ---- plot ----
    world, grid, goal, initial_path, _ = setup()
    fig = plt.figure(figsize=(16, 5.6))
    map_ax = fig.add_subplot(1, 3, 1)
    progress_ax = fig.add_subplot(1, 3, 2)
    detect_ax = fig.add_subplot(1, 3, 3)

    extent = (0, world.width, 0, world.height)
    map_ax.imshow(grid.blocked, origin="lower", extent=extent, cmap="Greys",
                  vmin=0, vmax=1, alpha=0.75)
    map_ax.plot(*zip(*initial_path), color="#7f7f7f", linewidth=1.2,
                label="initial path")
    map_ax.plot(*reactive.trajectory.T, color="#1f77b4", linewidth=1.9,
                label="reactive")
    map_ax.plot(*blind.trajectory.T, color="#ff7f0e", linewidth=1.6,
                linestyle="--", label="blind")
    if reactive.detected_cells:
        cells = [grid.cell_to_world(c) for c in reactive.detected_cells]
        map_ax.scatter(*zip(*cells), marker="s", s=14, color="#d62728",
                       alpha=0.55, label="detected obstacle")
    block = patches.Rectangle(
        (BLOCK.xmin, BLOCK.ymin), BLOCK.xmax - BLOCK.xmin,
        BLOCK.ymax - BLOCK.ymin, fill=False, edgecolor="#d62728",
        linewidth=2.0, label="new block",
    )
    map_ax.add_patch(block)
    map_ax.plot(*initial_path[0], marker="o", color="green", markersize=8)
    map_ax.plot(*initial_path[-1], marker="*", color="gold",
                markeredgecolor="black", markersize=15)
    map_ax.set_xlim(0, world.width)
    map_ax.set_ylim(0, world.height)
    map_ax.set_aspect("equal")
    map_ax.set_title("Route and detour")
    map_ax.grid(True, alpha=0.2)
    map_ax.legend(loc="upper left", fontsize=8)

    reactive_time = np.arange(len(reactive.distance)) * DT
    blind_time = np.arange(len(blind.distance)) * DT
    progress_ax.plot(reactive_time, reactive.distance, color="#1f77b4",
                     label=f"reactive: {reactive.final_error:.2f} m final")
    progress_ax.plot(blind_time, blind.distance, color="#ff7f0e",
                     linestyle="--",
                     label=f"blind: {blind.final_error:.2f} m final")
    progress_ax.axvline(INJECT_STEP * DT, color="#d62728", linestyle=":",
                        label="obstacle injected")
    for index, when in enumerate(reactive.replan_times or []):
        progress_ax.axvline(when, color="#2ca02c", linestyle=":", alpha=0.8,
                            label="replanned" if index == 0 else None)
    progress_ax.set_xlabel("time [s]")
    progress_ax.set_ylabel("distance to goal [m]")
    progress_ax.set_title("Progress")
    progress_ax.grid(True, alpha=0.25)
    progress_ax.legend(loc="upper right", fontsize=8)

    detect_ax.step(reactive_time, reactive.detections or [], where="post",
                   color="#d62728")
    detect_ax.set_xlabel("time [s]")
    detect_ax.set_ylabel("detected cells")
    detect_ax.set_title("Dynamic obstacle map growth")
    detect_ax.grid(True, alpha=0.25)

    fig.suptitle("ATLAS Phase 6 — dynamic obstacle detection and recovery")
    plt.tight_layout()

    out = Path(__file__).resolve().parents[1] / "experiments" / "plots"
    out.mkdir(parents=True, exist_ok=True)
    fig.savefig(out / "phase6_recovery.png", dpi=150)
    print(f"\nsaved plot -> {out / 'phase6_recovery.png'}")

    if matplotlib.get_backend().lower() not in {"agg", "pdf", "svg", "ps", "cairo"}:
        plt.show()


if __name__ == "__main__":
    main()
