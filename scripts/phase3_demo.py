"""Phase 3 experiment: follow an A* path with P-only and PID heading control."""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

from atlas.control import PID, PathFollower
from atlas.geometry import wrap_angle
from atlas.planning import OccupancyGrid, a_star
from atlas.robot.differential_drive import DifferentialDriveRobot, Pose
from atlas.simulation.simulator import Simulator
from atlas.simulation.world import RectObstacle, World

DT = 0.05
WHEEL_BIAS = 0.2  # rad/s of unmodelled yaw rate, to expose integral action


@dataclass
class Run:
    name: str
    trajectory: np.ndarray
    cross_track: np.ndarray
    heading_error: np.ndarray
    commands: np.ndarray
    goal: tuple[float, float]
    collisions: int
    reached: bool
    time: float
    length: float

    @property
    def final_error(self) -> float:
        return float(
            math.hypot(
                self.trajectory[-1, 0] - self.goal[0],
                self.trajectory[-1, 1] - self.goal[1],
            )
        )

    @property
    def rms_cross_track(self) -> float:
        return float(np.sqrt(np.mean(self.cross_track ** 2)))

    @property
    def max_cross_track(self) -> float:
        return float(np.max(self.cross_track))

    @property
    def rms_heading_error(self) -> float:
        return float(np.sqrt(np.mean(self.heading_error ** 2)))

    @property
    def control_effort(self) -> float:
        return float(np.sum(np.abs(self.commands[:, 1])) * DT)


def build_world() -> World:
    return World(20, 20, [
        RectObstacle(6, 0, 7, 14),     # vertical wall with a gap at the top
        RectObstacle(10, 9, 17, 10),   # horizontal wall
        RectObstacle(13, 2, 15, 4),    # free-standing block
    ])


def segment_projection(point, a, b):
    """Return (distance, path heading) for the nearest point on segment ab."""
    px, py = point
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    length_squared = dx * dx + dy * dy
    t = 0.0 if length_squared == 0 else max(
        0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_squared)
    )
    distance = math.hypot(px - (ax + t * dx), py - (ay + t * dy))
    return distance, math.atan2(dy, dx)


def path_errors(pose, path):
    best = min(
        (segment_projection((pose.x, pose.y), a, b) for a, b in zip(path, path[1:])),
        key=lambda item: item[0],
    )
    distance, heading = best
    return distance, wrap_angle(heading - pose.theta)


def drive(name, path, heading_pid):
    start = path[0]
    look = min(5, len(path) - 1)
    heading = math.atan2(path[look][1] - start[1], path[look][0] - start[0])

    world = build_world()
    robot = DifferentialDriveRobot(pose=Pose(start[0], start[1], heading))
    follower = PathFollower(heading_pid=heading_pid)
    sim = Simulator(world, robot, dt=DT, method="midpoint")

    points = [(robot.pose.x, robot.pose.y)]
    cross_track, heading_error, commands = [], [], []

    for _ in range(6000):
        command = follower.compute(robot.pose, path, DT)
        ok = sim.step(command.v, command.w + WHEEL_BIAS)
        points.append((robot.pose.x, robot.pose.y))
        distance, error = path_errors(robot.pose, path)
        cross_track.append(distance)
        heading_error.append(error)
        commands.append((command.v, command.w))
        if not ok or follower.goal_reached:
            break

    trajectory = np.array(points)
    length = float(np.sum(np.hypot(*np.diff(trajectory, axis=0).T)))
    return Run(
        name=name,
        trajectory=trajectory,
        cross_track=np.array(cross_track),
        heading_error=np.array(heading_error),
        commands=np.array(commands),
        goal=path[-1],
        collisions=sim.collision_count,
        reached=follower.goal_reached,
        time=(len(commands)) * DT,
        length=length,
    )


def print_table(runs):
    rows = [
        ("reached goal", lambda r: "yes" if r.reached else "no"),
        ("collisions", lambda r: str(r.collisions)),
        ("time [s]", lambda r: f"{r.time:.1f}"),
        ("driven length [m]", lambda r: f"{r.length:.2f}"),
        ("final error [m]", lambda r: f"{r.final_error:.3f}"),
        ("cross-track RMS [m]", lambda r: f"{r.rms_cross_track:.4f}"),
        ("cross-track max [m]", lambda r: f"{r.max_cross_track:.4f}"),
        ("heading RMS [deg]", lambda r: f"{math.degrees(r.rms_heading_error):.2f}"),
        ("control effort [rad]", lambda r: f"{r.control_effort:.1f}"),
    ]
    width = max(14, max(len(r.name) for r in runs) + 2)
    header = f"{'metric':<22}" + "".join(f"{r.name:>{width}}" for r in runs)
    print(header)
    print("-" * len(header))
    for label, getter in rows:
        print(f"{label:<22}" + "".join(f"{getter(r):>{width}}" for r in runs))


def main() -> None:
    world = build_world()
    resolution = 0.25
    grid = OccupancyGrid.from_world(
        world, resolution=resolution, inflate_radius=0.25
    )
    start_cell = grid.world_to_cell(1.0, 1.0)
    goal_cell = grid.world_to_cell(18.0, 18.0)
    result = a_star(grid, start_cell, goal_cell)

    if not result.found:
        raise SystemExit("no path found; cannot run the control experiment")
    path = [grid.cell_to_world(cell) for cell in result.path]

    print(
        f"planned path: {len(path)} waypoints, "
        f"{result.cost * resolution:.2f} m, starts {path[0]}, goal {path[-1]}"
    )
    print(f"control at {1 / DT:.0f} Hz, injected wheel bias {WHEEL_BIAS} rad/s\n")

    p_only = drive("P-only (kp=2)", path, PID(kp=2.0))
    pid = drive("PID (kp=2, ki=0.2, kd=0.05)", path, PID(kp=2.0, ki=0.2, kd=0.05))

    print_table([p_only, pid])
    if p_only.rms_cross_track > 0:
        reduction = 1 - pid.rms_cross_track / p_only.rms_cross_track
        print(f"\ncross-track RMS reduced by {reduction:.0%} with integral action")

    # ---- plot ----
    fig, (ax_path, ax_error) = plt.subplots(1, 2, figsize=(13, 6.5))
    extent = (0, world.width, 0, world.height)
    ax_path.imshow(
        grid.blocked,
        origin="lower",
        extent=extent,
        cmap="Greys",
        vmin=0,
        vmax=1,
        alpha=0.8,
    )
    ax_path.plot(*zip(*path), color="#7f7f7f", linewidth=1.5, label="planned")
    ax_path.plot(*p_only.trajectory.T, color="#ff7f0e", linewidth=1.8, label=p_only.name)
    ax_path.plot(*pid.trajectory.T, color="#1f77b4", linewidth=1.8, label=pid.name)
    ax_path.plot(*path[0], marker="o", color="green", markersize=9, zorder=4)
    ax_path.plot(*path[-1], marker="*", color="gold", markeredgecolor="black",
                 markersize=16, zorder=4)
    ax_path.set_xlim(0, world.width)
    ax_path.set_ylim(0, world.height)
    ax_path.set_aspect("equal")
    ax_path.set_title("Planned path and driven trajectories")
    ax_path.grid(True, alpha=0.2)
    ax_path.legend(loc="upper left", fontsize=9)

    ax_error.plot(p_only.cross_track, color="#ff7f0e",
                  label=f"{p_only.name}: RMS {p_only.rms_cross_track:.3f} m")
    ax_error.plot(pid.cross_track, color="#1f77b4",
                  label=f"{pid.name}: RMS {pid.rms_cross_track:.3f} m")
    ax_error.set_xlabel("control step")
    ax_error.set_ylabel("cross-track error [m]")
    ax_error.set_title(f"Tracking error with {WHEEL_BIAS} rad/s wheel bias")
    ax_error.grid(True, alpha=0.3)
    ax_error.legend(loc="upper right", fontsize=9)

    fig.suptitle("ATLAS Phase 3 — closed-loop path following")
    plt.tight_layout()

    out = Path(__file__).resolve().parents[1] / "experiments" / "plots"
    out.mkdir(parents=True, exist_ok=True)
    fig.savefig(out / "phase3_tracking.png", dpi=150)
    print(f"saved plot -> {out / 'phase3_tracking.png'}")

    if matplotlib.get_backend().lower() not in {"agg", "pdf", "svg", "ps", "cairo"}:
        plt.show()


if __name__ == "__main__":
    main()
