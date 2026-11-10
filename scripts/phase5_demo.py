"""Phase 5 experiment: localize with odometry and LiDAR instead of ground truth.

The robot follows the Phase 3 route while wheel encoders under/over-report by
1% on each side. Raw dead reckoning is compared with a particle filter that
fuses the same odometry with LiDAR against the known map. A second panel shows
how the filter recovers from progressively wider initial priors.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

from atlas.control import PID, PathFollower
from atlas.geometry import wrap_angle
from atlas.localization import ParticleFilter
from atlas.planning import OccupancyGrid, a_star
from atlas.robot.differential_drive import DifferentialDriveRobot, Pose
from atlas.robot.odometry import Odometry
from atlas.sensors import Lidar
from atlas.simulation.simulator import Simulator
from atlas.simulation.world import RectObstacle, World

DT = 0.05
LEFT_SCALE = 0.99       # measured / true wheel speed
RIGHT_SCALE = 1.01
MAX_STEPS = 6000


@dataclass
class TrackingRun:
    truth: np.ndarray
    odometry: np.ndarray
    estimate: np.ndarray
    odom_error: np.ndarray
    pf_error: np.ndarray
    odom_heading_error: np.ndarray
    pf_heading_error: np.ndarray
    reached: bool
    collisions: int
    particle_count: int

    @property
    def duration(self) -> float:
        return len(self.truth) * DT

    def rms(self, values: np.ndarray) -> float:
        return float(np.sqrt(np.mean(values ** 2)))

    def maximum(self, values: np.ndarray) -> float:
        return float(np.max(values))


def build_world() -> World:
    return World(20, 20, [
        RectObstacle(6, 0, 7, 14),     # vertical wall with a gap at the top
        RectObstacle(10, 9, 17, 10),   # horizontal wall
        RectObstacle(13, 2, 15, 4),    # free-standing block
    ])


def plan_route():
    world = build_world()
    grid = OccupancyGrid.from_world(world, resolution=0.25, inflate_radius=0.25)
    result = a_star(
        grid,
        grid.world_to_cell(1.0, 1.0),
        grid.world_to_cell(18.0, 18.0),
    )
    if not result.found:
        raise SystemExit("no path found; cannot run the localization experiment")
    return world, grid, [grid.cell_to_world(cell) for cell in result.path], result.cost


def drive(grid, path, particle_count=800, seed=4) -> TrackingRun:
    start = path[0]
    look = min(5, len(path) - 1)
    heading = math.atan2(path[look][1] - start[1], path[look][0] - start[0])

    world = build_world()
    robot = DifferentialDriveRobot(pose=Pose(start[0], start[1], heading))
    follower = PathFollower(heading_pid=PID(kp=2.0, ki=0.2, kd=0.05))
    sim = Simulator(world, robot, dt=DT, method="midpoint")
    lidar = Lidar(max_range=10.0, range_sigma=0.02, seed=3)

    odometry = Odometry(left_scale=LEFT_SCALE, right_scale=RIGHT_SCALE)
    odometry.reset(Pose(start[0], start[1], heading))

    pf = ParticleFilter(
        grid,
        n_particles=particle_count,
        max_beams=60,
        likelihood_scale=0.2,
        seed=seed,
    )
    pf.initialize(Pose(start[0], start[1], heading),
                  position_std=0.10, heading_std=0.05)

    truth, odom_track, pf_track = [], [], []
    odom_error, pf_error = [], []
    odom_heading, pf_heading = [], []

    for _ in range(MAX_STEPS):
        prior = pf.estimate()
        command = follower.compute(prior, path, DT)
        wheel_l, wheel_r = robot.twist_to_wheel_speeds(command.v, command.w)

        ok = sim.step(command.v, command.w)
        odometry.update_wheels(wheel_l, wheel_r, DT)

        v_hat, w_hat = odometry.wheel_speeds_to_twist(wheel_l, wheel_r)
        pf.predict(v_hat, w_hat, DT)
        pf.update(lidar.scan(world, robot.pose))

        est = pf.estimate()
        truth.append((robot.pose.x, robot.pose.y))
        odom_track.append((odometry.pose.x, odometry.pose.y))
        pf_track.append((est.x, est.y))

        odom_error.append(math.hypot(
            odometry.pose.x - robot.pose.x, odometry.pose.y - robot.pose.y
        ))
        pf_error.append(math.hypot(est.x - robot.pose.x, est.y - robot.pose.y))
        odom_heading.append(abs(wrap_angle(odometry.pose.theta - robot.pose.theta)))
        pf_heading.append(abs(wrap_angle(est.theta - robot.pose.theta)))

        if not ok or follower.goal_reached:
            break

    return TrackingRun(
        truth=np.array(truth),
        odometry=np.array(odom_track),
        estimate=np.array(pf_track),
        odom_error=np.array(odom_error),
        pf_error=np.array(pf_error),
        odom_heading_error=np.array(odom_heading),
        pf_heading_error=np.array(pf_heading),
        reached=follower.goal_reached,
        collisions=sim.collision_count,
        particle_count=particle_count,
    )


def recovery_curve(grid, world, truth_pose, prior_std, updates=30,
                   particle_count=1200, seed=2):
    """Stationary robot: error while a filter started with a wrong prior runs."""
    lidar = Lidar(max_range=10.0, range_sigma=0.02, seed=1)
    pf = ParticleFilter(
        grid,
        n_particles=particle_count,
        max_beams=60,
        likelihood_scale=0.2,
        seed=seed,
    )
    pf.initialize(
        Pose(truth_pose.x + 1.0, truth_pose.y - 0.5, truth_pose.theta + 0.15),
        position_std=prior_std,
        heading_std=0.2,
    )
    errors = []
    for _ in range(updates):
        pf.update(lidar.scan(world, truth_pose))
        estimate = pf.estimate()
        errors.append(math.hypot(
            estimate.x - truth_pose.x, estimate.y - truth_pose.y
        ))
    return np.array(errors)


def print_table(run: TrackingRun) -> None:
    rows = [
        ("position RMS [m]", run.rms(run.odom_error), run.rms(run.pf_error)),
        ("position max [m]", run.maximum(run.odom_error), run.maximum(run.pf_error)),
        ("final error [m]", run.odom_error[-1], run.pf_error[-1]),
        ("heading RMS [deg]",
         math.degrees(run.rms(run.odom_heading_error)),
         math.degrees(run.rms(run.pf_heading_error))),
        ("heading final [deg]",
         math.degrees(run.odom_heading_error[-1]),
         math.degrees(run.pf_heading_error[-1])),
    ]
    print(f"{'metric':<24}{'odometry':>14}{'particle filter':>18}")
    print("-" * 56)
    for label, odom, pf in rows:
        print(f"{label:<24}{odom:>14.3f}{pf:>18.3f}")
    print(f"\ngoal reached: {run.reached}   collisions: {run.collisions}   "
          f"duration: {run.duration:.1f} s   particles: {run.particle_count}")


def main() -> None:
    world, grid, path, cost = plan_route()
    print(f"planned path: {len(path)} waypoints, {cost * grid.resolution:.2f} m")
    print(f"wheel scales: left {LEFT_SCALE}, right {RIGHT_SCALE}; "
          f"LiDAR {10.0:.0f} m, sigma 0.02 m\n")

    run = drive(grid, path)
    print_table(run)

    if run.rms(run.odom_error) > 0:
        reduction = 1 - run.rms(run.pf_error) / run.rms(run.odom_error)
        print(f"particle filter reduces position RMS by {reduction:.0%} "
              f"relative to dead reckoning")

    # ---- recovery from increasingly honest priors ----
    truth_pose = Pose(5.0, 5.0, 0.3)
    priors = (0.3, 0.5, 0.8)
    curves = {std: recovery_curve(grid, world, truth_pose, std) for std in priors}
    print("\nstationary recovery after 30 updates:")
    for std, curve in curves.items():
        print(f"  prior std {std:.1f} m: start {curve[0]:.2f} m -> "
              f"end {curve[-1]:.2f} m")

    # ---- plot ----
    fig = plt.figure(figsize=(16, 5.6))
    grid_ax = fig.add_subplot(1, 3, 1)
    error_ax = fig.add_subplot(1, 3, 2)
    recovery_ax = fig.add_subplot(1, 3, 3)

    extent = (0, world.width, 0, world.height)
    grid_ax.imshow(grid.blocked, origin="lower", extent=extent, cmap="Greys",
                   vmin=0, vmax=1, alpha=0.8)
    grid_ax.plot(*zip(*path), color="#7f7f7f", linewidth=1.4, label="planned")
    grid_ax.plot(*run.truth.T, color="#2ca02c", linewidth=2.0, label="truth")
    grid_ax.plot(*run.odometry.T, color="#ff7f0e", linewidth=1.2,
                 linestyle="--", label="odometry")
    grid_ax.plot(*run.estimate.T, color="#1f77b4", linewidth=1.8,
                 label="particle filter")
    grid_ax.plot(*path[0], marker="o", color="green", markersize=8, zorder=4)
    grid_ax.plot(*path[-1], marker="*", color="gold", markeredgecolor="black",
                 markersize=15, zorder=4)
    grid_ax.set_xlim(0, world.width)
    grid_ax.set_ylim(0, world.height)
    grid_ax.set_aspect("equal")
    grid_ax.set_title("Closed-loop trajectory")
    grid_ax.grid(True, alpha=0.2)
    grid_ax.legend(loc="upper left", fontsize=8)

    time_axis = np.arange(len(run.odom_error)) * DT
    error_ax.semilogy(time_axis, run.odom_error, color="#ff7f0e",
                      label=f"odometry: RMS {run.rms(run.odom_error):.2f} m")
    error_ax.semilogy(time_axis, run.pf_error, color="#1f77b4",
                      label=f"particle filter: RMS {run.rms(run.pf_error):.2f} m")
    error_ax.set_xlabel("time [s]")
    error_ax.set_ylabel("position error [m]")
    error_ax.set_ylim(bottom=1e-3)
    error_ax.set_title("Estimation error over the run")
    error_ax.grid(True, which="both", alpha=0.25)
    error_ax.legend(loc="upper left", fontsize=9)

    for std, curve in curves.items():
        recovery_ax.plot(np.arange(1, len(curve) + 1), curve,
                         label=f"prior std {std:.1f} m")
    recovery_ax.set_xlabel("LiDAR update")
    recovery_ax.set_ylabel("position error [m]")
    recovery_ax.set_title("Recovery from a 1.1 m offset prior")
    recovery_ax.grid(True, alpha=0.25)
    recovery_ax.legend(loc="upper right", fontsize=9)

    fig.suptitle("ATLAS Phase 5 — particle-filter localization")
    plt.tight_layout()

    out = Path(__file__).resolve().parents[1] / "experiments" / "plots"
    out.mkdir(parents=True, exist_ok=True)
    fig.savefig(out / "phase5_localization.png", dpi=150)
    print(f"\nsaved plot -> {out / 'phase5_localization.png'}")

    if matplotlib.get_backend().lower() not in {"agg", "pdf", "svg", "ps", "cairo"}:
        plt.show()


if __name__ == "__main__":
    main()
