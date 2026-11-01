"""Phase 1 experiments: straight line, rotation, circle, integration error, collision."""
import math

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle

from atlas.robot.differential_drive import DifferentialDriveRobot, Pose
from atlas.simulation.simulator import Simulator
from atlas.simulation.world import RectObstacle, World


def circle_path(method, dt, v=0.5, w=0.5):
    robot = DifferentialDriveRobot(pose=Pose(1, 5, 0.0))
    total = 2 * math.pi / w
    n = round(total / dt)
    dt_eff = total / n
    xs, ys = [robot.pose.x], [robot.pose.y]
    for _ in range(n):
        robot.step(v, w, dt_eff, method)
        xs.append(robot.pose.x)
        ys.append(robot.pose.y)
    return xs, ys


def main():
    # 1) straight line: expect x = 3 + 0.5 * 4 = 5
    sim = Simulator(World(10, 10), DifferentialDriveRobot(pose=Pose(3, 5, 0)), dt=0.05, method="exact")
    for _ in range(80):
        sim.step(0.5, 0.0)
    p = sim.robot.pose
    print(f"[straight] final pose = ({p.x:.3f}, {p.y:.3f}, {p.theta:.3f})  expected x = 5")

    # 2) rotate in place by 180 degrees (theta wraps to -pi, same direction)
    sim = Simulator(World(10, 10), DifferentialDriveRobot(pose=Pose(5, 5, 0)), dt=0.05, method="exact")
    for _ in range(40):
        sim.step(0.0, math.pi / 2)
    p = sim.robot.pose
    print(f"[rotate]   position = ({p.x:.3f}, {p.y:.3f}), |heading| = {abs(math.degrees(p.theta)):.1f} deg")

    # 3) circle: error vs method and time step
    print("\n[circle] max distance from the exact solution during one full loop")
    print(f"{'dt':>6} | {'euler':>10} | {'midpoint':>10} | {'exact':>10}")
    for dt in (0.5, 0.2, 0.1, 0.05, 0.01):
        errs = []
        for m in ("euler", "midpoint", "exact"):
            xs, ys = circle_path(m, dt)
            ex, ey = circle_path("exact", dt)
            errs.append(max(math.hypot(a - b, c - d) for a, b, c, d in zip(xs, ex, ys, ey)))
        print(f"{dt:>6} | {errs[0]:>10.5f} | {errs[1]:>10.5f} | {errs[2]:>10.2e}")

    # 4) plots
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

    for m in ("euler", "midpoint", "exact"):
        xs, ys = circle_path(m, 0.5)
        ax1.plot(xs, ys, marker="o", markersize=3, label=m)
    ax1.set_title("Circle with dt = 0.5 s")
    ax1.set_aspect("equal")
    ax1.legend()
    ax1.grid(True)

    world = World(10, 10, [RectObstacle(6, 3, 7, 8)])
    robot = DifferentialDriveRobot(pose=Pose(1, 5, 0.3))
    sim = Simulator(world, robot, dt=0.05, method="midpoint")
    for _ in range(300):
        sim.step(0.5, 0.05)
    traj = sim.trajectory()
    ax2.plot(traj[:, 1], traj[:, 2], "b-", label="trajectory")
    for o in world.obstacles:
        ax2.add_patch(Rectangle((o.xmin, o.ymin), o.xmax - o.xmin, o.ymax - o.ymin, color="gray"))
    ax2.add_patch(Circle((robot.pose.x, robot.pose.y), robot.body_radius, fill=False, color="red"))
    ax2.set_xlim(0, world.width)
    ax2.set_ylim(0, world.height)
    ax2.set_aspect("equal")
    ax2.set_title(f"Collision test (collisions blocked: {sim.collision_count})")
    ax2.grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()