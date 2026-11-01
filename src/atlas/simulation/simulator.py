"""Simulator: owns the ground-truth robot state and records the trajectory."""
from __future__ import annotations

import numpy as np

from atlas.robot.differential_drive import DifferentialDriveRobot
from atlas.simulation.world import World


class Simulator:
    def __init__(self, world: World, robot: DifferentialDriveRobot,
                 dt: float = 0.05, method: str = "midpoint"):
        self.world = world
        self.robot = robot
        self.dt = dt
        self.method = method
        self.time = 0.0
        self.collided = False
        self.collision_count = 0
        p = robot.pose
        self._trajectory = [(self.time, p.x, p.y, p.theta)]

    def step(self, v: float, w: float) -> bool:
        """Advance one time step. If the move would collide, the robot stays put.

        Returns True if the step was collision-free.
        """
        p = self.robot.pose
        old = (p.x, p.y, p.theta)
        self.robot.step(v, w, self.dt, self.method)
        ok = True
        if self.world.in_collision(p.x, p.y, self.robot.body_radius):
            p.x, p.y, p.theta = old
            self.collided = True
            self.collision_count += 1
            ok = False
        self.time += self.dt
        self._trajectory.append((self.time, p.x, p.y, p.theta))
        return ok

    def step_wheels(self, omega_l: float, omega_r: float) -> bool:
        """Advance one time step from wheel angular velocities [rad/s]."""
        v, w = self.robot.wheel_speeds_to_twist(omega_l, omega_r)
        return self.step(v, w)

    def trajectory(self) -> np.ndarray:
        """Array of shape (N, 4): columns are time, x, y, theta."""
        return np.array(self._trajectory)