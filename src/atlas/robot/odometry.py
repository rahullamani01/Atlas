"""Dead-reckoning odometry for a differential-drive robot (Phase 5).

Odometry integrates wheel speeds using the robot geometry. It is deliberately
independent of the simulator's ground truth: systematic wheel-scale errors make
the estimate drift, which is exactly what the particle filter must correct.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from atlas.geometry import wrap_angle
from atlas.robot.differential_drive import Pose


@dataclass
class Odometry:
    wheel_radius: float = 0.05
    wheel_separation: float = 0.30
    left_scale: float = 1.0     # measured / true wheel speed
    right_scale: float = 1.0
    pose: Pose = field(default_factory=Pose)

    def __post_init__(self) -> None:
        if self.wheel_radius <= 0:
            raise ValueError("wheel_radius must be positive")
        if self.wheel_separation <= 0:
            raise ValueError("wheel_separation must be positive")
        if self.left_scale <= 0 or self.right_scale <= 0:
            raise ValueError("wheel scales must be positive")

    def wheel_speeds_to_twist(self, omega_l: float, omega_r: float) -> tuple[float, float]:
        """Twist implied by the measured (scaled) wheel speeds."""
        left = omega_l * self.left_scale
        right = omega_r * self.right_scale
        v = self.wheel_radius * (right + left) / 2.0
        w = self.wheel_radius * (right - left) / self.wheel_separation
        return v, w

    def update_wheels(self, omega_l: float, omega_r: float, dt: float) -> None:
        v, w = self.wheel_speeds_to_twist(omega_l, omega_r)
        self.update_twist(v, w, dt)

    def update_twist(self, v: float, w: float, dt: float) -> None:
        if dt <= 0:
            raise ValueError("dt must be positive")
        p = self.pose
        if abs(w) < 1e-9:
            p.x += v * math.cos(p.theta) * dt
            p.y += v * math.sin(p.theta) * dt
        else:
            theta = p.theta + w * dt
            p.x += (v / w) * (math.sin(theta) - math.sin(p.theta))
            p.y -= (v / w) * (math.cos(theta) - math.cos(p.theta))
        p.theta = wrap_angle(p.theta + w * dt)

    def reset(self, pose: Pose | None = None) -> None:
        self.pose = Pose() if pose is None else pose
