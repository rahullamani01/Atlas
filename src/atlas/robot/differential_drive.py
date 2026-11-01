"""Differential-drive robot kinematics (Phase 1)."""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from atlas.geometry import wrap_angle


@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0  # radians, 0 = facing +x, counter-clockwise positive


@dataclass
class DifferentialDriveRobot:
    wheel_radius: float = 0.05       # r  [m]
    wheel_separation: float = 0.30   # L  [m]
    body_radius: float = 0.20        # used for collision checks [m]
    max_linear_speed: float = 1.0    # [m/s]
    max_angular_speed: float = 2.0   # [rad/s]
    pose: Pose = field(default_factory=Pose)

    def __post_init__(self) -> None:
        if self.wheel_radius <= 0:
            raise ValueError("wheel_radius must be positive")
        if self.wheel_separation <= 0:
            raise ValueError("wheel_separation must be positive")
        if self.body_radius < 0:
            raise ValueError("body_radius must be non-negative")
        if self.max_linear_speed < 0 or self.max_angular_speed < 0:
            raise ValueError("speed limits must be non-negative")

    # ---- wheel <-> body velocity conversions ----
    def wheel_speeds_to_twist(self, omega_l: float, omega_r: float) -> tuple[float, float]:
        """v = r(wR + wL)/2,  omega = r(wR - wL)/L"""
        v = self.wheel_radius * (omega_r + omega_l) / 2.0
        w = self.wheel_radius * (omega_r - omega_l) / self.wheel_separation
        return v, w

    def twist_to_wheel_speeds(self, v: float, w: float) -> tuple[float, float]:
        """Inverse of wheel_speeds_to_twist. Returns (omega_l, omega_r)."""
        r, L = self.wheel_radius, self.wheel_separation
        omega_r = (2.0 * v + w * L) / (2.0 * r)
        omega_l = (2.0 * v - w * L) / (2.0 * r)
        return omega_l, omega_r

    # ---- motion update ----
    def step(self, v: float, w: float, dt: float, method: str = "midpoint") -> None:
        """Advance the pose by dt seconds under constant (v, w).

        method: "euler"    - simple forward Euler
                "midpoint" - uses the heading halfway through the step
                "exact"    - closed-form solution for constant v, w
        """
        v = max(-self.max_linear_speed, min(self.max_linear_speed, v))
        w = max(-self.max_angular_speed, min(self.max_angular_speed, w))
        p = self.pose

        if method == "euler":
            p.x += v * math.cos(p.theta) * dt
            p.y += v * math.sin(p.theta) * dt
        elif method == "midpoint":
            th_mid = p.theta + 0.5 * w * dt
            p.x += v * math.cos(th_mid) * dt
            p.y += v * math.sin(th_mid) * dt
        elif method == "exact":
            if abs(w) < 1e-9:
                p.x += v * math.cos(p.theta) * dt
                p.y += v * math.sin(p.theta) * dt
            else:
                th_new = p.theta + w * dt
                p.x += (v / w) * (math.sin(th_new) - math.sin(p.theta))
                p.y -= (v / w) * (math.cos(th_new) - math.cos(p.theta))
        else:
            raise ValueError(f"unknown method: {method}")

        p.theta = wrap_angle(p.theta + w * dt)

    def step_wheels(self, omega_l: float, omega_r: float, dt: float,
                    method: str = "midpoint") -> None:
        """Advance the pose by dt seconds given wheel angular velocities [rad/s].

        This is the forward-kinematics entry point: each wheel speed is mapped to
        a body twist, then integrated with `step`. Positive wheel speeds drive
        the robot forward; omega_r > omega_l turns counter-clockwise.
        """
        v, w = self.wheel_speeds_to_twist(omega_l, omega_r)
        self.step(v, w, dt, method)
