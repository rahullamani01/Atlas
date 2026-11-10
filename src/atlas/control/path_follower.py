"""Carrot-chasing path follower with PID heading control (Phase 3).

The follower takes a polyline of world-space waypoints (convert an occupancy
grid path with ``grid.cell_to_world``), selects a carrot point one lookahead
ahead along the path, and steers toward it with a PID on heading error. Linear
speed is proportional to the remaining distance and is cut when the carrot is
behind the robot, so the robot turns in place instead of driving away.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

from atlas.control.pid import PID
from atlas.geometry import wrap_angle
from atlas.robot.differential_drive import Pose

Point = tuple[float, float]


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass(frozen=True)
class Twist:
    """Body-frame velocity command."""

    v: float = 0.0  # m/s
    w: float = 0.0  # rad/s


class PathFollower:
    """Follow a world-space polyline using lookahead and PID heading control."""

    def __init__(
        self,
        lookahead: float = 0.5,
        max_linear_speed: float = 0.5,
        max_angular_speed: float = 1.5,
        goal_tolerance: float = 0.1,
        distance_gain: float = 0.8,
        min_speed: float = 0.05,
        heading_pid: PID | None = None,
    ) -> None:
        if lookahead <= 0:
            raise ValueError("lookahead must be positive")
        if max_linear_speed <= 0:
            raise ValueError("max_linear_speed must be positive")
        if max_angular_speed <= 0:
            raise ValueError("max_angular_speed must be positive")
        if goal_tolerance <= 0:
            raise ValueError("goal_tolerance must be positive")
        if distance_gain <= 0:
            raise ValueError("distance_gain must be positive")
        if min_speed < 0:
            raise ValueError("min_speed must be non-negative")
        if min_speed > max_linear_speed:
            raise ValueError("min_speed cannot exceed max_linear_speed")

        self.lookahead = lookahead
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.goal_tolerance = goal_tolerance
        self.distance_gain = distance_gain
        self.min_speed = min_speed
        self.heading_pid = heading_pid or PID(
            kp=2.0,
            ki=0.0,
            kd=0.05,
            output_limit=max_angular_speed,
        )
        self._goal_reached = False

    @property
    def goal_reached(self) -> bool:
        return self._goal_reached

    def reset(self) -> None:
        """Clear controller state so the follower can be reused."""
        self.heading_pid.reset()
        self._goal_reached = False

    def lookahead_point(self, pose: Pose, path: Sequence[Point]) -> Point:
        """Return the carrot point one lookahead ahead of the nearest waypoint."""
        index = self._nearest_index(pose, path)
        remaining = self.lookahead
        previous = path[index]

        for point in path[index + 1:]:
            segment = math.hypot(point[0] - previous[0], point[1] - previous[1])
            if segment <= 1e-12:
                continue
            if segment >= remaining:
                t = remaining / segment
                return (
                    previous[0] + t * (point[0] - previous[0]),
                    previous[1] + t * (point[1] - previous[1]),
                )
            remaining -= segment
            previous = point

        return (float(path[-1][0]), float(path[-1][1]))

    def compute(self, pose: Pose, path: Sequence[Point], dt: float) -> Twist:
        """Return the twist command for the current pose."""
        if not path:
            raise ValueError("path must contain at least one point")
        if dt <= 0:
            raise ValueError("dt must be positive")

        goal = path[-1]
        distance_to_goal = math.hypot(goal[0] - pose.x, goal[1] - pose.y)

        if distance_to_goal <= self.goal_tolerance:
            self._goal_reached = True
            self.heading_pid.reset()
            return Twist(0.0, 0.0)

        self._goal_reached = False
        target = self.lookahead_point(pose, path)
        alpha = wrap_angle(
            math.atan2(target[1] - pose.y, target[0] - pose.x) - pose.theta
        )

        w = _clamp(
            self.heading_pid.update(alpha, dt),
            -self.max_angular_speed,
            self.max_angular_speed,
        )

        if abs(alpha) >= math.pi / 2:
            # The carrot is behind the robot: rotate in place.
            v = 0.0
        else:
            desired = self.distance_gain * distance_to_goal * math.cos(alpha)
            v = _clamp(max(desired, self.min_speed), 0.0, self.max_linear_speed)

        return Twist(v, w)

    def _nearest_index(self, pose: Pose, path: Sequence[Point]) -> int:
        best_index = 0
        best_distance = math.inf
        for index, (x, y) in enumerate(path):
            distance = (x - pose.x) ** 2 + (y - pose.y) ** 2
            if distance < best_distance:
                best_distance = distance
                best_index = index
        return best_index
