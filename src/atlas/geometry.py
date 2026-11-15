"""Basic geometry utilities used throughout ATLAS.

Conventions:
- Angles in radians.
- Heading theta = 0 points along +x; counter-clockwise is positive.
"""
from __future__ import annotations

import math

import numpy as np


def deg_to_rad(degrees: float) -> float:
    return math.radians(degrees)


def rad_to_deg(radians: float) -> float:
    return math.degrees(radians)


def euclidean_distance(p1, p2) -> float:
    """d = sqrt((x2 - x1)^2 + (y2 - y1)^2)"""
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])


def wrap_angle(angle: float) -> float:
    """Wrap an angle to the interval [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def angle_difference(target: float, current: float) -> float:
    """Smallest signed angle that rotates `current` onto `target`."""
    return wrap_angle(target - current)


def rotation_matrix(theta: float) -> np.ndarray:
    """2x2 counter-clockwise rotation matrix."""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


def relative_pose(a, b) -> tuple[float, float, float]:
    """Pose of frame ``b`` expressed in frame ``a``.

    ``a`` and ``b`` are ``(x, y, theta)`` poses; the result is the rigid
    transform that takes a point from ``b`` into ``a``.
    """
    ax, ay, atheta = a
    bx, by, btheta = b
    dx, dy = bx - ax, by - ay
    c, s = math.cos(atheta), math.sin(atheta)
    return (
        c * dx + s * dy,
        -s * dx + c * dy,
        wrap_angle(btheta - atheta),
    )


def compose_pose(a, rel) -> tuple[float, float, float]:
    """Apply the relative pose ``rel`` to frame ``a``."""
    ax, ay, atheta = a
    rx, ry, rtheta = rel
    c, s = math.cos(atheta), math.sin(atheta)
    return (
        ax + c * rx - s * ry,
        ay + s * rx + c * ry,
        wrap_angle(atheta + rtheta),
    )


def robot_to_world(point_robot, pose) -> np.ndarray:
    """Convert a point in the robot frame to world coordinates.

    pose = (x, y, theta) of the robot in the world frame.
    """
    x, y, theta = pose
    return rotation_matrix(theta) @ np.asarray(point_robot, dtype=float) + np.array([x, y])


def world_to_robot(point_world, pose) -> np.ndarray:
    """Convert a point in world coordinates to the robot frame."""
    x, y, theta = pose
    return rotation_matrix(-theta) @ (np.asarray(point_world, dtype=float) - np.array([x, y]))
