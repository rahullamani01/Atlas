"""2D LiDAR simulation by ray casting against the simulated world (Phase 4).

Rays are cast from a sensor mounted on the robot. Each ray returns the distance
to the nearest wall or rectangular obstacle, capped at `max_range`. A ray that
hits nothing reports exactly `max_range`.

Angles are in the robot body frame: 0 points along the robot's heading, positive
is counter-clockwise. For a full 360 degree field of view the rays span
[-pi, pi); narrower fields are centred on the heading.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from atlas.robot.differential_drive import Pose
from atlas.simulation.world import RectObstacle, World

_EPS = 1e-9


def _ray_rectangle(
    ox: float, oy: float, dx: float, dy: float, rect: RectObstacle
) -> float | None:
    """Distance along a ray to a rectangle, or None if it is not hit.

    Slab method: intersect the ray with the rectangle's x and y slabs, then
    check that the two parameter intervals overlap.
    """
    t_min, t_max = 0.0, math.inf

    for origin, direction, low, high in (
        (ox, dx, rect.xmin, rect.xmax),
        (oy, dy, rect.ymin, rect.ymax),
    ):
        if abs(direction) < _EPS:
            if origin < low or origin > high:
                return None
            continue
        t1 = (low - origin) / direction
        t2 = (high - origin) / direction
        if t1 > t2:
            t1, t2 = t2, t1
        t_min = max(t_min, t1)
        t_max = min(t_max, t2)
        if t_min > t_max:
            return None

    if t_max < 0:
        return None
    return t_min if t_min >= 0 else t_max


def _ray_boundary(
    ox: float, oy: float, dx: float, dy: float, width: float, height: float
) -> float:
    """Distance to the world boundary along a ray starting inside the world."""
    candidates = []
    if dx > _EPS:
        candidates.append((width - ox) / dx)
    elif dx < -_EPS:
        candidates.append((0.0 - ox) / dx)
    if dy > _EPS:
        candidates.append((height - oy) / dy)
    elif dy < -_EPS:
        candidates.append((0.0 - oy) / dy)
    return min((t for t in candidates if t >= 0), default=math.inf)


@dataclass(frozen=True)
class LidarScan:
    """One scan: body-frame angles, ranges and the sensor origin in world."""

    ranges: np.ndarray
    angles: np.ndarray          # body frame [rad]
    world_angles: np.ndarray    # world frame [rad]
    origin: tuple[float, float]
    max_range: float

    @property
    def hit_mask(self) -> np.ndarray:
        return self.ranges < self.max_range - _EPS

    @property
    def ray_count(self) -> int:
        return len(self.ranges)

    @property
    def hits(self) -> int:
        return int(np.count_nonzero(self.hit_mask))

    def points(self) -> np.ndarray:
        """World-space (N, 2) points for rays that hit something."""
        mask = self.hit_mask
        ranges = self.ranges[mask]
        angles = self.world_angles[mask]
        return np.column_stack(
            (
                self.origin[0] + ranges * np.cos(angles),
                self.origin[1] + ranges * np.sin(angles),
            )
        )


class Lidar:
    """A planar range sensor with configurable resolution and noise."""

    def __init__(
        self,
        max_range: float = 10.0,
        fov: float = 2.0 * math.pi,
        angle_increment: float = math.pi / 180.0,
        range_sigma: float = 0.0,
        mount_offset: tuple[float, float] = (0.0, 0.0),
        angle_offset: float = 0.0,
        seed: int | None = None,
    ) -> None:
        if max_range <= 0:
            raise ValueError("max_range must be positive")
        if not 0 < fov <= 2.0 * math.pi:
            raise ValueError("fov must be in (0, 2*pi]")
        if angle_increment <= 0:
            raise ValueError("angle_increment must be positive")
        if range_sigma < 0:
            raise ValueError("range_sigma must be non-negative")
        if len(mount_offset) != 2:
            raise ValueError("mount_offset must be a 2D point")

        self.max_range = float(max_range)
        self.fov = float(fov)
        self.range_sigma = float(range_sigma)
        self.mount_offset = (float(mount_offset[0]), float(mount_offset[1]))
        self.angle_offset = float(angle_offset)

        rays = max(1, round(fov / angle_increment))
        self.angle_increment = fov / rays
        self.angles = (
            np.arange(rays) * self.angle_increment
            - 0.5 * fov
            + self.angle_offset
        )
        self._rng = np.random.default_rng(seed)

    @property
    def ray_count(self) -> int:
        return len(self.angles)

    def scan(self, world: World, pose: Pose) -> LidarScan:
        """Cast every ray for a robot `pose` and return the range measurements."""
        cos_t, sin_t = math.cos(pose.theta), math.sin(pose.theta)
        ox = pose.x + cos_t * self.mount_offset[0] - sin_t * self.mount_offset[1]
        oy = pose.y + sin_t * self.mount_offset[0] + cos_t * self.mount_offset[1]

        world_angles = self.angles + pose.theta
        ranges = np.empty(self.ray_count, dtype=float)

        for index, angle in enumerate(world_angles):
            dx, dy = math.cos(angle), math.sin(angle)
            distance = _ray_boundary(ox, oy, dx, dy, world.width, world.height)
            for obstacle in world.obstacles:
                hit = _ray_rectangle(ox, oy, dx, dy, obstacle)
                if hit is not None and hit < distance:
                    distance = hit

            if self.range_sigma > 0 and distance < self.max_range:
                distance += self._rng.normal(0.0, self.range_sigma)
                distance = float(np.clip(distance, 0.0, self.max_range))

            ranges[index] = min(distance, self.max_range)

        return LidarScan(
            ranges=ranges,
            angles=self.angles.copy(),
            world_angles=world_angles,
            origin=(ox, oy),
            max_range=self.max_range,
        )
