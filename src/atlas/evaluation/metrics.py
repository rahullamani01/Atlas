"""Reusable metrics for planning, tracking, localization and navigation (Phase 7).

Every function here is plain arithmetic on sequences. Ground truth is passed in
by the caller; no metric reads simulator state, so the same code can score a
demo, a unit test or a future experiment.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np

from atlas.geometry import wrap_angle

Point = tuple[float, float]
PoseTuple = tuple[float, float, float]


def rms(values: Sequence[float]) -> float:
    """Root-mean-square of a non-empty sequence."""
    array = np.asarray(values, dtype=float)
    if array.size == 0:
        raise ValueError("rms requires at least one value")
    return float(np.sqrt(np.mean(array ** 2)))


def segment_projection(point: Point, a: Point, b: Point) -> tuple[float, float]:
    """Return ``(distance, heading)`` for the nearest point on segment ``ab``.

    The distance is measured to the clamped projection, so a point beyond either
    end is scored against that endpoint. ``heading`` is the segment direction in
    world radians.
    """
    px, py = point
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    length_squared = dx * dx + dy * dy
    if length_squared == 0:
        return math.hypot(px - ax, py - ay), math.atan2(dy, dx)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_squared))
    nearest_x = ax + t * dx
    nearest_y = ay + t * dy
    return math.hypot(px - nearest_x, py - nearest_y), math.atan2(dy, dx)


def path_errors(
    path: Sequence[Point], poses: Sequence[PoseTuple]
) -> tuple[list[float], list[float]]:
    """Cross-track distance and heading error for each ``(x, y, theta)`` pose.

    A pose is scored against its nearest path segment. Heading error is wrapped
    to ``[-pi, pi)`` and measured as ``path heading - pose heading``.
    """
    if len(path) < 2:
        raise ValueError("path must contain at least two points")
    distances: list[float] = []
    headings: list[float] = []
    for x, y, theta in poses:
        distance, path_heading = min(
            (segment_projection((x, y), a, b) for a, b in zip(path, path[1:])),
            key=lambda item: item[0],
        )
        distances.append(distance)
        headings.append(wrap_angle(path_heading - theta))
    return distances, headings


@dataclass(frozen=True)
class MetricSummary:
    """Aggregate of one metric across seeds."""

    count: int
    mean: float
    std: float
    minimum: float
    maximum: float

    def as_dict(self) -> dict[str, float | int]:
        return {
            "count": self.count,
            "mean": self.mean,
            "std": self.std,
            "minimum": self.minimum,
            "maximum": self.maximum,
        }


def summarize(values: Sequence[float]) -> MetricSummary:
    """Mean, sample standard deviation, minimum and maximum.

    The standard deviation uses ``ddof=1`` when there is more than one sample,
    so it estimates the spread of the underlying run-to-run distribution. A
    single value reports ``std = 0``.
    """
    array = np.asarray(values, dtype=float)
    if array.size == 0:
        raise ValueError("summarize requires at least one value")
    std = float(np.std(array, ddof=1)) if array.size > 1 else 0.0
    return MetricSummary(
        count=int(array.size),
        mean=float(np.mean(array)),
        std=std,
        minimum=float(np.min(array)),
        maximum=float(np.max(array)),
    )


@dataclass(frozen=True)
class KinematicsMetrics:
    euler_error: float
    midpoint_error: float
    exact_error: float

    def as_dict(self) -> dict[str, float]:
        return {
            "euler_error": self.euler_error,
            "midpoint_error": self.midpoint_error,
            "exact_error": self.exact_error,
        }


@dataclass(frozen=True)
class PlanningMetrics:
    path_cost: float
    waypoints: int
    a_star_expanded: int
    dijkstra_expanded: int
    expansion_reduction: float
    a_star_ms: float
    dijkstra_ms: float

    def as_dict(self) -> dict[str, float | int]:
        return {
            "path_cost": self.path_cost,
            "waypoints": self.waypoints,
            "a_star_expanded": self.a_star_expanded,
            "dijkstra_expanded": self.dijkstra_expanded,
            "expansion_reduction": self.expansion_reduction,
            "a_star_ms": self.a_star_ms,
            "dijkstra_ms": self.dijkstra_ms,
        }


@dataclass(frozen=True)
class TrackingMetrics:
    final_error: float
    cross_track_rms: float
    cross_track_max: float
    heading_rms_deg: float
    control_effort: float
    driven_length: float
    duration: float
    collisions: int
    reached: bool

    def as_dict(self) -> dict[str, float | int]:
        return {
            "final_error": self.final_error,
            "cross_track_rms": self.cross_track_rms,
            "cross_track_max": self.cross_track_max,
            "heading_rms_deg": self.heading_rms_deg,
            "control_effort": self.control_effort,
            "driven_length": self.driven_length,
            "duration": self.duration,
            "collisions": self.collisions,
            "reached": 1.0 if self.reached else 0.0,
        }


@dataclass(frozen=True)
class LocalizationMetrics:
    odometry_rms: float
    odometry_max: float
    filter_rms: float
    filter_max: float
    filter_heading_rms_deg: float
    collisions: int
    reached: bool

    def as_dict(self) -> dict[str, float | int]:
        return {
            "odometry_rms": self.odometry_rms,
            "odometry_max": self.odometry_max,
            "filter_rms": self.filter_rms,
            "filter_max": self.filter_max,
            "filter_heading_rms_deg": self.filter_heading_rms_deg,
            "collisions": self.collisions,
            "reached": 1.0 if self.reached else 0.0,
        }


@dataclass(frozen=True)
class NavigationMetrics:
    reached: bool
    failed: bool
    collisions: int
    duration: float
    final_error: float
    replans: int
    detected_cells: int

    def as_dict(self) -> dict[str, float | int]:
        return {
            "reached": 1.0 if self.reached else 0.0,
            "failed": 1.0 if self.failed else 0.0,
            "collisions": self.collisions,
            "duration": self.duration,
            "final_error": self.final_error,
            "replans": self.replans,
            "detected_cells": self.detected_cells,
        }
