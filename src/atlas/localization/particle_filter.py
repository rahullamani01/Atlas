"""Particle filter for 2D pose estimation on a known occupancy grid (Phase 5).

Each particle is a pose hypothesis. Motion is sampled from a noisy unicycle
model driven by odometry; the LiDAR scan is scored against a precomputed
distance-to-obstacle field. Low-variance systematic odometry errors are corrected
by resampling toward hypotheses that explain the measurements.
"""
from __future__ import annotations

import math

import numpy as np

from atlas.geometry import wrap_angle
from atlas.localization.likelihood_field import DistanceField
from atlas.planning.grid import OccupancyGrid
from atlas.robot.differential_drive import Pose
from atlas.sensors.lidar import LidarScan


def _wrap_array(angles: np.ndarray) -> np.ndarray:
    return (angles + math.pi) % (2.0 * math.pi) - math.pi


class ParticleFilter:
    """A weighted set of pose hypotheses updated by odometry and LiDAR."""

    def __init__(
        self,
        grid: OccupancyGrid,
        n_particles: int = 400,
        measurement_sigma: float = 0.15,
        max_distance: float = 1.0,
        alpha_v: float = 0.05,
        alpha_w: float = 0.10,
        sigma_v: float = 0.01,
        sigma_w: float = 0.01,
        max_beams: int = 45,
        likelihood_scale: float = 0.2,
        resample_position_noise: float = 0.005,
        resample_heading_noise: float = 0.003,
        seed: int | None = None,
    ) -> None:
        if n_particles < 1:
            raise ValueError("n_particles must be positive")
        if measurement_sigma <= 0:
            raise ValueError("measurement_sigma must be positive")
        if max_distance <= 0:
            raise ValueError("max_distance must be positive")
        if max_beams < 1:
            raise ValueError("max_beams must be positive")
        if likelihood_scale <= 0:
            raise ValueError("likelihood_scale must be positive")
        if min(alpha_v, alpha_w, sigma_v, sigma_w) < 0:
            raise ValueError("noise parameters must be non-negative")

        self.grid = grid
        self.n_particles = n_particles
        self.measurement_sigma = measurement_sigma
        self.max_distance = max_distance
        self.alpha_v = alpha_v
        self.alpha_w = alpha_w
        self.sigma_v = sigma_v
        self.sigma_w = sigma_w
        self.max_beams = max_beams
        self.likelihood_scale = likelihood_scale
        self.resample_position_noise = resample_position_noise
        self.resample_heading_noise = resample_heading_noise

        self.field = DistanceField.from_grid(
            grid, include_boundary=True, boundary_only=True
        )
        self._rng = np.random.default_rng(seed)
        self.particles = np.zeros((n_particles, 3))
        self.weights = np.full(n_particles, 1.0 / n_particles)
        self._initialized = False
        self._ess = float(n_particles)

    # ---- setup ----

    def initialize(
        self,
        pose: Pose,
        position_std: float = 0.20,
        heading_std: float = 0.10,
    ) -> None:
        """Scatter particles around a known starting pose."""
        if position_std < 0 or heading_std < 0:
            raise ValueError("standard deviations must be non-negative")
        n = self.n_particles
        self.particles = np.column_stack((
            pose.x + self._rng.normal(0.0, position_std, n),
            pose.y + self._rng.normal(0.0, position_std, n),
            _wrap_array(pose.theta + self._rng.normal(0.0, heading_std, n)),
        ))
        self.weights = np.full(n, 1.0 / n)
        self._initialized = True
        self._ess = float(n)

    def initialize_global(self) -> None:
        """Sample poses uniformly over free cells with uniform headings."""
        free = np.argwhere(~self.grid.blocked)
        if len(free) == 0:
            raise ValueError("grid has no free cells")
        n = self.n_particles
        indices = self._rng.integers(0, len(free), n)
        rows, cols = free[indices, 0], free[indices, 1]
        ox, oy = self.grid.origin
        resolution = self.grid.resolution
        self.particles = np.column_stack((
            ox + (cols + self._rng.random(n)) * resolution,
            oy + (rows + self._rng.random(n)) * resolution,
            self._rng.uniform(-math.pi, math.pi, n),
        ))
        self.weights = np.full(n, 1.0 / n)
        self._initialized = True
        self._ess = float(n)

    # ---- filtering ----

    def predict(self, v: float, w: float, dt: float) -> None:
        """Propagate every particle through a noisy unicycle motion model."""
        if not self._initialized:
            raise RuntimeError("call initialize() before predict()")
        if dt <= 0:
            raise ValueError("dt must be positive")

        n = self.n_particles
        v_hat = v + self._rng.normal(0.0, self.sigma_v + self.alpha_v * abs(v), n)
        w_hat = w + self._rng.normal(0.0, self.sigma_w + self.alpha_w * abs(w), n)

        theta = self.particles[:, 2]
        d_theta = w_hat * dt
        theta_new = _wrap_array(theta + d_theta)

        x = self.particles[:, 0]
        y = self.particles[:, 1]
        straight = np.abs(w_hat) < 1e-6

        ratio = np.zeros(n)
        turning = ~straight
        ratio[turning] = v_hat[turning] / w_hat[turning]

        x = x + np.where(
            turning,
            ratio * (np.sin(theta + d_theta) - np.sin(theta)),
            v_hat * np.cos(theta) * dt,
        )
        y = y + np.where(
            turning,
            -ratio * (np.cos(theta + d_theta) - np.cos(theta)),
            v_hat * np.sin(theta) * dt,
        )

        self.particles = np.column_stack((x, y, theta_new))

    def update(self, scan: LidarScan, resample: bool = True) -> None:
        """Reweight particles by how well they explain the scan."""
        if not self._initialized:
            raise RuntimeError("call initialize() before update()")

        valid = np.flatnonzero(scan.hit_mask)
        if len(valid) == 0:
            return
        if len(valid) > self.max_beams:
            picks = np.linspace(0, len(valid) - 1, self.max_beams).round().astype(int)
            valid = valid[picks]

        ranges = scan.ranges[valid]
        angles = scan.angles[valid]
        body = np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles)))
        body += np.asarray(scan.body_origin)

        theta = self.particles[:, 2]
        cos_t = np.cos(theta)[:, None]
        sin_t = np.sin(theta)[:, None]
        xs = self.particles[:, 0:1] + cos_t * body[:, 0] - sin_t * body[:, 1]
        ys = self.particles[:, 1:2] + sin_t * body[:, 0] + cos_t * body[:, 1]

        points = np.stack((xs, ys), axis=2)
        distances = self.field.sample(points.reshape(-1, 2)).reshape(xs.shape)
        distances = np.minimum(distances, self.max_distance)

        log_likelihood = -0.5 * self.likelihood_scale * np.sum(
            (distances / self.measurement_sigma) ** 2, axis=1
        )
        log_likelihood -= np.max(log_likelihood)
        self.weights *= np.exp(log_likelihood)

        total = float(np.sum(self.weights))
        if total <= 0 or not math.isfinite(total):
            self.weights = np.full(self.n_particles, 1.0 / self.n_particles)
        else:
            self.weights /= total
        self._ess = 1.0 / float(np.sum(self.weights ** 2))

        if resample and self._ess < 0.5 * self.n_particles:
            self.resample()

    def resample(self) -> None:
        """Systematic resampling with a small roughening jitter."""
        n = self.n_particles
        positions = (self._rng.random() + np.arange(n)) / n
        cumulative = np.cumsum(self.weights)
        cumulative[-1] = 1.0
        indices = np.searchsorted(cumulative, positions)
        self.particles = self.particles[indices].copy()

        if self.resample_position_noise > 0:
            self.particles[:, 0] += self._rng.normal(
                0.0, self.resample_position_noise, n
            )
            self.particles[:, 1] += self._rng.normal(
                0.0, self.resample_position_noise, n
            )
        if self.resample_heading_noise > 0:
            self.particles[:, 2] = _wrap_array(
                self.particles[:, 2]
                + self._rng.normal(0.0, self.resample_heading_noise, n)
            )

        self.weights = np.full(n, 1.0 / n)
        self._ess = float(n)

    def step(self, v: float, w: float, dt: float, scan: LidarScan) -> None:
        """Convenience: predict one step and apply the measurement update."""
        self.predict(v, w, dt)
        self.update(scan)

    def replace_grid(self, grid: OccupancyGrid) -> None:
        """Swap the map used by the measurement model and rebuild the field.

        Mapping changes the occupancy grid while the robot drives. The filter
        must score scans against the current map, otherwise newly discovered
        obstacles keep looking like measurement outliers and the likelihood
        stays stale. Particle states and weights are preserved; only the map and
        its derived distance field change.
        """
        if grid.width <= 0 or grid.height <= 0:
            raise ValueError("grid must have positive dimensions")
        self.grid = grid
        self.field = DistanceField.from_grid(
            grid, include_boundary=True, boundary_only=True
        )

    # ---- estimates ----

    def estimate(self) -> Pose:
        """Weighted mean pose; heading uses a circular mean."""
        if not self._initialized:
            raise RuntimeError("call initialize() first")
        x = float(np.sum(self.weights * self.particles[:, 0]))
        y = float(np.sum(self.weights * self.particles[:, 1]))
        sin_sum = float(np.sum(self.weights * np.sin(self.particles[:, 2])))
        cos_sum = float(np.sum(self.weights * np.cos(self.particles[:, 2])))
        return Pose(x, y, math.atan2(sin_sum, cos_sum))

    def position_std(self) -> tuple[float, float]:
        """Weighted marginal standard deviations of x and y."""
        mean = self.estimate()
        var_x = float(np.sum(self.weights * (self.particles[:, 0] - mean.x) ** 2))
        var_y = float(np.sum(self.weights * (self.particles[:, 1] - mean.y) ** 2))
        return math.sqrt(var_x), math.sqrt(var_y)

    def scan_residual(self, scan: LidarScan, pose: Pose | None = None) -> float:
        """Mean distance from projected scan endpoints to mapped obstacles.

        A small residual means the scan is consistent with the map at ``pose``
        (the estimate by default). A large residual means the pose hypothesis
        does not explain what the sensor sees, which is the signal a
        kidnapped-robot monitor needs. Distances are clipped at
        ``max_distance`` and use the same beam subset as ``update``.
        """
        if not self._initialized:
            raise RuntimeError("call initialize() first")

        valid = np.flatnonzero(scan.hit_mask)
        if len(valid) == 0:
            return 0.0
        if len(valid) > self.max_beams:
            picks = np.linspace(
                0, len(valid) - 1, self.max_beams
            ).round().astype(int)
            valid = valid[picks]

        ranges = scan.ranges[valid]
        angles = scan.angles[valid]
        body = np.column_stack((
            ranges * np.cos(angles), ranges * np.sin(angles)
        ))
        body += np.asarray(scan.body_origin)

        reference = self.estimate() if pose is None else pose
        cos_t, sin_t = math.cos(reference.theta), math.sin(reference.theta)
        xs = reference.x + cos_t * body[:, 0] - sin_t * body[:, 1]
        ys = reference.y + sin_t * body[:, 0] + cos_t * body[:, 1]

        distances = self.field.sample(np.column_stack((xs, ys)))
        distances = np.minimum(distances, self.max_distance)
        return float(np.mean(distances))

    @property
    def effective_sample_size(self) -> float:
        return self._ess
