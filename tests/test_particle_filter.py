import math

import numpy as np
import pytest

from atlas.localization import ParticleFilter
from atlas.planning import OccupancyGrid
from atlas.robot.differential_drive import Pose
from atlas.sensors import Lidar
from atlas.simulation.world import RectObstacle, World


def demo_world():
    return World(20, 20, [
        RectObstacle(6, 0, 7, 14),
        RectObstacle(10, 9, 17, 10),
        RectObstacle(13, 2, 15, 4),
    ])


def demo_grid(resolution=0.25):
    return OccupancyGrid.from_world(demo_world(), resolution=resolution, inflate_radius=0.0)


def position_error(estimate, truth):
    return math.hypot(estimate.x - truth.x, estimate.y - truth.y)


def test_initialize_scatters_around_the_prior():
    pf = ParticleFilter(demo_grid(), n_particles=400, seed=1)
    pf.initialize(Pose(3.0, 4.0, 0.5), position_std=0.2, heading_std=0.1)
    assert pf.particles.shape == (400, 3)
    assert np.allclose(pf.weights, 1.0 / 400)
    assert pf.particles[:, 0].mean() == pytest.approx(3.0, abs=0.05)
    assert pf.particles[:, 1].mean() == pytest.approx(4.0, abs=0.05)
    assert pf.effective_sample_size == pytest.approx(400)


def test_zero_std_initialization_is_exact():
    pf = ParticleFilter(demo_grid(), n_particles=50, seed=1)
    pf.initialize(Pose(3.0, 4.0, 0.5), position_std=0.0, heading_std=0.0)
    assert np.allclose(pf.particles, [3.0, 4.0, 0.5])
    estimate = pf.estimate()
    assert (estimate.x, estimate.y, estimate.theta) == pytest.approx((3.0, 4.0, 0.5))


def test_predict_advances_a_straight_line():
    pf = ParticleFilter(demo_grid(), n_particles=100, sigma_v=0.0, sigma_w=0.0,
                        alpha_v=0.0, alpha_w=0.0, seed=1)
    pf.initialize(Pose(2.0, 2.0, 0.0), position_std=0.0, heading_std=0.0)
    for _ in range(20):
        pf.predict(0.5, 0.0, 0.05)
    estimate = pf.estimate()
    assert estimate.x == pytest.approx(2.5)
    assert estimate.y == pytest.approx(2.0)
    assert estimate.theta == pytest.approx(0.0)


def test_predict_follows_a_circular_arc():
    pf = ParticleFilter(demo_grid(), n_particles=100, sigma_v=0.0, sigma_w=0.0,
                        alpha_v=0.0, alpha_w=0.0, seed=1)
    pf.initialize(Pose(5.0, 5.0, 0.0), position_std=0.0, heading_std=0.0)
    v, w, dt = 0.5, 1.0, 0.05
    for _ in range(20):
        pf.predict(v, w, dt)
    radius = v / w
    angle = w * dt * 20
    assert pf.estimate().x == pytest.approx(5.0 + radius * math.sin(angle), abs=1e-9)
    assert pf.estimate().y == pytest.approx(5.0 + radius * (1 - math.cos(angle)), abs=1e-9)


def test_estimate_uses_a_circular_heading_mean():
    pf = ParticleFilter(demo_grid(), n_particles=2, seed=1)
    pf.initialize(Pose(1.0, 1.0, 0.0), position_std=0.0, heading_std=0.0)
    pf.particles[:, 2] = [math.pi - 0.1, -math.pi + 0.1]
    pf.weights = np.array([0.5, 0.5])
    heading = pf.estimate().theta
    assert abs(abs(heading) - math.pi) < 0.05  # not 0, which a naive mean would give


def test_update_keeps_the_estimate_near_the_true_pose():
    world = demo_world()
    grid = demo_grid()
    truth = Pose(5.0, 5.0, 0.3)
    lidar = Lidar(max_range=10.0, range_sigma=0.02, seed=2)
    pf = ParticleFilter(grid, n_particles=500, max_beams=60,
                        likelihood_scale=0.2, seed=2)
    pf.initialize(Pose(5.4, 4.7, 0.45), position_std=0.3, heading_std=0.15)

    for _ in range(15):
        pf.update(lidar.scan(world, truth))

    estimate = pf.estimate()
    assert position_error(estimate, truth) < 0.35
    assert abs(estimate.theta - truth.theta) < math.radians(5.0)


def test_an_informative_scan_lowers_the_effective_sample_size():
    world = demo_world()
    lidar = Lidar(max_range=10.0, range_sigma=0.0)
    good = lidar.scan(world, Pose(5.0, 5.0, 0.0))
    bad = lidar.scan(world, Pose(15.0, 15.0, 0.0))

    def run(scan):
        pf = ParticleFilter(demo_grid(), n_particles=300, seed=3)
        pf.initialize(Pose(5.0, 5.0, 0.0), position_std=0.2, heading_std=0.1)
        pf.update(scan, resample=False)
        return pf.effective_sample_size

    # A scan that matches the prior discriminates between particles; one taken
    # far away penalises them all roughly equally.
    assert run(good) < run(bad)


def test_resample_restores_uniform_weights():
    pf = ParticleFilter(demo_grid(), n_particles=200, seed=4)
    pf.initialize(Pose(5.0, 5.0, 0.0), position_std=0.2, heading_std=0.1)
    pf.weights = pf._rng.random(200)
    pf.weights /= pf.weights.sum()
    pf.resample()
    assert np.allclose(pf.weights, 1.0 / 200)
    assert pf.effective_sample_size == pytest.approx(200)


def test_global_initialization_samples_free_cells():
    grid = demo_grid()
    pf = ParticleFilter(grid, n_particles=300, seed=5)
    pf.initialize_global()
    assert np.all(pf.particles[:, 2] >= -math.pi)
    assert np.all(pf.particles[:, 2] < math.pi)
    cols = np.floor((pf.particles[:, 0] - grid.origin[0]) / grid.resolution).astype(int)
    rows = np.floor((pf.particles[:, 1] - grid.origin[1]) / grid.resolution).astype(int)
    inside = (rows >= 0) & (rows < grid.blocked.shape[0]) & \
             (cols >= 0) & (cols < grid.blocked.shape[1])
    assert inside.all()
    assert not grid.blocked[rows, cols].any()


def test_position_std_is_small_when_particles_agree():
    pf = ParticleFilter(demo_grid(), n_particles=100, seed=6)
    pf.initialize(Pose(2.0, 2.0, 0.0), position_std=0.0, heading_std=0.0)
    std_x, std_y = pf.position_std()
    assert std_x == pytest.approx(0.0, abs=1e-12)
    assert std_y == pytest.approx(0.0, abs=1e-12)


def test_seeded_filters_are_reproducible():
    world = demo_world()
    grid = demo_grid()
    truth = Pose(5.0, 5.0, 0.3)
    scans = [Lidar(max_range=10.0, range_sigma=0.02, seed=7).scan(world, truth)
             for _ in range(5)]

    def run(seed):
        pf = ParticleFilter(grid, n_particles=200, seed=seed)
        pf.initialize(Pose(5.2, 4.9, 0.2), position_std=0.2, heading_std=0.1)
        for scan in scans:
            pf.update(scan)
        return pf.estimate()

    a, b = run(11), run(11)
    assert (a.x, a.y, a.theta) == pytest.approx((b.x, b.y, b.theta))


def test_step_combines_predict_and_update():
    world = demo_world()
    pf = ParticleFilter(demo_grid(), n_particles=200, seed=8)
    pf.initialize(Pose(5.0, 5.0, 0.0), position_std=0.05, heading_std=0.02)
    scan = Lidar(max_range=10.0, range_sigma=0.0).scan(world, Pose(5.05, 5.0, 0.0))
    pf.step(0.1, 0.0, 0.05, scan)
    assert position_error(pf.estimate(), Pose(5.05, 5.0, 0.0)) < 0.2


def test_predict_and_update_require_initialization():
    pf = ParticleFilter(demo_grid(), n_particles=10, seed=9)
    with pytest.raises(RuntimeError, match="initialize"):
        pf.predict(0.1, 0.0, 0.05)
    with pytest.raises(RuntimeError, match="initialize"):
        pf.update(Lidar().scan(demo_world(), Pose()))


def test_invalid_parameters_are_rejected():
    grid = demo_grid()
    with pytest.raises(ValueError, match="n_particles"):
        ParticleFilter(grid, n_particles=0)
    with pytest.raises(ValueError, match="measurement_sigma"):
        ParticleFilter(grid, measurement_sigma=0)
    with pytest.raises(ValueError, match="max_distance"):
        ParticleFilter(grid, max_distance=0)
    with pytest.raises(ValueError, match="max_beams"):
        ParticleFilter(grid, max_beams=0)
    with pytest.raises(ValueError, match="likelihood_scale"):
        ParticleFilter(grid, likelihood_scale=0)
    with pytest.raises(ValueError, match="noise"):
        ParticleFilter(grid, alpha_v=-1)


def test_non_positive_dt_is_rejected():
    pf = ParticleFilter(demo_grid(), n_particles=10, seed=10)
    pf.initialize(Pose())
    with pytest.raises(ValueError, match="dt"):
        pf.predict(0.1, 0.0, 0.0)
