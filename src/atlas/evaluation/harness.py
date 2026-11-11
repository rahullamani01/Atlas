"""Scenario runners and aggregation for the Phase 7 benchmark."""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Sequence

from atlas.control import PID, PathFollower
from atlas.evaluation.metrics import (
    KinematicsMetrics,
    LocalizationMetrics,
    MetricSummary,
    NavigationMetrics,
    PlanningMetrics,
    TrackingMetrics,
    path_errors,
    rms,
    summarize,
)
from atlas.evaluation.scenarios import Scenario, default_scenarios
from atlas.localization import ParticleFilter
from atlas.perception import DynamicObstacleMap
from atlas.planning import OccupancyGrid, a_star, dijkstra
from atlas.recovery import NavigationSupervisor
from atlas.robot.differential_drive import DifferentialDriveRobot, Pose
from atlas.robot.odometry import Odometry
from atlas.sensors import Lidar
from atlas.simulation.simulator import Simulator
from atlas.simulation.world import RectObstacle

WHEEL_SCALE_LEFT = 0.99
WHEEL_SCALE_RIGHT = 1.01
LIDAR_RANGE = 10.0
LIDAR_SIGMA = 0.02
LIKELIHOOD_SCALE = 0.2
TRACKING_BIAS = 0.2


@dataclass(frozen=True)
class ScenarioResult:
    """One scenario run and its flat metric dictionary."""

    scenario: str
    seed: int
    metrics: dict[str, float | int]


def _plan(scenario: Scenario):
    """Build a fresh world and plan the scenario's route."""
    world = scenario.world()
    grid = OccupancyGrid.from_world(
        world,
        resolution=scenario.resolution,
        inflate_radius=scenario.inflation,
    )
    start = grid.world_to_cell(*scenario.start)
    goal = grid.world_to_cell(*scenario.goal)
    result = a_star(grid, start, goal)
    if not result.found:
        raise RuntimeError(f"scenario {scenario.name!r} has no route")
    path = [grid.cell_to_world(cell) for cell in result.path]
    return world, grid, result, path


def _start_pose(path: Sequence[tuple[float, float]]) -> Pose:
    look = min(5, len(path) - 1)
    heading = math.atan2(
        path[look][1] - path[0][1], path[look][0] - path[0][0]
    )
    return Pose(path[0][0], path[0][1], heading)


def run_kinematics(scenario: Scenario, seed: int = 0) -> ScenarioResult:
    """Compare Euler, midpoint and closed-form integration over one circle."""
    v, w, dt = 0.5, 1.0, scenario.dt
    steps = int(round(2.0 * math.pi / (w * dt)))
    total_angle = w * dt * steps
    truth_x = (v / w) * math.sin(total_angle)
    truth_y = (v / w) * (1.0 - math.cos(total_angle))

    errors: dict[str, float] = {}
    for method in ("euler", "midpoint", "exact"):
        robot = DifferentialDriveRobot(pose=Pose())
        for _ in range(steps):
            robot.step(v, w, dt, method=method)
        errors[method] = math.hypot(
            robot.pose.x - truth_x, robot.pose.y - truth_y
        )

    metrics = KinematicsMetrics(
        euler_error=errors["euler"],
        midpoint_error=errors["midpoint"],
        exact_error=errors["exact"],
    )
    return ScenarioResult(scenario.name, seed, metrics.as_dict())


def run_planning(scenario: Scenario, seed: int = 0) -> ScenarioResult:
    """Plan with Dijkstra and A*, and report cost and search effort."""
    world = scenario.world()
    grid = OccupancyGrid.from_world(
        world,
        resolution=scenario.resolution,
        inflate_radius=scenario.inflation,
    )
    start = grid.world_to_cell(*scenario.start)
    goal = grid.world_to_cell(*scenario.goal)

    begin = time.perf_counter()
    dijkstra_result = dijkstra(grid, start, goal)
    dijkstra_ms = (time.perf_counter() - begin) * 1000.0

    begin = time.perf_counter()
    a_star_result = a_star(grid, start, goal)
    a_star_ms = (time.perf_counter() - begin) * 1000.0

    if not dijkstra_result.found or not a_star_result.found:
        raise RuntimeError(f"scenario {scenario.name!r} has no route")
    if not math.isclose(
        dijkstra_result.cost, a_star_result.cost, rel_tol=1e-9, abs_tol=1e-9
    ):
        raise RuntimeError("A* cost does not match Dijkstra; heuristic is not admissible")

    reduction = (
        1.0 - a_star_result.expanded / dijkstra_result.expanded
        if dijkstra_result.expanded
        else 0.0
    )
    metrics = PlanningMetrics(
        path_cost=a_star_result.cost * grid.resolution,
        waypoints=len(a_star_result.path),
        a_star_expanded=a_star_result.expanded,
        dijkstra_expanded=dijkstra_result.expanded,
        expansion_reduction=reduction,
        a_star_ms=a_star_ms,
        dijkstra_ms=dijkstra_ms,
    )
    return ScenarioResult(scenario.name, seed, metrics.as_dict())


def _heading_pid(controller: str) -> PID:
    if controller == "pid":
        return PID(kp=2.0, ki=0.2, kd=0.05)
    return PID(kp=2.0)


def run_tracking(scenario: Scenario, seed: int = 0) -> ScenarioResult:
    """Drive the route with a P or PID heading controller under a yaw bias."""
    world, grid, _, path = _plan(scenario)
    robot = DifferentialDriveRobot(pose=_start_pose(path))
    follower = PathFollower(heading_pid=_heading_pid(scenario.controller))
    sim = Simulator(world, robot, dt=scenario.dt, method="midpoint")

    points = [(robot.pose.x, robot.pose.y)]
    poses: list[tuple[float, float, float]] = []
    commands: list[tuple[float, float]] = []

    for _ in range(scenario.max_steps):
        command = follower.compute(robot.pose, path, scenario.dt)
        ok = sim.step(command.v, command.w + scenario.yaw_bias)
        points.append((robot.pose.x, robot.pose.y))
        poses.append(
            (robot.pose.x, robot.pose.y, robot.pose.theta)
        )
        commands.append((command.v, command.w))
        if not ok or follower.goal_reached:
            break

    distances, heading_errors = path_errors(path, poses)
    driven_length = float(sum(
        math.hypot(bx - ax, by - ay)
        for (ax, ay), (bx, by) in zip(points, points[1:])
    ))
    metrics = TrackingMetrics(
        final_error=math.hypot(
            robot.pose.x - path[-1][0], robot.pose.y - path[-1][1]
        ),
        cross_track_rms=rms(distances),
        cross_track_max=max(distances),
        heading_rms_deg=math.degrees(rms(heading_errors)),
        control_effort=float(
            sum(abs(w) for _, w in commands) * scenario.dt
        ),
        driven_length=driven_length,
        duration=len(commands) * scenario.dt,
        collisions=sim.collision_count,
        reached=follower.goal_reached,
    )
    return ScenarioResult(scenario.name, seed, metrics.as_dict())


def run_localization(
    scenario: Scenario,
    seed: int = 0,
    particles: int = 800,
    beams: int = 60,
) -> ScenarioResult:
    """Drive the route on the particle-filter estimate under wheel-scale error."""
    world, grid, _, path = _plan(scenario)
    robot = DifferentialDriveRobot(pose=_start_pose(path))
    follower = PathFollower(heading_pid=_heading_pid("pid"))
    sim = Simulator(world, robot, dt=scenario.dt, method="midpoint")
    lidar = Lidar(
        max_range=LIDAR_RANGE, range_sigma=LIDAR_SIGMA, seed=seed
    )
    odometry = Odometry(
        left_scale=WHEEL_SCALE_LEFT, right_scale=WHEEL_SCALE_RIGHT
    )
    odometry.reset(Pose(robot.pose.x, robot.pose.y, robot.pose.theta))
    pf = ParticleFilter(
        grid,
        n_particles=particles,
        max_beams=beams,
        likelihood_scale=LIKELIHOOD_SCALE,
        seed=seed,
    )
    pf.initialize(robot.pose, position_std=0.10, heading_std=0.05)

    odometry_errors: list[float] = []
    filter_errors: list[float] = []
    heading_errors: list[float] = []

    for _ in range(scenario.max_steps):
        estimate = pf.estimate()
        command = follower.compute(estimate, path, scenario.dt)
        wheel_l, wheel_r = robot.twist_to_wheel_speeds(
            command.v, command.w
        )
        ok = sim.step(command.v, command.w)

        odometry.update_wheels(wheel_l, wheel_r, scenario.dt)
        v_hat, w_hat = odometry.wheel_speeds_to_twist(wheel_l, wheel_r)
        pf.predict(v_hat, w_hat, scenario.dt)
        pf.update(lidar.scan(world, robot.pose))

        estimate = pf.estimate()
        odometry_errors.append(math.hypot(
            odometry.pose.x - robot.pose.x,
            odometry.pose.y - robot.pose.y,
        ))
        filter_errors.append(math.hypot(
            estimate.x - robot.pose.x, estimate.y - robot.pose.y
        ))
        heading_errors.append(abs(
            math.atan2(
                math.sin(estimate.theta - robot.pose.theta),
                math.cos(estimate.theta - robot.pose.theta),
            )
        ))

        if not ok or follower.goal_reached:
            break

    metrics = LocalizationMetrics(
        odometry_rms=rms(odometry_errors),
        odometry_max=max(odometry_errors),
        filter_rms=rms(filter_errors),
        filter_max=max(filter_errors),
        filter_heading_rms_deg=math.degrees(rms(heading_errors)),
        collisions=sim.collision_count,
        reached=follower.goal_reached,
    )
    return ScenarioResult(scenario.name, seed, metrics.as_dict())


def run_recovery(scenario: Scenario, seed: int = 0) -> ScenarioResult:
    """Run the supervisor on ground-truth pose with an optional mid-run block."""
    world, grid, _, path = _plan(scenario)
    robot = DifferentialDriveRobot(pose=_start_pose(path))
    sim = Simulator(world, robot, dt=scenario.dt, method="midpoint")
    lidar = Lidar(
        max_range=LIDAR_RANGE, range_sigma=LIDAR_SIGMA, seed=seed
    )
    detector = DynamicObstacleMap(grid, clearance=scenario.inflation)
    goal_cell = grid.world_to_cell(*scenario.goal)
    supervisor = NavigationSupervisor(grid, goal_cell, detector=detector)
    if not supervisor.plan(robot.pose):
        raise RuntimeError(f"scenario {scenario.name!r} has no route")

    executed = 0
    for step in range(scenario.max_steps):
        if scenario.inject_step is not None and step == scenario.inject_step:
            world.obstacles.append(RectObstacle(*scenario.inject_rect))
        command = supervisor.step(
            robot.pose, lidar.scan(world, robot.pose), scenario.dt
        )
        ok = sim.step(command.v, command.w)
        executed += 1
        if not ok or supervisor.goal_reached or supervisor.failed:
            break

    goal_world = grid.cell_to_world(goal_cell)
    metrics = NavigationMetrics(
        reached=supervisor.goal_reached,
        failed=supervisor.failed,
        collisions=sim.collision_count,
        duration=executed * scenario.dt,
        final_error=math.hypot(
            robot.pose.x - goal_world[0], robot.pose.y - goal_world[1]
        ),
        replans=supervisor.replan_count,
        detected_cells=len(supervisor.detected_cells),
    )
    return ScenarioResult(scenario.name, seed, metrics.as_dict())


def run_scenario(scenario: Scenario, seed: int = 0) -> ScenarioResult:
    """Dispatch a scenario to its runner."""
    if scenario.kind == "kinematics":
        return run_kinematics(scenario, seed=seed)
    if scenario.kind == "planning":
        return run_planning(scenario, seed=seed)
    if scenario.kind == "tracking":
        return run_tracking(scenario, seed=seed)
    if scenario.kind == "localization":
        return run_localization(scenario, seed=seed)
    if scenario.kind == "recovery":
        return run_recovery(scenario, seed=seed)
    raise ValueError(f"unknown scenario kind: {scenario.kind!r}")


def run_suite(
    scenarios: Sequence[Scenario] | None = None,
    seeds: Sequence[int] = (0, 1, 2),
) -> list[ScenarioResult]:
    """Run every scenario for every seed, in suite order."""
    chosen = list(scenarios) if scenarios is not None else default_scenarios()
    return [
        run_scenario(scenario, seed=seed)
        for scenario in chosen
        for seed in seeds
    ]


def aggregate(
    results: Sequence[ScenarioResult],
) -> dict[str, dict[str, MetricSummary]]:
    """Group metrics by scenario and summarize each one across seeds."""
    grouped: dict[str, dict[str, list[float]]] = {}
    for result in results:
        metrics = grouped.setdefault(result.scenario, {})
        for name, value in result.metrics.items():
            metrics.setdefault(name, []).append(float(value))
    return {
        scenario: {
            name: summarize(values) for name, values in metrics.items()
        }
        for scenario, metrics in grouped.items()
    }
