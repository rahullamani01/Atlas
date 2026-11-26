"""Scenario runners and aggregation for the Phase 7 benchmark."""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Sequence

import numpy as np

from atlas.control import PID, PathFollower, Twist
from atlas.evaluation.metrics import (
    ExplorationMetrics,
    IntegratedMetrics,
    KinematicsMetrics,
    LocalizationMetrics,
    MappingMetrics,
    MetricSummary,
    NavigationMetrics,
    OnlineSlamMetrics,
    PlanningMetrics,
    RelocalizationMetrics,
    SlamMetrics,
    TrackingMetrics,
    path_errors,
    rms,
    summarize,
)
from atlas.evaluation.scenarios import Scenario, default_scenarios
from atlas.exploration import select_viewpoint
from atlas.localization import LocalizationMonitor, ParticleFilter, Relocalizer
from atlas.mapping import (
    LogOddsMap,
    MapEvent,
    MappingNavigator,
    map_error,
    repair_fraction,
)
from atlas.perception import DynamicObstacleMap
from atlas.planning import OccupancyGrid, a_star, dijkstra
from atlas.recovery import NavEvent, NavigationSupervisor, nearest_free_cell
from atlas.robot.differential_drive import DifferentialDriveRobot, Pose
from atlas.robot.odometry import Odometry
from atlas.sensors import Lidar
from atlas.simulation.simulator import Simulator
from atlas.simulation.world import RectObstacle
from atlas.slam import LoopEvent, PoseGraphSlam

WHEEL_SCALE_LEFT = 0.99
WHEEL_SCALE_RIGHT = 1.01
LIDAR_RANGE = 10.0
LIDAR_SIGMA = 0.02
LIKELIHOOD_SCALE = 0.2
TRACKING_BIAS = 0.2

RELOCALIZE_STEPS = 40
RELOCALIZE_SETTLE_STEPS = 10
RELOCALIZE_SPEED = 0.6
RESIDUAL_LIMIT = 0.5
RESIDUAL_PATIENCE = 8
RECOVERY_TOLERANCE = 0.5
SLAM_SPEED = 0.4


@dataclass(frozen=True)
class ScenarioResult:
    """One scenario run and its flat metric dictionary."""

    scenario: str
    seed: int
    metrics: dict[str, float | int]


@dataclass(frozen=True)
class IntegratedTrace:
    """Detailed record of one estimate-driven navigation run."""

    metrics: IntegratedMetrics
    trajectory: list[tuple[float, float, float]]
    odometry_errors: list[float]
    filter_errors: list[float]
    heading_errors: list[float]
    detected_counts: list[int]
    events: list[NavEvent]
    detected_cells: set[tuple[int, int]]


@dataclass(frozen=True)
class RelocalizationTrace:
    """Detailed record of one kidnapped-robot recovery run."""

    metrics: RelocalizationMetrics
    trajectory: list[tuple[float, float, float]]
    filter_errors: list[float]
    residuals: list[float]
    events: list[NavEvent]
    detected_cells: set[tuple[int, int]]


@dataclass(frozen=True)
class MappingTrace:
    """Detailed record of one mapping-and-repair run."""

    metrics: MappingMetrics
    trajectory: list[tuple[float, float, float]]
    filter_errors: list[float]
    map_errors: list[int]
    events: list[MapEvent]
    prior_grid: OccupancyGrid
    final_grid: OccupancyGrid
    truth_grid: OccupancyGrid
    added_cells: set[tuple[int, int]]
    removed_cells: set[tuple[int, int]]


@dataclass(frozen=True)
class SlamTrace:
    """Detailed record of one pose-graph SLAM run."""

    metrics: SlamMetrics
    trajectory: list[tuple[float, float, float]]
    filter_errors: list[float]
    node_raw: list[Pose]
    node_corrected: list[Pose]
    node_truth: list[Pose]
    loops: list[LoopEvent]
    cost_history: list[float]
    prior_grid: OccupancyGrid
    raw_grid: OccupancyGrid
    slam_grid: OccupancyGrid
    truth_grid: OccupancyGrid


@dataclass(frozen=True)
class OnlineSlamTrace:
    """Detailed record of one online SLAM run with a mid-run correction."""

    metrics: OnlineSlamMetrics
    trajectory: list[tuple[float, float, float]]
    filter_errors: list[float]
    node_pre: list[Pose]
    node_post: list[Pose]
    node_truth: list[Pose]
    loops: list[LoopEvent]
    cost_history: list[float]
    closure_step: int
    prior_grid: OccupancyGrid
    raw_grid: OccupancyGrid
    slam_grid: OccupancyGrid
    final_grid: OccupancyGrid
    truth_grid: OccupancyGrid


@dataclass(frozen=True)
class ExplorationTrace:
    """Detailed record of one next-best-view exploration run."""

    metrics: ExplorationMetrics
    trajectory: list[tuple[float, float, float]]
    filter_errors: list[float]
    heading_errors: list[float]
    coverage_history: list[tuple[int, float]]
    truth_coverage_history: list[tuple[int, float]]
    viewpoints: list[tuple[int, int]]
    events: list[MapEvent]
    prior_grid: OccupancyGrid
    final_grid: OccupancyGrid
    truth_grid: OccupancyGrid


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
    *,
    beam_selection: str = "scan_order",
) -> ScenarioResult:
    """Drive the route on the particle-filter estimate under wheel-scale error."""
    world, grid, _, path = _plan(scenario)
    robot = DifferentialDriveRobot(pose=_start_pose(path))
    follower = PathFollower(heading_pid=_heading_pid("pid"))
    sim = Simulator(world, robot, dt=scenario.dt, method="midpoint")
    lidar = Lidar(
        max_range=LIDAR_RANGE, range_sigma=scenario.lidar_sigma, seed=seed
    )
    odometry = Odometry(
        left_scale=scenario.wheel_scale_left,
        right_scale=scenario.wheel_scale_right,
    )
    odometry.reset(Pose(robot.pose.x, robot.pose.y, robot.pose.theta))
    pf = ParticleFilter(
        grid,
        n_particles=particles,
        max_beams=beams,
        beam_selection=beam_selection,
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
        max_range=LIDAR_RANGE, range_sigma=scenario.lidar_sigma, seed=seed
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


def simulate_integrated(
    scenario: Scenario,
    seed: int = 0,
    particles: int = 800,
    beams: int = 60,
) -> IntegratedTrace:
    """Run localization, detection, planning and control as one loop.

    The particle-filter estimate drives the supervisor, and the same estimate
    is used to interpret LiDAR returns for map consistency. This is the Phase 8
    integration test: no component sees ground-truth pose except the simulator
    and the error metrics.
    """
    world, grid, _, path = _plan(scenario)
    robot = DifferentialDriveRobot(pose=_start_pose(path))
    sim = Simulator(world, robot, dt=scenario.dt, method="midpoint")
    lidar = Lidar(max_range=LIDAR_RANGE, range_sigma=scenario.lidar_sigma, seed=seed)
    odometry = Odometry(
        left_scale=scenario.wheel_scale_left,
        right_scale=scenario.wheel_scale_right,
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
    detector = DynamicObstacleMap(grid, clearance=scenario.inflation)
    goal_cell = grid.world_to_cell(*scenario.goal)
    supervisor = NavigationSupervisor(grid, goal_cell, detector=detector)
    if not supervisor.plan(pf.estimate()):
        raise RuntimeError(f"scenario {scenario.name!r} has no initial route")

    trajectory = [(robot.pose.x, robot.pose.y, robot.pose.theta)]
    odometry_errors: list[float] = []
    filter_errors: list[float] = []
    heading_errors: list[float] = []
    detected_counts: list[int] = []
    executed = 0

    for step in range(scenario.max_steps):
        if scenario.inject_step is not None and step == scenario.inject_step:
            world.obstacles.append(RectObstacle(*scenario.inject_rect))

        estimate = pf.estimate()
        scan = lidar.scan(world, robot.pose)
        command = supervisor.step(estimate, scan, scenario.dt)
        detected_counts.append(len(supervisor.detected_cells))
        ok = sim.step(command.v, command.w)
        executed += 1
        trajectory.append((robot.pose.x, robot.pose.y, robot.pose.theta))

        wheel_l, wheel_r = robot.twist_to_wheel_speeds(command.v, command.w)
        odometry.update_wheels(wheel_l, wheel_r, scenario.dt)
        v_hat, w_hat = odometry.wheel_speeds_to_twist(wheel_l, wheel_r)
        pf.predict(v_hat, w_hat, scenario.dt)
        pf.update(scan)

        estimate = pf.estimate()
        odometry_errors.append(math.hypot(
            odometry.pose.x - robot.pose.x,
            odometry.pose.y - robot.pose.y,
        ))
        filter_errors.append(math.hypot(
            estimate.x - robot.pose.x,
            estimate.y - robot.pose.y,
        ))
        heading_errors.append(abs(math.atan2(
            math.sin(estimate.theta - robot.pose.theta),
            math.cos(estimate.theta - robot.pose.theta),
        )))

        if not ok or supervisor.goal_reached or supervisor.failed:
            break

    goal_world = grid.cell_to_world(goal_cell)
    metrics = IntegratedMetrics(
        reached=supervisor.goal_reached,
        failed=supervisor.failed,
        collisions=sim.collision_count,
        duration=executed * scenario.dt,
        final_error=math.hypot(
            robot.pose.x - goal_world[0], robot.pose.y - goal_world[1]
        ),
        replans=supervisor.replan_count,
        detected_cells=len(supervisor.detected_cells),
        odometry_rms=rms(odometry_errors),
        odometry_max=max(odometry_errors),
        filter_rms=rms(filter_errors),
        filter_max=max(filter_errors),
        filter_heading_rms_deg=math.degrees(rms(heading_errors)),
    )
    return IntegratedTrace(
        metrics=metrics,
        trajectory=trajectory,
        odometry_errors=odometry_errors,
        filter_errors=filter_errors,
        heading_errors=heading_errors,
        detected_counts=detected_counts,
        events=list(supervisor.events),
        detected_cells=set(supervisor.detected_cells),
    )


def run_integrated(
    scenario: Scenario,
    seed: int = 0,
    particles: int = 800,
    beams: int = 60,
) -> ScenarioResult:
    """Evaluate the estimate-driven navigation loop."""
    trace = simulate_integrated(
        scenario, seed=seed, particles=particles, beams=beams
    )
    return ScenarioResult(scenario.name, seed, trace.metrics.as_dict())


def simulate_relocalization(
    scenario: Scenario,
    seed: int = 0,
    particles: int = 800,
    beams: int = 60,
) -> RelocalizationTrace:
    """Run navigation, then recover from a mid-run pose jump.

    The robot is teleported once, which odometry does not observe. A residual
    monitor declares the filter lost, the robot rotates in place while a
    multi-hypothesis scan matcher accumulates evidence, and the winning pose
    seeds the filter again. The dynamic map is cleared because cells confirmed
    in the old frame are not trustworthy.
    """
    world, grid, _, path = _plan(scenario)
    if scenario.kidnap_pose is not None:
        cell = grid.world_to_cell(
            scenario.kidnap_pose[0], scenario.kidnap_pose[1]
        )
        if not grid.is_free(cell):
            raise RuntimeError("kidnap_pose must be free in the planning grid")

    robot = DifferentialDriveRobot(pose=_start_pose(path))
    sim = Simulator(world, robot, dt=scenario.dt, method="midpoint")
    lidar = Lidar(max_range=LIDAR_RANGE, range_sigma=scenario.lidar_sigma, seed=seed)
    odometry = Odometry(
        left_scale=scenario.wheel_scale_left,
        right_scale=scenario.wheel_scale_right,
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
    detector = DynamicObstacleMap(grid, clearance=scenario.inflation)
    goal_cell = grid.world_to_cell(*scenario.goal)
    supervisor = NavigationSupervisor(grid, goal_cell, detector=detector)
    if not supervisor.plan(pf.estimate()):
        raise RuntimeError(f"scenario {scenario.name!r} has no initial route")

    monitor = LocalizationMonitor(RESIDUAL_LIMIT, RESIDUAL_PATIENCE)
    relocalizer = Relocalizer(grid, max_beams=beams)

    trajectory = [(robot.pose.x, robot.pose.y, robot.pose.theta)]
    filter_errors: list[float] = []
    heading_errors: list[float] = []
    residuals: list[float] = []
    events: list[NavEvent] = []
    post_kidnap: list[float] = []
    post_recovery: list[float] = []
    post_recovery_headings: list[float] = []

    relocalizing = False
    settle = False
    relocalize_step = 0
    settle_step = 0
    relocalizations = 0
    kidnapped = False
    detection_step: int | None = None
    recovery_step: int | None = None
    error_at_kidnap = 0.0
    executed = 0

    for step in range(scenario.max_steps):
        if scenario.kidnap_step is not None and step == scenario.kidnap_step:
            robot.pose = Pose(*scenario.kidnap_pose)
            kidnapped = True
            error_at_kidnap = math.hypot(
                pf.estimate().x - robot.pose.x,
                pf.estimate().y - robot.pose.y,
            )
            trajectory.append(
                (robot.pose.x, robot.pose.y, robot.pose.theta)
            )
            events.append(NavEvent(
                step,
                "kidnap",
                f"teleported to ({robot.pose.x:.2f}, {robot.pose.y:.2f})",
            ))

        scan = lidar.scan(world, robot.pose)
        residual = pf.scan_residual(scan)
        residuals.append(residual)
        skip_filter = False
        finish = False

        if relocalizing:
            relocalizer.observe(
                scan, dtheta=RELOCALIZE_SPEED * scenario.dt
            )
            command = Twist(0.0, RELOCALIZE_SPEED)
            relocalize_step += 1
            supervisor.advance_clock()
            skip_filter = True
            finish = relocalize_step >= RELOCALIZE_STEPS
        elif settle:
            command = Twist(0.0, RELOCALIZE_SPEED)
            settle_step += 1
            supervisor.advance_clock()
        elif kidnapped and monitor.update(residual):
            relocalizer.search(scan)
            relocalizing = True
            relocalize_step = 0
            detection_step = step
            command = Twist()
            skip_filter = True
            supervisor.advance_clock()
            events.append(NavEvent(
                step, "lost", f"scan residual {residual:.2f} m"
            ))
        else:
            command = supervisor.step(pf.estimate(), scan, scenario.dt)

        ok = sim.step(command.v, command.w)
        executed += 1
        trajectory.append((robot.pose.x, robot.pose.y, robot.pose.theta))

        wheel_l, wheel_r = robot.twist_to_wheel_speeds(
            command.v, command.w
        )
        odometry.update_wheels(wheel_l, wheel_r, scenario.dt)

        if finish:
            best = relocalizer.best()
            pf.initialize(best.pose, position_std=0.10, heading_std=0.03)
            relocalizing = False
            settle = True
            settle_step = 0
            relocalizations += 1
            monitor.reset()
            relocalizer.reset()
            events.append(NavEvent(
                step,
                "relocalized",
                f"hypothesis residual {best.score:.3f} m",
            ))
        elif not skip_filter:
            v_hat, w_hat = odometry.wheel_speeds_to_twist(
                wheel_l, wheel_r
            )
            pf.predict(v_hat, w_hat, scenario.dt)
            pf.update(scan)

        if settle and settle_step >= RELOCALIZE_SETTLE_STEPS:
            supervisor.resume(pf.estimate())
            settle = False
            events.append(NavEvent(
                step,
                "resumed",
                "detection and driving restarted",
            ))

        estimate = pf.estimate()
        error = math.hypot(
            estimate.x - robot.pose.x, estimate.y - robot.pose.y
        )
        filter_errors.append(error)
        heading_error = abs(math.atan2(
            math.sin(estimate.theta - robot.pose.theta),
            math.cos(estimate.theta - robot.pose.theta),
        ))
        heading_errors.append(heading_error)
        if kidnapped:
            post_kidnap.append(error)
        if recovery_step is not None:
            post_recovery.append(error)
            post_recovery_headings.append(heading_error)
        elif (
            kidnapped
            and relocalizations > 0
            and error < RECOVERY_TOLERANCE
        ):
            recovery_step = step

        if not ok or supervisor.goal_reached or supervisor.failed:
            break

    goal_world = grid.cell_to_world(goal_cell)
    metrics = RelocalizationMetrics(
        reached=supervisor.goal_reached,
        failed=supervisor.failed,
        collisions=sim.collision_count,
        duration=executed * scenario.dt,
        final_error=math.hypot(
            robot.pose.x - goal_world[0], robot.pose.y - goal_world[1]
        ),
        replans=supervisor.replan_count,
        detected_cells=len(supervisor.detected_cells),
        kidnapped=kidnapped,
        relocalizations=relocalizations,
        relocalized=recovery_step is not None,
        detection_latency=(
            (detection_step - scenario.kidnap_step) * scenario.dt
            if detection_step is not None and scenario.kidnap_step is not None
            else -1.0
        ),
        recovery_time=(
            (recovery_step - scenario.kidnap_step) * scenario.dt
            if recovery_step is not None and scenario.kidnap_step is not None
            else -1.0
        ),
        error_at_kidnap=error_at_kidnap,
        post_kidnap_rms=rms(post_kidnap) if post_kidnap else 0.0,
        post_recovery_rms=(
            rms(post_recovery) if post_recovery else 0.0
        ),
        post_recovery_heading_rms_deg=(
            math.degrees(rms(post_recovery_headings))
            if post_recovery_headings
            else 0.0
        ),
        filter_max=max(filter_errors),
        filter_heading_rms_deg=math.degrees(rms(heading_errors)),
    )
    return RelocalizationTrace(
        metrics=metrics,
        trajectory=trajectory,
        filter_errors=filter_errors,
        residuals=residuals,
        events=sorted(
            list(supervisor.events) + events, key=lambda event: event.step
        ),
        detected_cells=set(supervisor.detected_cells),
    )


def run_relocalization(
    scenario: Scenario,
    seed: int = 0,
    particles: int = 800,
    beams: int = 60,
) -> ScenarioResult:
    """Evaluate kidnapped-robot detection and recovery."""
    trace = simulate_relocalization(
        scenario, seed=seed, particles=particles, beams=beams
    )
    return ScenarioResult(scenario.name, seed, trace.metrics.as_dict())


def simulate_mapping(
    scenario: Scenario,
    seed: int = 0,
    particles: int = 800,
    beams: int = 60,
) -> MappingTrace:
    """Build and repair a map while navigating on the estimate.

    The robot starts from a prior map that may contain phantom obstacles and may
    omit real ones. Every scan is integrated at the particle-filter estimate,
    the planner replans when new evidence blocks the route or clears a phantom,
    and the filter's likelihood field is rebuilt from the current map. Ground
    truth is read only to score pose error and map error.
    """
    truth_world = scenario.world()
    truth_raw = OccupancyGrid.from_world(
        truth_world, resolution=scenario.resolution, boundary=True
    )
    prior_world = (
        scenario.prior_world() if scenario.prior_world is not None
        else scenario.world()
    )
    prior_raw = OccupancyGrid.from_world(
        prior_world, resolution=scenario.resolution, boundary=True
    )

    mapping = LogOddsMap(
        prior_raw,
        max_range=LIDAR_RANGE,
        max_beams=max(beams, 90),
    )
    planning = mapping.to_grid(scenario.inflation)
    start_cell = nearest_free_cell(
        planning, planning.world_to_cell(*scenario.start)
    )
    goal_cell = planning.world_to_cell(*scenario.goal)
    if start_cell is None or not planning.is_free(goal_cell):
        raise RuntimeError("slam scenario needs a free start and goal")
    initial = a_star(planning, start_cell, goal_cell)
    if not initial.found:
        raise RuntimeError(f"scenario {scenario.name!r} has no prior route")
    initial_path = [planning.cell_to_world(cell) for cell in initial.path]

    robot = DifferentialDriveRobot(pose=_start_pose(initial_path))
    sim = Simulator(truth_world, robot, dt=scenario.dt, method="midpoint")
    lidar = Lidar(max_range=LIDAR_RANGE, range_sigma=scenario.lidar_sigma, seed=seed)
    odometry = Odometry(
        left_scale=scenario.wheel_scale_left,
        right_scale=scenario.wheel_scale_right,
    )
    odometry.reset(Pose(robot.pose.x, robot.pose.y, robot.pose.theta))
    pf = ParticleFilter(
        planning,
        n_particles=particles,
        max_beams=beams,
        likelihood_scale=LIKELIHOOD_SCALE,
        seed=seed,
    )
    pf.initialize(robot.pose, position_std=0.10, heading_std=0.05)
    navigator = MappingNavigator(
        mapping, goal_cell, inflation=scenario.inflation,
        map_repair=scenario.map_repair,
    )
    if not navigator.plan(pf.estimate()):
        raise RuntimeError(f"scenario {scenario.name!r} has no prior route")

    trajectory = [(robot.pose.x, robot.pose.y, robot.pose.theta)]
    filter_errors: list[float] = []
    heading_errors: list[float] = []
    map_errors: list[int] = []
    executed = 0
    last_version = -1

    for step in range(scenario.max_steps):
        estimate = pf.estimate()
        scan = lidar.scan(truth_world, robot.pose)
        command = navigator.step(estimate, scan, scenario.dt)
        ok = sim.step(command.v, command.w)
        executed += 1
        trajectory.append((robot.pose.x, robot.pose.y, robot.pose.theta))

        wheel_l, wheel_r = robot.twist_to_wheel_speeds(
            command.v, command.w
        )
        odometry.update_wheels(wheel_l, wheel_r, scenario.dt)
        v_hat, w_hat = odometry.wheel_speeds_to_twist(wheel_l, wheel_r)
        pf.predict(v_hat, w_hat, scenario.dt)
        pf.update(scan)

        if navigator.map_version != last_version:
            pf.replace_grid(navigator.planning_grid())
            last_version = navigator.map_version

        estimate = pf.estimate()
        filter_errors.append(math.hypot(
            estimate.x - robot.pose.x,
            estimate.y - robot.pose.y,
        ))
        heading_errors.append(abs(math.atan2(
            math.sin(estimate.theta - robot.pose.theta),
            math.cos(estimate.theta - robot.pose.theta),
        )))
        if step % 10 == 0:
            map_errors.append(map_error(
                mapping.occupancy,
                truth_raw.blocked,
                mapping.observed,
            ))

        if (
            not ok
            or navigator.goal_reached
            or navigator.failed
        ):
            break

    observed = mapping.observed
    prior_error = map_error(
        prior_raw.blocked, truth_raw.blocked, observed
    )
    final_error_cells = map_error(
        mapping.occupancy, truth_raw.blocked, observed
    )
    reduction = prior_error - final_error_cells
    prior_error_cells, repaired_cells = repair_fraction(
        mapping.occupancy, prior_raw.blocked, truth_raw.blocked, observed
    )
    # Repair is judged only on cells the prior got wrong. Net reduction also
    # counts new cell-level sampling error, so it can be negative even when
    # every observable prior defect was fixed.
    repair_ratio = (
        repaired_cells / prior_error_cells if prior_error_cells > 0 else 1.0
    )

    goal_world = planning.cell_to_world(goal_cell)
    metrics = MappingMetrics(
        reached=navigator.goal_reached,
        failed=navigator.failed or sim.collision_count > 0,
        collisions=sim.collision_count,
        duration=executed * scenario.dt,
        final_error=math.hypot(
            robot.pose.x - goal_world[0],
            robot.pose.y - goal_world[1],
        ),
        replans=navigator.replan_count,
        map_updates=navigator.map_updates,
        added_cells=len(navigator.added_cells),
        removed_cells=len(navigator.removed_cells),
        known_ratio=mapping.known_ratio,
        observed_ratio=mapping.observed_ratio,
        prior_map_error=prior_error,
        final_map_error=final_error_cells,
        map_error_reduction=reduction,
        map_repair_ratio=repair_ratio,
        filter_rms=rms(filter_errors),
        filter_max=max(filter_errors),
        filter_heading_rms_deg=math.degrees(rms(heading_errors)),
    )
    return MappingTrace(
        metrics=metrics,
        trajectory=trajectory,
        filter_errors=filter_errors,
        map_errors=map_errors,
        events=list(navigator.events),
        prior_grid=prior_raw,
        final_grid=mapping.to_grid(0.0),
        truth_grid=truth_raw,
        added_cells=set(navigator.added_cells),
        removed_cells=set(navigator.removed_cells),
    )


def run_mapping(
    scenario: Scenario,
    seed: int = 0,
    particles: int = 800,
    beams: int = 60,
) -> ScenarioResult:
    """Evaluate mapping and map repair during navigation."""
    trace = simulate_mapping(
        scenario, seed=seed, particles=particles, beams=beams
    )
    return ScenarioResult(scenario.name, seed, trace.metrics.as_dict())


def simulate_slam(
    scenario: Scenario,
    seed: int = 0,
    particles: int = 800,
    beams: int = 60,
) -> SlamTrace:
    """Navigate, then correct the run with pose-graph loop closure.

    Navigation is the Phase 10 loop: particle-filter estimate, mapping
    navigator, mapping at the estimate. The back-end records wheel odometry,
    scans and estimates, builds a graph on the odometry chain, verifies loop
    closures against the prior map, optimises, and rebuilds the map from the
    corrected poses. Ground truth is read only to score the result.
    """
    truth_world = scenario.world()
    truth_raw = OccupancyGrid.from_world(
        truth_world, resolution=scenario.resolution, boundary=True
    )
    prior_world = (
        scenario.prior_world() if scenario.prior_world is not None
        else scenario.world()
    )
    prior_raw = OccupancyGrid.from_world(
        prior_world, resolution=scenario.resolution, boundary=True
    )

    mapping = LogOddsMap(
        prior_raw,
        max_range=LIDAR_RANGE,
        max_beams=max(beams, 90),
    )
    planning = mapping.to_grid(scenario.inflation)
    start_cell = nearest_free_cell(
        planning, planning.world_to_cell(*scenario.start)
    )
    goal_cell = planning.world_to_cell(*scenario.goal)
    if start_cell is None or not planning.is_free(goal_cell):
        raise RuntimeError("slam scenario needs a free start and goal")
    initial = a_star(planning, start_cell, goal_cell)
    if not initial.found:
        raise RuntimeError(f"scenario {scenario.name!r} has no prior route")
    initial_path = [planning.cell_to_world(cell) for cell in initial.path]

    robot = DifferentialDriveRobot(pose=_start_pose(initial_path))
    sim = Simulator(truth_world, robot, dt=scenario.dt, method="midpoint")
    lidar = Lidar(max_range=LIDAR_RANGE, range_sigma=scenario.lidar_sigma, seed=seed)
    odometry = Odometry(
        left_scale=scenario.wheel_scale_left,
        right_scale=scenario.wheel_scale_right,
    )
    odometry.reset(Pose(robot.pose.x, robot.pose.y, robot.pose.theta))
    pf = ParticleFilter(
        planning,
        n_particles=particles,
        max_beams=beams,
        likelihood_scale=LIKELIHOOD_SCALE,
        seed=seed,
    )
    pf.initialize(robot.pose, position_std=0.10, heading_std=0.05)
    navigator = MappingNavigator(
        mapping,
        goal_cell,
        inflation=scenario.inflation,
        follower=PathFollower(max_linear_speed=SLAM_SPEED),
        map_repair=scenario.map_repair,
    )
    if not navigator.plan(pf.estimate()):
        raise RuntimeError(f"scenario {scenario.name!r} has no prior route")

    slam = PoseGraphSlam(prior_raw, min_gap=80, max_loops=20)

    trajectory = [(robot.pose.x, robot.pose.y, robot.pose.theta)]
    filter_errors: list[float] = []
    heading_errors: list[float] = []
    executed = 0
    last_version = -1

    for step in range(scenario.max_steps):
        estimate = pf.estimate()
        scan = lidar.scan(truth_world, robot.pose)
        command = navigator.step(estimate, scan, scenario.dt)
        ok = sim.step(command.v, command.w)
        executed += 1
        trajectory.append((robot.pose.x, robot.pose.y, robot.pose.theta))

        wheel_l, wheel_r = robot.twist_to_wheel_speeds(
            command.v, command.w
        )
        odometry.update_wheels(wheel_l, wheel_r, scenario.dt)
        v_hat, w_hat = odometry.wheel_speeds_to_twist(wheel_l, wheel_r)
        pf.predict(v_hat, w_hat, scenario.dt)
        pf.update(scan)

        if navigator.map_version != last_version:
            pf.replace_grid(navigator.planning_grid())
            last_version = navigator.map_version

        slam.observe(step, odometry.pose, estimate, robot.pose, scan)

        estimate = pf.estimate()
        filter_errors.append(math.hypot(
            estimate.x - robot.pose.x,
            estimate.y - robot.pose.y,
        ))
        heading_errors.append(abs(math.atan2(
            math.sin(estimate.theta - robot.pose.theta),
            math.cos(estimate.theta - robot.pose.theta),
        )))

        if not ok or navigator.goal_reached or navigator.failed:
            break

    raw_nodes = slam.corrected()
    raw_errors = slam.node_errors(raw_nodes)
    slam.solve(detect=scenario.loop_closure)
    corrected = slam.corrected()
    slam_errors = slam.node_errors(corrected)

    raw_map = slam.map_from(raw_nodes)
    slam_map = slam.map_from(corrected)
    observed = raw_map.observed & slam_map.observed
    prior_error = map_error(
        prior_raw.blocked, truth_raw.blocked, observed
    )
    raw_error = map_error(
        raw_map.occupancy, truth_raw.blocked, observed
    )
    slam_error = map_error(
        slam_map.occupancy, truth_raw.blocked, observed
    )

    odometry_rms = rms(raw_errors)
    slam_rms = rms(slam_errors)
    reduction_ratio = (
        (odometry_rms - slam_rms) / odometry_rms
        if odometry_rms > 0 else 0.0
    )
    history = slam.cost_history or [0.0]
    goal_world = planning.cell_to_world(goal_cell)

    metrics = SlamMetrics(
        reached=navigator.goal_reached,
        failed=navigator.failed or sim.collision_count > 0,
        collisions=sim.collision_count,
        duration=executed * scenario.dt,
        final_error=math.hypot(
            robot.pose.x - goal_world[0],
            robot.pose.y - goal_world[1],
        ),
        nodes=len(corrected),
        loop_closures=len(slam.loops),
        odometry_rms=odometry_rms,
        odometry_max=max(raw_errors),
        slam_rms=slam_rms,
        slam_max=max(slam_errors),
        slam_final=slam_errors[-1],
        ate_reduction=reduction_ratio,
        graph_cost_before=history[0],
        graph_cost_after=history[-1],
        prior_map_error=prior_error,
        raw_map_error=raw_error,
        slam_map_error=slam_error,
        map_error_reduction=raw_error - slam_error,
        filter_rms=rms(filter_errors),
        filter_heading_rms_deg=math.degrees(rms(heading_errors)),
    )
    return SlamTrace(
        metrics=metrics,
        trajectory=trajectory,
        filter_errors=filter_errors,
        node_raw=raw_nodes,
        node_corrected=corrected,
        node_truth=slam.node_truth,
        loops=list(slam.loops),
        cost_history=list(history),
        prior_grid=prior_raw,
        raw_grid=raw_map.to_grid(0.0),
        slam_grid=slam_map.to_grid(0.0),
        truth_grid=truth_raw,
    )


def run_slam(
    scenario: Scenario,
    seed: int = 0,
    particles: int = 800,
    beams: int = 60,
) -> ScenarioResult:
    """Evaluate pose-graph loop closure over a navigation run."""
    trace = simulate_slam(
        scenario, seed=seed, particles=particles, beams=beams
    )
    return ScenarioResult(scenario.name, seed, trace.metrics.as_dict())


def simulate_online_slam(
    scenario: Scenario,
    seed: int = 0,
    particles: int = 800,
    beams: int = 60,
) -> OnlineSlamTrace:
    """Drive, correct the pose graph when a loop closes, then keep driving.

    The first leg reaches ``scenario.goal``; ``goal_sequence`` supplies extra
    goals so the run can continue after the correction. When a loop closure is
    verified, the graph is optimised mid-run, the live map is rebuilt from the
    corrected poses, odometry is reset to the corrected pose and the filter is
    reseeded there. Navigation then continues on the corrected map. Ground truth
    is read only to score the result.
    """
    truth_world = scenario.world()
    truth_raw = OccupancyGrid.from_world(
        truth_world, resolution=scenario.resolution, boundary=True
    )
    prior_world = (
        scenario.prior_world() if scenario.prior_world is not None
        else scenario.world()
    )
    prior_raw = OccupancyGrid.from_world(
        prior_world, resolution=scenario.resolution, boundary=True
    )

    mapping = LogOddsMap(
        prior_raw,
        max_range=LIDAR_RANGE,
        max_beams=max(beams, 90),
    )
    planning = mapping.to_grid(scenario.inflation)
    start_cell = nearest_free_cell(
        planning, planning.world_to_cell(*scenario.start)
    )
    if start_cell is None:
        raise RuntimeError("online scenario needs a free start")
    goal_cells = []
    for gx, gy in (scenario.goal, *scenario.goal_sequence):
        cell = planning.world_to_cell(gx, gy)
        if not planning.is_free(cell):
            raise RuntimeError(f"scenario {scenario.name!r} has a blocked goal")
        goal_cells.append(cell)
    initial = a_star(planning, start_cell, goal_cells[0])
    if not initial.found:
        raise RuntimeError(f"scenario {scenario.name!r} has no prior route")
    initial_path = [planning.cell_to_world(cell) for cell in initial.path]

    robot = DifferentialDriveRobot(pose=_start_pose(initial_path))
    sim = Simulator(truth_world, robot, dt=scenario.dt, method="midpoint")
    lidar = Lidar(max_range=LIDAR_RANGE, range_sigma=scenario.lidar_sigma, seed=seed)
    odometry = Odometry(
        left_scale=scenario.wheel_scale_left,
        right_scale=scenario.wheel_scale_right,
    )
    odometry.reset(Pose(robot.pose.x, robot.pose.y, robot.pose.theta))
    pf = ParticleFilter(
        planning,
        n_particles=particles,
        max_beams=beams,
        likelihood_scale=LIKELIHOOD_SCALE,
        seed=seed,
    )
    pf.initialize(robot.pose, position_std=0.10, heading_std=0.05)
    navigator = MappingNavigator(
        mapping,
        goal_cells[0],
        inflation=scenario.inflation,
        follower=PathFollower(max_linear_speed=SLAM_SPEED),
        map_repair=scenario.map_repair,
    )
    if not navigator.plan(pf.estimate()):
        raise RuntimeError(f"scenario {scenario.name!r} has no prior route")

    slam = PoseGraphSlam(prior_raw, min_gap=80, max_loops=20)

    trajectory = [(robot.pose.x, robot.pose.y, robot.pose.theta)]
    filter_errors: list[float] = []
    heading_errors: list[float] = []
    executed = 0
    last_version = -1
    goal_index = 0
    closure_step = -1
    closure_time = 0.0
    correction = 0.0
    pre_closure_rms = 0.0
    post_closure_rms = 0.0
    pre_cost = 0.0
    post_cost = 0.0
    pre_nodes: list[Pose] = []
    pre_map: LogOddsMap | None = None
    closure_map: LogOddsMap | None = None
    corrected_once = False

    for step in range(scenario.max_steps):
        estimate = pf.estimate()
        scan = lidar.scan(truth_world, robot.pose)
        command = navigator.step(estimate, scan, scenario.dt)
        ok = sim.step(command.v, command.w)
        executed += 1
        trajectory.append((robot.pose.x, robot.pose.y, robot.pose.theta))

        wheel_l, wheel_r = robot.twist_to_wheel_speeds(
            command.v, command.w
        )
        odometry.update_wheels(wheel_l, wheel_r, scenario.dt)
        v_hat, w_hat = odometry.wheel_speeds_to_twist(wheel_l, wheel_r)
        pf.predict(v_hat, w_hat, scenario.dt)
        pf.update(scan)

        if navigator.map_version != last_version:
            pf.replace_grid(navigator.planning_grid())
            last_version = navigator.map_version

        slam.observe(step, odometry.pose, estimate, robot.pose, scan)

        if (
            scenario.loop_closure
            and not corrected_once
            and step % slam.check_interval == 0
            and len(slam.graph.poses) > slam.min_gap
        ):
            before = len(slam.loops)
            slam.close_loops()
            if len(slam.loops) > before:
                pre_nodes = [
                    Pose(pose.x, pose.y, pose.theta)
                    for pose in slam.node_odometry
                ]
                pre_map = slam.map_from(pre_nodes)
                pre_closure_rms = rms(slam.node_errors(pre_nodes))
                pre_cost = slam.graph.cost()
                slam.solve(detect=False)
                corrected = slam.corrected()
                post_closure_rms = rms(slam.node_errors(corrected))
                post_cost = (
                    slam.cost_history[-1]
                    if slam.cost_history else slam.graph.cost()
                )
                correction = max(
                    math.hypot(
                        before_pose.x - after_pose.x,
                        before_pose.y - after_pose.y,
                    )
                    for before_pose, after_pose in zip(pre_nodes, corrected)
                )
                closure_step = step
                closure_time = step * scenario.dt
                corrected_once = True

                rebuilt = slam.map_from(corrected)
                closure_map = rebuilt
                if scenario.map_repair:
                    mapping = rebuilt
                    navigator.replace_map(rebuilt)
                current = corrected[-1]
                odometry.reset(
                    Pose(current.x, current.y, current.theta)
                )
                pf.replace_grid(navigator.planning_grid())
                pf.initialize(
                    current, position_std=0.05, heading_std=0.02
                )
                navigator.replan(current)
                last_version = navigator.map_version

        estimate = pf.estimate()
        filter_errors.append(math.hypot(
            estimate.x - robot.pose.x,
            estimate.y - robot.pose.y,
        ))
        heading_errors.append(abs(math.atan2(
            math.sin(estimate.theta - robot.pose.theta),
            math.cos(estimate.theta - robot.pose.theta),
        )))

        if not ok or navigator.failed:
            break
        if navigator.goal_reached:
            if goal_index + 1 < len(goal_cells):
                goal_index += 1
                navigator.retarget(goal_cells[goal_index])
                if not navigator.plan(pf.estimate()):
                    navigator.failed = True
                    break
            else:
                break

    final_nodes = slam.corrected()
    final_errors = slam.node_errors(final_nodes)
    if pre_map is None:
        pre_nodes = [
            Pose(pose.x, pose.y, pose.theta)
            for pose in slam.node_odometry
        ]
        pre_map = slam.map_from(pre_nodes)
        pre_closure_rms = rms(slam.node_errors(pre_nodes))
        post_closure_rms = pre_closure_rms
        pre_cost = slam.graph.cost()
        post_cost = pre_cost
    if closure_map is None:
        closure_map = slam.map_from(
            final_nodes[: len(pre_nodes)]
        )
    final_map = slam.map_from(final_nodes)
    observed = pre_map.observed & closure_map.observed
    raw_error = map_error(
        pre_map.occupancy, truth_raw.blocked, observed
    )
    slam_error = map_error(
        closure_map.occupancy, truth_raw.blocked, observed
    )

    slam_rms = rms(final_errors)
    reduction_ratio = (
        (pre_closure_rms - post_closure_rms) / pre_closure_rms
        if pre_closure_rms > 0 else 0.0
    )
    history = slam.cost_history or [post_cost]
    goal_world = truth_raw.cell_to_world(goal_cells[-1])
    reached = (
        goal_index == len(goal_cells) - 1 and navigator.goal_reached
    )

    metrics = OnlineSlamMetrics(
        reached=reached,
        failed=navigator.failed or sim.collision_count > 0,
        collisions=sim.collision_count,
        duration=executed * scenario.dt,
        final_error=math.hypot(
            robot.pose.x - goal_world[0],
            robot.pose.y - goal_world[1],
        ),
        nodes=len(final_nodes),
        loop_closures=len(slam.loops),
        closure_step=closure_step,
        closure_time=closure_time,
        correction=correction,
        pre_closure_rms=pre_closure_rms,
        post_closure_rms=post_closure_rms,
        ate_reduction=reduction_ratio,
        slam_rms=slam_rms,
        slam_final=final_errors[-1],
        graph_cost_before=pre_cost,
        graph_cost_after=history[-1],
        raw_map_error=raw_error,
        slam_map_error=slam_error,
        map_error_reduction=raw_error - slam_error,
        filter_rms=rms(filter_errors),
        filter_heading_rms_deg=math.degrees(rms(heading_errors)),
    )
    return OnlineSlamTrace(
        metrics=metrics,
        trajectory=trajectory,
        filter_errors=filter_errors,
        node_pre=pre_nodes,
        node_post=final_nodes,
        node_truth=slam.node_truth,
        loops=list(slam.loops),
        cost_history=list(history),
        closure_step=closure_step,
        prior_grid=prior_raw,
        raw_grid=pre_map.to_grid(0.0),
        slam_grid=closure_map.to_grid(0.0),
        final_grid=final_map.to_grid(0.0),
        truth_grid=truth_raw,
    )


def run_online_slam(
    scenario: Scenario,
    seed: int = 0,
    particles: int = 800,
    beams: int = 60,
) -> ScenarioResult:
    """Evaluate online pose-graph correction during navigation."""
    trace = simulate_online_slam(
        scenario, seed=seed, particles=particles, beams=beams
    )
    return ScenarioResult(scenario.name, seed, trace.metrics.as_dict())


def simulate_exploration(
    scenario: Scenario,
    seed: int = 0,
    particles: int = 300,
    beams: int = 40,
    spacing: int = 8,
    radius_cells: int = 40,
    min_gain: int = 40,
    coverage_interval: int = 25,
    max_failures: int = 15,
) -> ExplorationTrace:
    """Explore an unknown map by driving to next-best viewpoints.

    The robot starts from an empty prior, so every cell is unknown and no route
    exists to a goal. It repeatedly scores a coarse lattice of candidate cells
    by the unknown area within LiDAR range divided by the A* cost, drives to the
    best one, and re-scores after each arrival. Unknown cells are free to the
    planner, which lets the robot leave the observed region; a target that turns
    out to be blocked is blacklisted and replaced instead of ending the run.

    The run ends when the observed fraction of the map reaches
    ``scenario.coverage_target``. That fraction comes from map state only, so
    the stopping rule needs no ground truth. Ground truth is read afterwards to
    score free-space coverage and map error.
    """
    truth_world = scenario.world()
    truth_raw = OccupancyGrid.from_world(
        truth_world, resolution=scenario.resolution, boundary=True
    )
    prior_world = (
        scenario.prior_world() if scenario.prior_world is not None
        else scenario.world()
    )
    prior_raw = OccupancyGrid.from_world(
        prior_world, resolution=scenario.resolution
    )

    unknown = np.zeros_like(prior_raw.blocked)
    mapping = LogOddsMap(
        prior_raw,
        max_range=LIDAR_RANGE,
        max_beams=max(beams, 240),
        known=unknown.copy(),
    )
    planning = mapping.to_grid(scenario.inflation)
    start_cell = nearest_free_cell(
        planning, planning.world_to_cell(*scenario.start)
    )
    if start_cell is None:
        raise RuntimeError(f"scenario {scenario.name!r} needs a free start")

    robot = DifferentialDriveRobot(
        pose=Pose(*planning.cell_to_world(start_cell), 0.0)
    )
    sim = Simulator(truth_world, robot, dt=scenario.dt, method="midpoint")
    lidar = Lidar(max_range=LIDAR_RANGE, range_sigma=scenario.lidar_sigma, seed=seed)
    odometry = Odometry(
        left_scale=scenario.wheel_scale_left,
        right_scale=scenario.wheel_scale_right,
    )
    odometry.reset(Pose(robot.pose.x, robot.pose.y, robot.pose.theta))
    pf = ParticleFilter(
        planning,
        n_particles=particles,
        max_beams=beams,
        likelihood_scale=LIKELIHOOD_SCALE,
        seed=seed,
    )
    pf.initialize(robot.pose, position_std=0.10, heading_std=0.05)

    scan = lidar.scan(truth_world, robot.pose)
    mapping.integrate(scan, robot.pose)
    pick = select_viewpoint(
        mapping,
        mapping.to_grid(scenario.inflation),
        start_cell,
        spacing=spacing,
        radius=radius_cells,
        min_gain=min_gain,
    )
    if pick is None:
        raise RuntimeError(
            f"scenario {scenario.name!r} starts with no viewpoint"
        )
    navigator = MappingNavigator(
        mapping,
        pick.cell,
        inflation=scenario.inflation,
        follower=PathFollower(max_linear_speed=SLAM_SPEED),
    )
    if not navigator.plan(pf.estimate()):
        raise RuntimeError(f"scenario {scenario.name!r} has no initial route")

    visited: list[tuple[int, int]] = []
    blacklist: list[tuple[int, int]] = []
    viewpoints: list[tuple[int, int]] = [pick.cell]
    trajectory = [(robot.pose.x, robot.pose.y, robot.pose.theta)]
    filter_errors: list[float] = []
    heading_errors: list[float] = []
    coverage_history: list[tuple[int, float]] = []
    truth_coverage_history: list[tuple[int, float]] = []
    executed = 0
    failures = 0
    completed = False
    last_version = -1

    truth_free = np.count_nonzero(~truth_raw.blocked)

    def free_coverage() -> float:
        if not truth_free:
            return 0.0
        observed_free = np.count_nonzero(
            mapping.observed & ~truth_raw.blocked
        )
        return observed_free / truth_free

    def next_viewpoint() -> str:
        """Choose and plan to a fresh viewpoint.

        Returns ``"ok"`` when a route was planned, ``"exhausted"`` when no
        candidate has useful gain, and ``"failed"`` when the chosen candidate
        could not be reached from here.
        """
        grid = navigator.planning_grid()
        here = nearest_free_cell(
            grid, grid.world_to_cell(robot.pose.x, robot.pose.y)
        )
        if here is None:
            return "exhausted"
        choice = select_viewpoint(
            navigator.map,
            grid,
            here,
            spacing=spacing,
            radius=radius_cells,
            min_gain=min_gain,
            visited=visited,
            blacklist=blacklist,
        )
        if choice is None:
            return "exhausted"
        navigator.retarget(choice.cell)
        if not navigator.plan(pf.estimate()):
            blacklist.append(choice.cell)
            return "failed"
        viewpoints.append(choice.cell)
        return "ok"

    for step in range(scenario.max_steps):
        estimate = pf.estimate()
        scan = lidar.scan(truth_world, robot.pose)
        command = navigator.step(estimate, scan, scenario.dt)
        ok = sim.step(command.v, command.w)
        executed += 1
        trajectory.append((robot.pose.x, robot.pose.y, robot.pose.theta))

        wheel_l, wheel_r = robot.twist_to_wheel_speeds(command.v, command.w)
        odometry.update_wheels(wheel_l, wheel_r, scenario.dt)
        v_hat, w_hat = odometry.wheel_speeds_to_twist(wheel_l, wheel_r)
        pf.predict(v_hat, w_hat, scenario.dt)
        pf.update(scan)

        if navigator.map_version != last_version:
            pf.replace_grid(navigator.planning_grid())
            last_version = navigator.map_version

        estimate = pf.estimate()
        filter_errors.append(math.hypot(
            estimate.x - robot.pose.x,
            estimate.y - robot.pose.y,
        ))
        heading_errors.append(abs(math.atan2(
            math.sin(estimate.theta - robot.pose.theta),
            math.cos(estimate.theta - robot.pose.theta),
        )))
        if step % coverage_interval == 0:
            coverage_history.append((step, mapping.observed_ratio))
            truth_coverage_history.append((step, free_coverage()))

        if not ok:
            break
        if mapping.observed_ratio >= scenario.coverage_target:
            completed = True
            break
        if navigator.failed:
            failures += 1
            blacklist.append(navigator.goal_cell)
            if failures > max_failures:
                break
            navigator.failed = False
            if next_viewpoint() == "exhausted":
                break
            continue
        if navigator.goal_reached:
            visited.append(navigator.goal_cell)
            if next_viewpoint() == "exhausted":
                break
            continue

    coverage = mapping.observed_ratio
    free_coverage_final = free_coverage()
    metrics = ExplorationMetrics(
        completed=completed,
        failed=sim.collision_count > 0 or failures > max_failures,
        collisions=sim.collision_count,
        duration=executed * scenario.dt,
        coverage=coverage,
        free_coverage=free_coverage_final,
        map_error=map_error(
            mapping.occupancy, truth_raw.blocked, mapping.observed
        ),
        viewpoints=len(viewpoints),
        steps=executed,
        filter_rms=rms(filter_errors),
        filter_heading_rms_deg=math.degrees(rms(heading_errors)),
    )
    return ExplorationTrace(
        metrics=metrics,
        trajectory=trajectory,
        filter_errors=filter_errors,
        heading_errors=heading_errors,
        coverage_history=coverage_history,
        truth_coverage_history=truth_coverage_history,
        viewpoints=viewpoints,
        events=list(navigator.events),
        prior_grid=prior_raw,
        final_grid=mapping.to_grid(0.0),
        truth_grid=truth_raw,
    )


def run_exploration(
    scenario: Scenario,
    seed: int = 0,
    particles: int = 300,
    beams: int = 40,
) -> ScenarioResult:
    """Evaluate next-best-view exploration on an unknown map."""
    trace = simulate_exploration(
        scenario, seed=seed, particles=particles, beams=beams
    )
    return ScenarioResult(scenario.name, seed, trace.metrics.as_dict())


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
    if scenario.kind == "integrated":
        return run_integrated(scenario, seed=seed)
    if scenario.kind == "relocalization":
        return run_relocalization(scenario, seed=seed)
    if scenario.kind == "slam":
        return run_mapping(scenario, seed=seed)
    if scenario.kind == "slam_loop":
        return run_slam(scenario, seed=seed)
    if scenario.kind == "slam_online":
        return run_online_slam(scenario, seed=seed)
    if scenario.kind == "exploration":
        return run_exploration(scenario, seed=seed)
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
