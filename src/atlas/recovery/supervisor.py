"""Reactive navigation: detect, replan, and recover when the route changes.

The supervisor wraps a path follower with a dynamic obstacle map and a small
state machine. In normal driving it follows the current path. When LiDAR
confirms cells that block that path it replans immediately. If no route exists,
it stops and rotates to gather more evidence, then tries again, and finally
backs up a short distance before declaring failure. Failure is an explicit stop,
never a collision.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

from atlas.control import PID, PathFollower, Twist
from atlas.perception import DynamicObstacleMap
from atlas.planning import OccupancyGrid, a_star
from atlas.planning.grid import Cell
from atlas.robot.differential_drive import Pose
from atlas.sensors.lidar import LidarScan


class NavState(str, Enum):
    DRIVING = "driving"
    SCANNING = "scanning"
    BACKING_UP = "backing_up"
    ARRIVED = "arrived"
    FAILED = "failed"


@dataclass(frozen=True)
class NavEvent:
    """A timestamped transition or discovery, for logging and plotting."""

    step: int
    kind: str
    detail: str


def nearest_free_cell(
    grid: OccupancyGrid, cell: Cell, max_radius: int = 12
) -> Cell | None:
    """Return the closest free cell to `cell`, searching outward ring by ring."""
    if grid.is_free(cell):
        return cell

    col0, row0 = cell
    for radius in range(1, max_radius + 1):
        best: Cell | None = None
        best_distance = math.inf
        for dc in range(-radius, radius + 1):
            for dr in range(-radius, radius + 1):
                if max(abs(dc), abs(dr)) != radius:
                    continue
                candidate = (col0 + dc, row0 + dr)
                if not grid.is_free(candidate):
                    continue
                distance = dc * dc + dr * dr
                if distance < best_distance:
                    best_distance = distance
                    best = candidate
        if best is not None:
            return best
    return None


class NavigationSupervisor:
    """Follow a path, replan around new obstacles, and recover from failure."""

    def __init__(
        self,
        grid: OccupancyGrid,
        goal_cell: Cell,
        follower: PathFollower | None = None,
        detector: DynamicObstacleMap | None = None,
        scan_steps: int = 30,
        scan_speed: float = 1.0,
        backup_steps: int = 20,
        backup_speed: float = -0.15,
        stall_window: int = 120,
        stall_distance: float = 0.10,
    ) -> None:
        if not grid.is_free(goal_cell):
            raise ValueError("goal_cell must be free")
        if scan_steps < 1:
            raise ValueError("scan_steps must be positive")
        if scan_speed <= 0:
            raise ValueError("scan_speed must be positive")
        if backup_steps < 1:
            raise ValueError("backup_steps must be positive")
        if backup_speed >= 0:
            raise ValueError("backup_speed must be negative")
        if stall_window < 1:
            raise ValueError("stall_window must be positive")
        if stall_distance < 0:
            raise ValueError("stall_distance must be non-negative")

        self.known_grid = grid
        self.map = detector or DynamicObstacleMap(grid)
        self.goal_cell = goal_cell
        self.follower = follower or PathFollower(
            heading_pid=PID(kp=2.0, ki=0.2, kd=0.05)
        )
        self.scan_steps = scan_steps
        self.scan_speed = scan_speed
        self.backup_steps = backup_steps
        self.backup_speed = backup_speed
        self.stall_window = stall_window
        self.stall_distance = stall_distance

        self.state = NavState.DRIVING
        self.path: list[tuple[float, float]] = []
        self.events: list[NavEvent] = []
        self.plan_count = 0
        self.replan_count = 0
        self.detected_cells: set[Cell] = set()

        self._step = 0
        self._state_step = 0
        self._progress_pose: Pose | None = None
        self._progress_step = 0

    # ---- status ----

    @property
    def goal_reached(self) -> bool:
        return self.state is NavState.ARRIVED

    @property
    def failed(self) -> bool:
        return self.state is NavState.FAILED

    @property
    def recovering(self) -> bool:
        return self.state in (NavState.SCANNING, NavState.BACKING_UP)

    # ---- planning ----

    def _plan(self, pose: Pose) -> bool:
        start = nearest_free_cell(
            self.map.grid, self.map.grid.world_to_cell(pose.x, pose.y)
        )
        if start is None:
            return False
        result = a_star(self.map.grid, start, self.goal_cell)
        if not result.found:
            return False
        self.path = [self.map.grid.cell_to_world(cell) for cell in result.path]
        self.follower.reset()
        self._reset_progress(pose)
        return True

    def plan(self, pose: Pose) -> bool:
        """Initial plan from the current pose. Returns False if unreachable."""
        if not self._plan(pose):
            return False
        self.plan_count += 1
        self._log("plan", f"{len(self.path)} waypoints")
        return True

    def replan(self, pose: Pose) -> bool:
        """Replan after a map change. Returns False if unreachable."""
        if not self._plan(pose):
            self._log("no_path", "planner found no route")
            return False
        self.replan_count += 1
        self._log("replan", f"{len(self.path)} waypoints")
        return True

    def resume(self, pose: Pose) -> bool:
        """Resume driving after an external pose correction.

        Cells confirmed in the old frame are untrustworthy after a pose jump,
        so the dynamic map is cleared and the route is replanned from ``pose``.
        Returns False if no route exists; the caller may keep trying.
        """
        self.map.reset()
        self.detected_cells.clear()
        self.path = []
        self.follower.reset()
        if not self.replan(pose):
            return False
        self._enter(NavState.DRIVING, "pose reacquired")
        return True

    def advance_clock(self, steps: int = 1) -> None:
        """Advance the step counter while the supervisor is not driving.

        The supervisor is paused during relocalization. Counting those steps
        keeps its event timestamps aligned with the caller's control loop.
        """
        if steps < 0:
            raise ValueError("steps must be non-negative")
        self._step += steps

    # ---- main loop ----

    def step(self, pose: Pose, scan: LidarScan, dt: float) -> Twist:
        """Update the map and return the next velocity command."""
        if dt <= 0:
            raise ValueError("dt must be positive")
        self._step += 1

        new_cells = self.map.integrate(scan, pose)
        if new_cells:
            self.detected_cells.update(new_cells)
            self._log("detect", f"{len(new_cells)} cells")
            if self.state is NavState.DRIVING and self._path_blocked():
                if not self.replan(pose):
                    self._enter(NavState.SCANNING, "path blocked")
                    return Twist()

        if self.state in (NavState.ARRIVED, NavState.FAILED):
            return Twist()

        if self.state is NavState.DRIVING:
            if self.follower.goal_reached:
                self._enter(NavState.ARRIVED, "goal reached")
                return Twist()
            if not self.path and not self.replan(pose):
                self._enter(NavState.SCANNING, "no route")
                return Twist()
            if self._stalled(pose):
                self._enter(NavState.SCANNING, "no progress")
                return Twist()
            return self.follower.compute(pose, self.path, dt)

        if self.state is NavState.SCANNING:
            self._state_step += 1
            if self._state_step >= self.scan_steps:
                if self.replan(pose):
                    self._enter(NavState.DRIVING, "route found")
                else:
                    self._enter(NavState.BACKING_UP, "still blocked")
                return Twist()
            return Twist(0.0, self.scan_speed)

        if self.state is NavState.BACKING_UP:
            if not self._backup_clear(pose):
                self._enter(NavState.FAILED, "backup blocked")
                return Twist()
            self._state_step += 1
            if self._state_step >= self.backup_steps:
                if self.replan(pose):
                    self._enter(NavState.DRIVING, "route found after backup")
                else:
                    self._enter(NavState.FAILED, "no route after backup")
                return Twist()
            return Twist(self.backup_speed, 0.0)

        return Twist()

    # ---- internals ----

    def _path_blocked(self) -> bool:
        if not self.path:
            return True
        return any(
            not self.map.grid.is_free(self.map.grid.world_to_cell(x, y))
            for x, y in self.path
        )

    def _stalled(self, pose: Pose) -> bool:
        if self._progress_pose is None:
            self._reset_progress(pose)
            return False
        if self._step - self._progress_step < self.stall_window:
            return False
        moved = math.hypot(
            pose.x - self._progress_pose.x, pose.y - self._progress_pose.y
        )
        self._reset_progress(pose)
        return moved < self.stall_distance

    def _reset_progress(self, pose: Pose) -> None:
        self._progress_pose = Pose(pose.x, pose.y, pose.theta)
        self._progress_step = self._step

    def _backup_clear(self, pose: Pose) -> bool:
        look = 0.25
        x = pose.x - look * math.cos(pose.theta)
        y = pose.y - look * math.sin(pose.theta)

        grid = self.map.grid
        ox, oy = grid.origin
        xmax = ox + grid.width * grid.resolution
        ymax = oy + grid.height * grid.resolution
        margin = 0.25
        if not (ox + margin <= x <= xmax - margin
                and oy + margin <= y <= ymax - margin):
            return False
        return grid.is_free(grid.world_to_cell(x, y))

    def _enter(self, state: NavState, detail: str) -> None:
        self.state = state
        self._state_step = 0
        self._log(state.value, detail)

    def _log(self, kind: str, detail: str) -> None:
        self.events.append(NavEvent(self._step, kind, detail))
