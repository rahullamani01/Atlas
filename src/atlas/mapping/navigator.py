"""Mapping-driven navigation: plan on the map the robot is building (Phase 10).

`MappingNavigator` closes a mapping, planning and control loop. It integrates
each scan into a `LogOddsMap`, replans when new evidence blocks the current
route or clears a phantom obstacle, and follows the resulting path with the
existing `PathFollower`.

The navigator deliberately does not own the particle filter or the simulator.
The caller supplies the pose estimate and advances the filter, which keeps the
data flow the same as the other evaluation runners: the navigator never reads
ground truth.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from atlas.control import PID, PathFollower, Twist
from atlas.mapping.occupancy_map import LogOddsMap
from atlas.planning import OccupancyGrid, a_star
from atlas.planning.grid import Cell
from atlas.recovery import nearest_free_cell
from atlas.robot.differential_drive import Pose
from atlas.sensors.lidar import LidarScan


@dataclass(frozen=True)
class MapEvent:
    """A timestamped mapping or planning decision, for logging and plotting."""

    step: int
    kind: str
    detail: str


class MappingNavigator:
    """Follow a route and repair the map as new scans arrive."""

    def __init__(
        self,
        occupancy_map: LogOddsMap,
        goal_cell: Cell,
        inflation: float = 0.30,
        follower: PathFollower | None = None,
        replan_interval: int = 15,
    ) -> None:
        if inflation < 0:
            raise ValueError("inflation must be non-negative")
        if replan_interval < 1:
            raise ValueError("replan_interval must be positive")

        self.map = occupancy_map
        self.goal_cell = goal_cell
        self.inflation = float(inflation)
        self.follower = follower or PathFollower(
            heading_pid=PID(kp=2.0, ki=0.2, kd=0.05)
        )
        self.replan_interval = int(replan_interval)

        self.path: list[tuple[float, float]] = []
        self.events: list[MapEvent] = []
        self.plan_count = 0
        self.replan_count = 0
        self.map_updates = 0
        self.added_cells: set[Cell] = set()
        self.removed_cells: set[Cell] = set()
        self.map_version = 0
        self.failed = False

        self._step = 0
        self._last_plan_step = -10**9
        self._arrived = False
        self._cached_grid: OccupancyGrid | None = None
        self._cached_version = -1

    # ---- status ----

    @property
    def goal_reached(self) -> bool:
        return self.follower.goal_reached

    def planning_grid(self) -> OccupancyGrid:
        """Current occupancy map inflated for planning, cached by version."""
        if (
            self._cached_grid is None
            or self._cached_version != self.map_version
        ):
            self._cached_grid = self.map.to_grid(self.inflation)
            self._cached_version = self.map_version
        return self._cached_grid

    # ---- planning ----

    def _plan(self, pose: Pose) -> bool:
        grid = self.planning_grid()
        if not grid.is_free(self.goal_cell):
            return False
        start = nearest_free_cell(
            grid, grid.world_to_cell(pose.x, pose.y)
        )
        if start is None:
            return False
        result = a_star(grid, start, self.goal_cell)
        if not result.found:
            return False
        self.path = [grid.cell_to_world(cell) for cell in result.path]
        self.follower.reset()
        self._last_plan_step = self._step
        return True

    def plan(self, pose: Pose) -> bool:
        """Initial plan. Returns False if the goal is unreachable."""
        if not self._plan(pose):
            self._log("no_path", "initial plan failed")
            return False
        self.plan_count += 1
        self._log("plan", f"{len(self.path)} waypoints")
        return True

    def replan(self, pose: Pose) -> bool:
        """Replan after a map change, keeping the old path on failure."""
        if not self._plan(pose):
            self._log("no_path", "planner found no route")
            return False
        self.replan_count += 1
        self._log("replan", f"{len(self.path)} waypoints")
        return True

    # ---- main loop ----

    def step(self, pose: Pose, scan: LidarScan, dt: float) -> Twist:
        """Integrate the scan, repair the route if needed, and command motion."""
        if dt <= 0:
            raise ValueError("dt must be positive")
        self._step += 1

        update = self.map.integrate(scan, pose)
        if update.changed:
            self.map_updates += 1
            self.map_version += 1
            self.added_cells.update(update.added)
            self.removed_cells.update(update.removed)
            self._log(
                "map",
                f"+{len(update.added)} -{len(update.removed)} cells",
            )

        if update.added and self._path_blocked():
            if not self.replan(pose):
                self.failed = True
                return Twist()
        elif (
            update.removed
            and self._step - self._last_plan_step >= self.replan_interval
        ):
            # A phantom obstacle disappeared; the old route is valid but may be
            # needlessly long. Replanning is best effort, so a failure keeps the
            # current path.
            self.replan(pose)

        if self.failed:
            return Twist()
        if not self.path:
            if not self.replan(pose):
                self.failed = True
                return Twist()

        if self.follower.goal_reached:
            if not self._arrived:
                self._arrived = True
                self._log("arrived", "goal reached")
            return Twist()

        return self.follower.compute(pose, self.path, dt)

    # ---- internals ----

    def _path_blocked(self) -> bool:
        if not self.path:
            return True
        grid = self.planning_grid()
        return any(
            not grid.is_free(grid.world_to_cell(x, y))
            for x, y in self.path
        )

    def _log(self, kind: str, detail: str) -> None:
        self.events.append(MapEvent(self._step, kind, detail))


def route_length(path: list[tuple[float, float]]) -> float:
    """Sum of segment lengths of a world-space polyline."""
    return float(sum(
        math.hypot(bx - ax, by - ay)
        for (ax, ay), (bx, by) in zip(path, path[1:])
    ))
