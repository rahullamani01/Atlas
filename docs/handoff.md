# Handoff — Project ATLAS

**Date:** 10 November 2026
**Branch:** `main` (clean, synced with `origin/main`)
**Latest commit:** `0618959` Phase 2: add planning demo and documentation
**Tests:** 73 passing (`pytest`)
**Repository:** https://github.com/rahullamani01/Atlas

---

## 1. Where the project stands

| Phase | Scope | State |
|---|---|---|
| 0 | Geometry utilities, conventions, scaffold | complete |
| 1 | Differential-drive kinematics and motion simulation | complete |
| 2 | Occupancy grids, Dijkstra, A* path planning | complete |
| 3 | Motion control and trajectory tracking | next |

The system can currently model a differential-drive robot, simulate its motion
in a 2D world with obstacles, rasterise that world into an occupancy grid, and
plan an optimal collision-free route between two cells.

It **cannot yet follow the route**. That is Phase 3.

---

## 2. What exists

### Phase 0 — geometry (`src/atlas/geometry.py`)
- `deg_to_rad`, `rad_to_deg`, `euclidean_distance`
- `wrap_angle`, `angle_difference`
- `rotation_matrix`, `robot_to_world`, `world_to_robot`

### Phase 1 — kinematics (`src/atlas/robot/differential_drive.py`)
- `Pose(x, y, theta)` and `DifferentialDriveRobot`
- Robot parameters: `wheel_radius=0.05`, `wheel_separation=0.30`,
  `body_radius=0.20`, `max_linear_speed=1.0`, `max_angular_speed=2.0`
- `wheel_speeds_to_twist(omega_l, omega_r) -> (v, w)`
- `twist_to_wheel_speeds(v, w) -> (omega_l, omega_r)`
- `step(v, w, dt, method="midpoint")` with `euler`, `midpoint`, `exact`
- `step_wheels(omega_l, omega_r, dt, method="midpoint")`
- Construction validates radii and speed limits.

### Phase 1 — simulation (`src/atlas/simulation/`)
- `World(width, height, obstacles)` with `in_collision(x, y, radius)`
- `RectObstacle` with circle-vs-rectangle checks
- `Simulator(world, robot, dt, method)`:
  - `step(v, w)`, `step_wheels(omega_l, omega_r)` return `True` if collision-free
  - reverts the pose and counts a collision if the move is blocked
  - `trajectory()` returns `(N, 4)` array of time, x, y, theta

### Phase 2 — planning (`src/atlas/planning/`)
- `OccupancyGrid(width, height, resolution, origin, blocked)`
  - `world_to_cell`, `cell_to_world`, `in_bounds`, `is_free`
  - `set_blocked`, `free_count`, `free_ratio`
  - `from_world(world, resolution, inflate_radius)` rasterises obstacles
  - `inflated(radius)` returns a conservative copy grown by the robot radius
  - `neighbors(cell, connectivity=8, allow_corner_cutting=False)`
- `SearchResult(path, cost, expanded, expanded_cells)` with `.found`
- `dijkstra(grid, start, goal, connectivity=8, ...)`
- `a_star(grid, start, goal, heuristic="octile", ...)`
- Heuristics: `zero`, `manhattan` (4-connected only), `euclidean`, `octile`

### Experiments
- `scripts/phase1_demo.py` — straight line, rotation, integrator error table,
  circle and collision plots.
- `scripts/phase2_demo.py` — 80x80 inflated map, Dijkstra vs A*, saves
  `experiments/plots/phase2_planning.png`.

### Documentation
- `docs/mathematics.md` — pose conventions, forward/inverse kinematics,
  nonholonomic constraint, integrators, ICC, error measurements, occupancy
  grids, inflation, graph search, heuristic admissibility.
- `docs/design_decisions.md` — all conventions and deliberate simplifications.
- `README.md` — status, layout, setup, demos, usage snippets.

---

## 3. Verified numbers

- 73 tests pass.
- Phase 1 demo: straight line reaches exactly `x = 5.000`; 180 degree rotation;
  at `dt = 0.05 s` midpoint error over one circle is `5.2e-5 m` vs Euler
  `2.5e-2 m`.
- Phase 2 demo (80x80 grid, 0.25 m cells, 0.20 m inflation):
  - free space 90.4%
  - both planners return a **29.02 m** path
  - Dijkstra expands 4,465 cells (~12 ms)
  - A* expands 1,074 cells (~6 ms), a **76% reduction**
  - A* with octile is optimal and matches Dijkstra's cost exactly

---

## 4. Conventions (do not break)

- Angles are radians internally; wrap to `[-pi, pi)`.
- `theta = 0` is +x; counter-clockwise is positive.
- `omega_r > omega_l` turns counter-clockwise.
- Positive wheel speeds drive forward.
- Grid storage is `blocked[row, col]`; row is y, column is x.
- Cell centres are at `origin + (index + 0.5) * resolution`.
- Planning defaults to 8-connectivity, diagonal cost `sqrt(2)`, no corner
  cutting.
- Search endpoints must be free; invalid input raises `ValueError`.
- Pose from the simulator is ground truth (no noise until Phase 5).

---

## 5. How to run

```bash
cd ~/Documents/Atlas
source .venv/bin/activate
pytest
python scripts/phase1_demo.py
python scripts/phase2_demo.py
```

Repository workflow used so far:
- Author identity: `rahullamaniiii01-create <rahullamaniiii01@gmail.com>`
  (repo-local config; do not revert to the placeholder).
- Commits are split by concern, implementation first and docs/demo second.
- Each phase updates `README.md`, `docs/mathematics.md`, and
  `docs/design_decisions.md`.
- `experiments/plots/` is gitignored; generated plots are not committed.

---

## 6. Known limitations and gotchas

1. **No path following yet.** A planned grid path is a list of cell centres;
   nothing converts it into motion commands.
2. **Grid paths cut visually close to corners.** Inflation gives 0.20 m
   clearance plus a conservative half-cell, but a controller that cuts corners
   with a long lookahead can still clip an inflated obstacle.
3. **Collision is discrete.** `Simulator.step` checks only the post-step pose.
   At high speed the body can tunnel through a thin obstacle. Keep demo speeds
   at or below 0.5 m/s with `dt = 0.05` until swept collision is added.
4. **Velocity saturation is at the twist level**, not per wheel, so the wheel
   mapping can request wheel speeds beyond any real actuator limit.
5. **No wheel dynamics.** Wheel speeds are commands, not states; there is no
   acceleration limit or slip model yet.
6. **Paths are dense.** The Phase 2 demo path has 103 waypoints for 29 m. A
   controller can follow them directly, but nearest-point searches are linear
   in waypoint count.
7. **No dynamic replanning.** The grid is static; that is Phase 6.
8. **No metrics module.** Tracking metrics currently belong in the Phase 3
   demo; a reusable `evaluation/` module arrives in Phase 7.

---

## 7. Phase 3 — Motion control (next stage)

### Goal
Make the simulated robot follow the planned path using feedback, with
measurable tracking error. Expected output from the roadmap: a robot that
follows a planned path and corrects its movement using feedback.

### Scope (in order)
1. Closed-loop control loop over `Simulator`.
2. Proportional heading control.
3. Full PID heading control with anti-windup and output limits.
4. Velocity control with slowdown near the goal.
5. Trajectory tracking metrics.
6. Comparative experiment: P-only vs PID.

Do **not** add sensors, localization, or learning in this phase.

### Proposed files

```text
src/atlas/control/
├── __init__.py          # exports
├── pid.py               # generic PID
└── path_follower.py     # carrot-chasing path follower

tests/
├── test_pid.py
└── test_path_follower.py

scripts/phase3_demo.py   # closed-loop tracking experiment
```

### Proposed API

```python
@dataclass
class Twist:
    v: float   # m/s
    w: float   # rad/s


class PID:
    def __init__(self, kp, ki=0.0, kd=0.0, output_limit=None,
                 integral_limit=None):
        ...
    def update(self, error: float, dt: float) -> float: ...
    def reset(self) -> None: ...


class PathFollower:
    def __init__(
        self,
        lookahead: float = 0.5,
        max_linear_speed: float = 0.5,
        max_angular_speed: float = 1.5,
        goal_tolerance: float = 0.1,
        distance_gain: float = 0.8,
        min_speed: float = 0.05,
        heading_pid: PID | None = None,
    ):
        ...
    def compute(self, pose: Pose, path: list[tuple[float, float]],
                dt: float) -> Twist: ...
    @property
    def goal_reached(self) -> bool: ...
```

Grid paths are converted before use:

```python
waypoints = [grid.cell_to_world(cell) for cell in result.path]
```

### Control law

Carrot chasing on the path, with PID on heading error:

```text
target  = first path point at least `lookahead` metres ahead of the robot
alpha   = wrap(atan2(target_y - y, target_x - x) - theta)
w       = clamp(PID_heading.update(alpha, dt), ±max_angular_speed)
v       = clamp(distance_gain * distance_to_goal * cos(alpha),
                min_speed, max_linear_speed)
```

Stop when `distance_to_goal < goal_tolerance` by commanding `(0, 0)`.

PID:

```text
u = kp e + ki ∫e dt + kd de/dt
```

Discrete form: `integral += e * dt` (clamped), derivative
`(e - e_prev) / dt`, then clamp the output. Freeze integration while the output
is saturated (conditional anti-windup).

Pure pursuit curvature (`omega = 2 v sin(alpha) / lookahead`) can be added as a
second strategy later; start with PID because the phase is about feedback.

### Metrics to record
- Final position error [m]
- RMS and maximum cross-track error to the polyline [m]
- RMS heading error [rad]
- Path length driven vs planned
- Time to goal [s]
- Collision count (must be 0)
- Control effort (sum of |omega| dt), useful for P vs PID comparison

### Test plan (~18 tests)
- PID proportional output equals `kp * error`.
- Integral accumulates and eliminates steady-state error on a simple plant.
- Derivative reacts to error rate and is zero for constant error.
- Output and integral clamps hold.
- Anti-windup prevents overshoot after saturation.
- `reset` clears integral/previous error.
- Non-positive `dt` raises.
- Follower drives straight when already aligned.
- Follower turns toward a path offset to its left/right (sign check).
- Follower slows or stops when facing away from the carrot.
- Follower reports `goal_reached` inside tolerance and commands zero.
- Follower respects speed limits.
- Lookahead advances to later waypoints as the robot moves.
- Closed-loop test: Phase 2 demo map, start to goal, no collisions, final
  error < 0.15 m, cross-track RMS < 0.1 m.
- P-only vs PID: PID has lower RMS heading error or lower settling time on the
  same path.

### Demo
`scripts/phase3_demo.py`:
1. Build the Phase 2 world and grid, plan with A*.
2. Convert cells to world waypoints.
3. Drive the robot with `PathFollower` through `Simulator` at `dt = 0.05`.
4. Stop on goal tolerance, collision, or a step cap.
5. Print metrics and save a plot of planned path, driven trajectory, and
   cross-track/heading error over time.
6. Compare P-only and PID in the same run.

### Acceptance criteria
- All tests pass (target ~91 total).
- Robot reaches the goal without collision on the demo map.
- Final position error < 0.15 m.
- Cross-track RMS < 0.1 m on the demo run.
- Control commands always respect speed limits.
- README, mathematics and design decisions updated to Phase 3.

### Order of work
1. `pid.py` + `test_pid.py`.
2. `path_follower.py` + unit tests for geometry/signs.
3. Closed-loop integration test against the Phase 2 map.
4. `phase3_demo.py` with P vs PID comparison and plot.
5. Documentation and README update.
6. Commit implementation and docs separately, then push.

---

## 8. Immediate next action

Start Phase 3 at `src/atlas/control/pid.py`, then build the follower. Keep the
existing conventions and do not modify Phase 0-2 behaviour; the current suite is
the regression baseline.
