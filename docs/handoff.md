# Handoff — Project ATLAS

**Date:** 10 November 2026
**Branch:** `main` (synced with `origin/main`)
**Latest work:** Phase 5 — odometry and particle-filter localization
**Tests:** 161 passing (`pytest`)
**Repository:** https://github.com/rahullamani01/Atlas

---

## 1. Where the project stands

| Phase | Scope | State |
|---|---|---|
| 0 | Geometry utilities, conventions, scaffold | complete |
| 1 | Differential-drive kinematics and motion simulation | complete |
| 2 | Occupancy grids, Dijkstra, A* path planning | complete |
| 3 | PID control and closed-loop path following | complete |
| 4 | LiDAR simulation and range uncertainty | complete |
| 5 | Odometry and particle-filter localization | complete |
| 6 | Dynamic obstacles and recovery | next |

The system can model a differential-drive robot, simulate its motion in a 2D
world with obstacles, rasterise that world into an occupancy grid, plan an
optimal collision-free route, drive the route with feedback control, sense
surrounding geometry with a simulated 2D LiDAR, and estimate its own pose by
fusing wheel odometry with those scans.

The controller now closes the loop on the **estimated** pose, not ground truth.
It still assumes a static map and a known route, so detecting and driving around
unexpected obstacles is Phase 6.

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

### Phase 3 — control (`src/atlas/control/`)
- `PID(kp, ki, kd, output_limit, integral_limit)`
  - `update(error, dt)`, `reset()`, `integral` property
  - output clamp, integral clamp, conditional anti-windup, zero derivative on
    the first sample
- `PathFollower(lookahead=0.5, max_linear_speed=0.5, max_angular_speed=1.5,
  goal_tolerance=0.1, distance_gain=0.8, min_speed=0.05, heading_pid=None)`
  - `compute(pose, path, dt) -> Twist(v, w)`
  - `lookahead_point(pose, path)`, `goal_reached`, `reset()`
  - carrot chasing on world-space waypoints; PID heading control; turns in
    place when the carrot is behind; stops within goal tolerance

### Phase 4 — sensing (`src/atlas/sensors/`)
- `Lidar(max_range=10.0, fov=2*pi, angle_increment=pi/180, range_sigma=0.0,
  mount_offset=(0,0), angle_offset=0.0, seed=None)`
  - `scan(world, pose) -> LidarScan`
  - analytic ray casting: slab method for rectangles, closed form for walls
  - `ray_count`, body-frame `angles`, actual `angle_increment`
- `LidarScan(ranges, angles, world_angles, origin, max_range)`
  - `hit_mask`, `hits`, `ray_count`
  - `points()` returns world-space hit points
  - a miss is exactly `max_range`, never `inf`/NaN
- Seeded Gaussian range noise on finite hits, clipped to sensor range
- `LidarScan.body_origin` records the mount offset used by the localization
  measurement model

### Phase 5 — localization (`src/atlas/`)
- `Odometry(wheel_radius=0.05, wheel_separation=0.30, left_scale=1.0,
  right_scale=1.0, pose=Pose())`
  - `wheel_speeds_to_twist(omega_l, omega_r)`, `update_wheels`,
    `update_twist`, `reset`
  - exact constant-twist integration; per-wheel scale error models miscalibration
- `DistanceField(distances, resolution, origin, include_boundary=True)`
  - `from_grid(grid, include_boundary=True, boundary_only=False)`
  - 8-connected chamfer transform in metres
  - `distance_at(x, y)` and vectorised `sample(points)`
  - analytic world-wall distance merged into lookups when `include_boundary`
- `ParticleFilter(grid, n_particles=400, measurement_sigma=0.15,
  max_distance=1.0, alpha_v=0.05, alpha_w=0.10, sigma_v=0.01, sigma_w=0.01,
  max_beams=45, likelihood_scale=0.2, resample_position_noise=0.005,
  resample_heading_noise=0.003, seed=None)`
  - `initialize(pose, position_std, heading_std)`, `initialize_global`,
    `predict(v, w, dt)`, `update(scan, resample=True)`, `step(...)`
  - `resample()`, `estimate()`, `position_std()`, `effective_sample_size`
  - noisy unicycle motion model; endpoint likelihood from the distance field;
    systematic resampling below `ESS < N/2`; roughening jitter

### Experiments
- `scripts/phase1_demo.py` — straight line, rotation, integrator error table,
  circle and collision plots.
- `scripts/phase2_demo.py` — 80x80 inflated map, Dijkstra vs A*, saves
  `experiments/plots/phase2_planning.png`.
- `scripts/phase3_demo.py` — plans the Phase 2 map, follows it with P-only and
  PID heading control under a wheel bias, reports metrics, saves
  `experiments/plots/phase3_tracking.png`.
- `scripts/phase4_demo.py` — 360 degree scans at four poses, clean vs noisy,
  saves `experiments/plots/phase4_lidar.png`.
- `scripts/phase5_demo.py` — drives the Phase 3 route on the particle-filter
  estimate under wheel-scale error, compares odometry and filter error, shows
  offset-prior recovery, saves `experiments/plots/phase5_localization.png`.

### Documentation
- `docs/mathematics.md` — pose conventions, forward/inverse kinematics,
  nonholonomic constraint, integrators, ICC, error measurements, occupancy
  grids, inflation, graph search, heuristic admissibility, PID, tracking, ray
  casting, odometry error, Bayes recursion, motion/measurement models,
  likelihood fields, ESS and resampling.
- `docs/design_decisions.md` — all conventions and deliberate simplifications.
- `README.md` — status, layout, setup, demos, usage snippets.

---

## 3. Verified numbers

- 161 tests pass.
- Phase 1 demo: straight line reaches exactly `x = 5.000`; 180 degree rotation;
  at `dt = 0.05 s` midpoint error over one circle is `5.2e-5 m` vs Euler
  `2.5e-2 m`.
- Phase 2 demo (80x80 grid, 0.25 m cells, 0.20 m inflation):
  - free space 90.4%
  - both planners return a **29.02 m** path
  - Dijkstra expands 4,465 cells (~12 ms)
  - A* expands 1,074 cells (~6 ms), a **76% reduction**
  - A* with octile is optimal and matches Dijkstra's cost exactly
- Phase 3 demo (29.17 m path, 20 Hz, 0.2 rad/s injected wheel bias):
  - both controllers reach the goal with **zero collisions**
  - P-only cross-track RMS `0.0513 m`, PID `0.0258 m` (50% reduction)
  - final error `0.099 m` (P) and `0.098 m` (PID), inside the 0.1 m tolerance
  - PID control effort `14.1 rad` vs P-only `14.5 rad`
- Phase 4 demo (360 rays, 8 m range, `sigma = 0.03 m`):
  - scans from four poses return 250-315 hits out of 360
  - measured noise RMS `0.028-0.030 m`, matching the configured sigma
  - minimum reported range `0.50 m` beside the long wall
  - rays correctly stop at obstacles and miss beyond 8 m
- Phase 5 demo (29.17 m route, 20 Hz, 800 particles, 60 beams, `sigma = 0.02 m`,
  wheel scales 0.99/1.01):
  - odometry position RMS `11.395 m`, maximum `23.875 m`, final `23.875 m`
  - particle-filter position RMS `0.085 m`, maximum `0.122 m`, final `0.080 m`
  - heading RMS `0.19 deg` (filter) vs `64.87 deg` (odometry)
  - goal reached, zero collisions, 58.9 s, 99% RMS reduction
  - stationary recovery with a 1.1 m offset prior: passes from 0.45/0.22/0.07 m
    to 0.41/0.12/0.08 m for prior std 0.3/0.5/0.8 m

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
- `Pose` from the simulator is ground truth. `Odometry` and `ParticleFilter`
  produce estimates and never read it.

---

## 5. How to run

```bash
cd ~/Documents/Atlas
source .venv/bin/activate
pytest
python scripts/phase1_demo.py
python scripts/phase2_demo.py
python scripts/phase3_demo.py
python scripts/phase4_demo.py
python scripts/phase5_demo.py
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

1. **State estimation is local.** The filter tracks well from a known start but
   cannot recover from an arbitrarily wrong prior; capture range is set by the
   initial particle spread. Global relocalization and recovery behaviours are
   Phase 6.
2. **Grid paths cut visually close to corners.** Inflation gives clearance plus
   a conservative half-cell; the Phase 3 demo adds a 0.05 m planning margin
   because a lookahead controller cuts corners. Longer lookaheads need more
   margin.
3. **Collision is discrete.** `Simulator.step` checks only the post-step pose.
   At high speed the body can tunnel through a thin obstacle. Keep demo speeds
   at or below 0.5 m/s with `dt = 0.05` until swept collision is added.
4. **Velocity saturation is at the twist level**, not per wheel, so the wheel
   mapping can request wheel speeds beyond any real actuator limit.
5. **No wheel dynamics.** Wheel speeds are commands, not states; there is no
   acceleration limit or slip model yet.
6. **Paths are dense.** The Phase 2 demo path has ~103 waypoints for 29 m. The
   follower does a linear nearest-point search each step, which is fine at this
   scale but would need a spatial index in a much larger map.
7. **No dynamic replanning.** The grid is static; that is Phase 6.
8. **No metrics module.** Tracking metrics live in the Phase 3 demo; a
   reusable `evaluation/` module arrives in Phase 7.
9. **LiDAR rays are infinitely thin.** Small obstacles can fall between rays;
   there is no beam width, intensity, multipath or mixed-pixel model.
10. **LiDAR only handles axis-aligned rectangles** plus world walls, and noise
    is independent Gaussian rather than range-dependent with outliers.
11. **Scans are now fused into localization, but not into planning.** The filter
    consumes `LidarScan`; the planner still uses a static map and never reacts
    to unexpected obstacles.
12. **The likelihood is annealed.** `likelihood_scale = 0.2` keeps the cloud
    from collapsing but means the posterior is not calibrated and the reported
    particle spread underestimates true error.
13. **Wheel scale error is constant.** There is no random slip, and the motion
    noise is an approximation of real encoder error.

---

## 7. Phase 3 — Motion control (completed)

**Outcome:** `PID` and `PathFollower` are implemented and integrated with the
simulator. On the Phase 2 map the robot reaches the goal with zero collisions,
0.098 m final error and 0.026 m cross-track RMS under a 0.2 rad/s wheel bias.
The plan below is kept as the record of how the phase was scoped.

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

### Test plan (implemented: 32 tests)
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
- P-only vs PID under wheel bias: PID has lower cross-track RMS.

### Demo
`scripts/phase3_demo.py`:
1. Build the Phase 2 world and grid, plan with A*.
2. Convert cells to world waypoints.
3. Drive the robot with `PathFollower` through `Simulator` at `dt = 0.05`.
4. Stop on goal tolerance, collision, or a step cap.
5. Print metrics and save a plot of planned path, driven trajectory, and
   cross-track error over time.
6. Compare P-only and PID in the same run.

### Acceptance criteria (met)
- All tests pass (105 total, 32 added for Phase 3).
- Robot reaches the goal without collision on the demo map.
- Final position error < 0.15 m.
- Cross-track RMS < 0.1 m on the demo run.
- Control commands always respect speed limits.
- README, mathematics and design decisions updated to Phase 3.

### Order of work (completed)
1. `pid.py` + `test_pid.py`.
2. `path_follower.py` + unit tests for geometry/signs.
3. Closed-loop integration test against the Phase 2 map.
4. `phase3_demo.py` with P vs PID comparison and plot.
5. Documentation and README update.
6. Commit implementation and docs separately, then push.

---

## 8. Phase 4 — Sensor simulation (completed)

**Outcome:** `Lidar` and `LidarScan` are implemented with analytic ray casting,
360°/partial fields of view, mounting offsets, seeded Gaussian range noise and
world-space hit points. Twenty sensor tests cover walls, rectangles, occlusion,
range limits, angle conventions, mounts and noise statistics.

### Goal
Replace perfect environmental knowledge with simulated range measurements,
without yet estimating pose.

### Implemented scope
1. `src/atlas/sensors/lidar.py` — 2D LiDAR with configurable angular field of
   view, angular resolution, max range and mounting offset.
2. Ray casting against `RectObstacle` and world boundaries; misses report
   exactly `max_range`.
3. Optional Gaussian range noise with a documented standard deviation and seed.
4. `tests/test_lidar.py` — known geometry, occlusion, out-of-range, noise
   statistics, measurement count and angle convention.
5. `scripts/phase4_demo.py` — scans the Phase 2 world from four poses and plots
   rays and returns against ground truth.
6. Docs: ray-rectangle intersection derivation, sensor conventions and limits.

The Phase 3 follower still runs on ground-truth pose and the known path. Using
LiDAR for localization and avoidance is Phase 5-6, not Phase 4.

---

## 9. Phase 5 — Odometry and localization (completed)

**Outcome:** `Odometry`, `DistanceField` and `ParticleFilter` are implemented.
The Phase 3 route is driven with the controller consuming the filter estimate
under a 1% per-wheel scale mismatch. The filter holds 0.085 m position RMS and
0.19 deg heading RMS while raw dead reckoning drifts 11.4 m RMS. Thirty-six
localization tests cover integration, field geometry, motion, convergence,
resampling, determinism and validation.

### Goal
Estimate pose from noisy motion and range measurements instead of reading it
from the simulator.

### Implemented scope
1. `src/atlas/robot/odometry.py` — exact constant-twist dead reckoning with
   per-wheel scale error; never reads simulator state.
2. `src/atlas/localization/likelihood_field.py` — 8-connected chamfer distance
   field with obstacle-boundary-only sources and analytic world-wall distance.
3. `src/atlas/localization/particle_filter.py` — noisy unicycle motion model,
   endpoint likelihood against the field, ESS-triggered systematic resampling
   with roughening, circular-mean estimate.
4. `tests/test_odometry.py`, `tests/test_likelihood_field.py`,
   `tests/test_particle_filter.py` — 36 tests.
5. `scripts/phase5_demo.py` — closed-loop route on the estimate, odometry vs
   filter metrics, offset-prior recovery panel.
6. Docs: Bayes recursion, motion and measurement models, likelihood annealing,
   degeneracy, resampling and limitations.

### Acceptance (met)
- All prior tests still pass; total is 161.
- Filter position RMS is far below raw odometry drift (`0.085 m` vs `11.395 m`).
- Goal reached with zero collisions; no NaNs or crashes.
- README, mathematics, design decisions and handoff updated to Phase 5.

---

## 10. Phase 6 — Dynamic obstacles and recovery (next stage)

**Goal:** react to the environment changing after planning, and recover when the
robot is blocked or lost.

Proposed scope:
1. `src/atlas/perception/` — detect unexpected obstacles from LiDAR returns
   against the known map (map-consistency / free-space violation checks).
2. Local obstacle memory — mark newly seen occupied cells, with decay or
   confirmation so a noisy return does not permanently poison the map.
3. Replanning — re-run A* on the updated grid from the current estimate to the
   unchanged goal; handle "no path" explicitly.
4. Recovery behaviours — when progress stalls or the path is blocked: stop,
   rotate/scan to gather information, attempt a short backup, then replan; if
   still blocked, declare failure rather than collide.
5. Tests: obstacle detection from synthetic scans, local map updates, replan
   around a new block, no-path handling, stall detection, backup behaviour,
   recovery determinism.
6. `scripts/phase6_demo.py` — drive the Phase 5 loop, drop an obstacle on the
   route mid-run, show detection, replanned path, recovery manoeuvre and final
   outcome.
7. Docs: map-consistency test, replanning trigger, recovery state machine,
   failure modes.

Acceptance: the robot detects a newly placed obstacle, replans without collision,
and reaches the goal on the modified map while the Phase 5 localization demo
still passes.

---

## 11. Working agreement

Keep the existing conventions and do not change Phase 0-5 behaviour; the
current 161-test suite is the regression baseline. Commit implementation first,
docs and demo second, then push.
