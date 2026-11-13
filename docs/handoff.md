# Handoff — Project ATLAS

**Date:** 13 November 2026
**Branch:** `main` (working tree; nothing pushed)
**Latest work:** Phase 9 — kidnapped-robot relocalization
**Tests:** 230 passing (`pytest`)
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
| 6 | Dynamic obstacles and recovery | complete |
| 7 | Evaluation and metrics | complete |
| 8 | Integrated autonomy | complete |
| 9 | Kidnapped-robot relocalization | complete |

The system can model a differential-drive robot, simulate its motion in a 2D
world with obstacles, rasterise that world into an occupancy grid, plan an
optimal collision-free route, drive the route with feedback control, sense
surrounding geometry with a simulated 2D LiDAR, estimate its own pose by fusing
wheel odometry with those scans, react when the world changes after planning,
and recover when it is displaced without warning.

The navigation loop detects LiDAR returns that disagree with the known map,
grows a dynamic obstacle map, replans around confirmed obstacles, and falls back
to a bounded scan/backup/fail recovery sequence. A reusable evaluation package
scores kinematics, planning, tracking, localization, recovery, integrated
autonomy and relocalization on one set of scenarios across seeds, so regressions
surface in a single command. The loop is closed: the particle-filter estimate
drives the supervisor and the detector, and ground truth is used only to score
error. A map-consistency monitor now notices a kidnapped robot, a coarse-to-fine
global search reacquires the pose, and navigation resumes. The core roadmap and
the optional relocalization extension are both complete.

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
  - `points_at(pose)` rebuilds the hit points as if the scan were taken at
    `pose`, keeping the measured ranges and body-frame angles (Phase 8)
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
  - `scan_residual(scan, pose=None)` — mean clipped scan-to-map distance at a
    pose, used by the Phase 9 lost detector
  - noisy unicycle motion model; endpoint likelihood from the distance field;
    systematic resampling below `ESS < N/2`; roughening jitter
- `src/atlas/localization/relocalization.py` (Phase 9)
  - `PoseHypothesis(pose, score)`
  - `LocalizationMonitor(residual_limit=0.5, patience=8)`
    - `update(residual) -> bool` fires once after a sustained streak and latches
    - `reset()` rearms after recovery
  - `Relocalizer(grid, max_beams=60, coarse_stride=4, heading_step_deg=10,
    max_candidates=4, coarse_separation=1.5, refine_distance=0.5,
    refine_position_step=0.25, refine_heading_deg=10,
    refine_heading_step_deg=5, max_distance=1.0, wall_tolerance=0.10)`
    - `search(scan)` coarse-to-fine global match, keeps a separated candidate bank
    - `observe(scan, dtheta)` accumulates scores, then rotates hypotheses
    - `best()`, `reset()`, `hypothesis_count`, `observations`
    - out-of-bounds endpoints are penalised by distance outside the map minus
      `wall_tolerance`, never clamped to the boundary

### Phase 6 — perception and recovery (`src/atlas/`)
- `DynamicObstacleMap(known_grid, confirm_threshold=2.0, unblock_threshold=0.5,
  decay=0.8, tolerance=1, clearance=0.25)`
  - owns a copy of the planning grid; never mutates `known_grid`
  - `integrate(scan, pose=None) -> list[Cell]` returns cells that became
    blocked; `pose` interprets the scan at an estimated sensor pose (Phase 8)
  - `evidence` accumulates map-inconsistent returns and decays each update
  - `confirmed` uses hysteresis; `dynamic` is the inflated confirmed footprint
  - `grid` is the known map plus the dynamic footprint
  - `reset()`, `detected_count`
  - known obstacles with a one-cell tolerance are ignored, and a
    `tolerance + 1` cell band along every world edge is treated as known so
    pose error does not turn wall returns into false detections
- `NavState` — `DRIVING`, `SCANNING`, `BACKING_UP`, `ARRIVED`, `FAILED`
- `NavEvent(step, kind, detail)` — `plan`, `replan`, `detect`, `no_path`,
  state transitions
- `NavigationSupervisor(grid, goal_cell, follower=None, detector=None,
  scan_steps=30, scan_speed=1.0, backup_steps=20, backup_speed=-0.15,
  stall_window=120, stall_distance=0.10)`
  - `plan(pose)`, `replan(pose)`, `step(pose, scan, dt) -> Twist`
  - `resume(pose)` — clear the dynamic map and detected cells, replan, drive
  - `advance_clock(steps=1)` — keep event timestamps aligned while paused
  - `goal_reached`, `failed`, `recovering`, `path`, `events`, `plan_count`,
    `replan_count`, `detected_cells`
  - replans when a path cell is blocked; recovers by scanning, backing up, then
    failing safely if no route appears
- `nearest_free_cell(grid, cell, max_radius=12)` — ring search used to snap a
  replan start out of a new obstacle's clearance

### Phase 7 — evaluation (`src/atlas/evaluation/`)
- `metrics.py`
  - `rms(values)`, `segment_projection(point, a, b)`,
    `path_errors(path, poses)`
  - `summarize(values) -> MetricSummary(count, mean, std, minimum, maximum)`
  - metric dataclasses: `KinematicsMetrics`, `PlanningMetrics`,
    `TrackingMetrics`, `LocalizationMetrics`, `NavigationMetrics`,
    `IntegratedMetrics`, `RelocalizationMetrics`
  - every dataclass has `as_dict()` returning a flat float/int dictionary;
    aggregate summaries also expose `as_dict()`
- `scenarios.py`
  - `Scenario(name, kind, world, start, goal, resolution, inflation, dt,
    max_steps, controller, yaw_bias, inject_step, inject_rect, kidnap_step,
    kidnap_pose)` with validation
  - `demo_world()` returns a fresh 20 x 20 world per run
  - `SCENARIOS`, `get_scenario(name)`, `default_scenarios()`
  - eight scenarios: `kinematics`, `planning`, `tracking_pid`, `tracking_p`,
    `localization`, `recovery`, `integrated`, `relocalization`
- `harness.py`
  - `ScenarioResult(scenario, seed, metrics)`
  - `run_kinematics`, `run_planning`, `run_tracking`, `run_localization`,
    `run_recovery`, `run_integrated`, `run_relocalization`
  - `simulate_integrated(...) -> IntegratedTrace` runs the closed loop once and
    returns the trajectory, per-step odometry/filter/heading errors, detection
    counts, supervisor events and detected cells; the benchmark runner and the
    Phase 8 demo both consume it
  - `simulate_relocalization(...) -> RelocalizationTrace` runs the kidnap,
    detection, search, settling and resume sequence and returns the trajectory,
    per-step filter errors and residuals, events and detected cells; the
    benchmark runner and the Phase 9 demo both consume it
  - `run_scenario(scenario, seed)`, `run_suite(scenarios, seeds)`
  - `aggregate(results)` groups by scenario and summarizes each metric

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
- `scripts/phase6_demo.py` — drops a block on the route mid-run and compares
  reactive navigation with a blind baseline; shows detection, replanning,
  collisions and dynamic map growth, saves
  `experiments/plots/phase6_recovery.png`.
- `scripts/phase8_demo.py` — runs the same scenario on the particle-filter
  estimate with estimate-frame detection, compares it against the ground-truth
  recovery run, plots the route, localization error and map growth, and saves
  `experiments/plots/phase8_integrated.png`.
- `scripts/phase9_demo.py` — kidnaps the robot mid-route, detects the mismatch,
  runs the global search during an in-place rotation, reseeds the filter and
  resumes navigation. Plots the route and teleport, localization error and the
  map-consistency residual with all recovery events, and saves
  `experiments/plots/phase9_relocalization.png`.
- `scripts/evaluate.py` — runs the scenario suite across seeds, prints aggregate
  tables, and exports JSON/CSV (`--scenario`, `--seeds`, `--json`, `--csv`,
  `--list`).

### Documentation
- `docs/mathematics.md` — pose conventions, forward/inverse kinematics,
  nonholonomic constraint, integrators, ICC, error measurements, occupancy
  grids, inflation, graph search, heuristic admissibility, PID, tracking, ray
  casting, odometry error, Bayes recursion, motion/measurement models,
  likelihood fields, ESS, resampling, map-consistency detection, evidence
  decay/hysteresis, clearance, replanning, the recovery state machine, scenario
  design, tracking/localization/planning metrics, aggregation, determinism,
  the Phase 8 integrated loop (estimate-frame projection, boundary band,
  integrated metrics) and the Phase 9 relocalization pipeline (scan residual,
  lost detection, coarse-to-fine search, out-of-bounds penalty,
  multi-hypothesis accumulation, recovery sequence and metrics).
- `docs/design_decisions.md` — all conventions and deliberate simplifications.
- `README.md` — status, layout, setup, demos, usage snippets.

---

## 3. Verified numbers

- 230 tests pass.
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
- Phase 6 demo (29.17 m route, 20 Hz, 0.30 m inflation, 0.60 x 0.60 m block
  injected at `t = 4.0 s`):
  - reactive: goal reached, zero collisions, 58.8 s, final error `0.10 m`
  - 56 detected cells, one replan at `t = 4.05 s`
  - blind baseline: collides at 10.0 s, final distance 20.00 m from the goal
  - no-path branch: scans, backs up, reports `FAILED` with a zero command and no
    collision
- Phase 7 benchmark (7 scenarios, seeds 0-2, 21 runs; the suite now has 8
  scenarios and 24 runs):
  - kinematics: Euler `2.10e-4 m`, midpoint `8.76e-7 m`, exact `6.27e-15 m`
    over one circle
  - planning: A* cost `29.021 m`, 103 waypoints, 1,074 vs 4,465 cells expanded
    (**75.9% reduction**), identical cost to Dijkstra
  - tracking: PID cross-track RMS `0.0258 m` vs P-only `0.0513 m`; PID final
    error `0.0982 m`; no collisions
  - localization: odometry position RMS `11.383 m` vs filter `0.0845 m`;
    filter heading RMS `0.18 deg`; no collisions
  - recovery: reached, zero collisions, one replan, 56 detected cells,
    `0.0987 m` final error
  - integrated: reached, zero collisions, one replan, `59.03 ± 0.03 s`,
    `0.0608 ± 0.0044 m` final error, `65.7 ± 2.9` detected cells, filter
    position RMS `0.0915 ± 0.0008 m` (max `0.136 ± 0.011 m`) against odometry
    `11.301 ± 0.006 m`, heading RMS `0.617 ± 0.007 deg`
- Phase 9 benchmark (relocalization, seeds 0-2, kidnap at step 200 to
  `(2.125, 16.125, 0.0)`):
  - reached, zero collisions, exactly one relocalization per run
  - detection latency `0.35 s`, recovery time `2.35 s`
  - error at kidnap `10.531 ± 0.005 m`, post-kidnap RMS `2.592 ± 0.002 m`
  - post-recovery RMS `0.0920 ± 0.0005 m`, post-recovery heading RMS
    `0.58 ± 0.09 deg`
  - final error `0.063 ± 0.003 m`, duration `47.38 ± 0.03 s`, `2.7 ± 0.6`
    replans, zero detected cells at the end (dynamic map cleared on resume)
  - overall heading RMS `14.24 ± 0.03 deg` is dominated by the lost interval
    and is not a steady-state figure
  - Phase 6 recovery (`56` cells, zero collisions, one replan, `0.0987 m` final
    error) and Phase 8 integrated (`59.03 s`, `0.0608 m`, `65.7` cells, filter
    RMS `0.0915 m`) are unchanged
  - deterministic metrics repeat to floating-point precision (aggregate std at
    most ~1e-17); wall-clock planner times vary and are not part of the
    reproducibility claim

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
python scripts/phase6_demo.py
python scripts/phase8_demo.py
python scripts/phase9_demo.py
python scripts/evaluate.py
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

1. **Tracking is local; recovery is explicit.** The filter tracks well from a
   known start, and its capture range is set by the initial particle spread.
   Phase 9 adds a separate global search after a detected kidnap, but the
   filter itself is still a tracking estimator and the search still assumes the
   static map is correct.
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
7. **Replanning reacts only to newly observed cells.** Paths are replanned when
   the current route becomes blocked; moving objects are treated as static and
   there is no tracking or prediction.
8. **The evaluation suite is deliberately small.** One map, one route, seven
   scenario kinds, eight scenarios and three seeds. It is a regression harness,
   not a broad robotics benchmark; aggregation reports descriptive statistics
   only.
9. **Demos and the harness are mostly separate.** The Phase 1-6 scripts remain
   narrative experiments with plots and do not call `evaluation/`. Phase 8 is
   the exception: `scripts/phase8_demo.py` and `run_integrated` both use
   `simulate_integrated`, so the plotted and scored runs cannot drift. A change
   to a shared component should still run both, because the older harness runs
   will not catch a plotting or narrative regression.
10. **LiDAR rays are infinitely thin.** Small obstacles can fall between rays;
    there is no beam width, intensity, multipath or mixed-pixel model.
11. **LiDAR only handles axis-aligned rectangles** plus world walls, and noise
    is independent Gaussian rather than range-dependent with outliers.
12. **Detection only adds observed surfaces.** Occluded parts of a new obstacle
    stay unknown and the map is never repaired if the world changes elsewhere.
13. **The likelihood is annealed.** `likelihood_scale = 0.2` keeps the cloud
    from collapsing but means the posterior is not calibrated and the reported
    particle spread underestimates true error.
14. **Wheel scale error is constant.** There is no random slip, and the motion
    noise is an approximation of real encoder error.
15. **The Phase 6 demo still uses ground-truth pose** to isolate perception and
    recovery. Phase 8 closes that gap with `integrated`, which drives the same
    supervisor from the particle-filter estimate.
16. **The integrated filter still scores against the static map.** The dynamic
    map feeds planning and detection, not the measurement likelihood. A
    prototype that rebuilt the likelihood field from the dynamic map made filter
    RMS slightly worse in this scenario, so it was not adopted.
17. **The boundary band is geometric, not covariance-aware.** A fixed
    `tolerance + 1` cell band along world edges suppresses false detections from
    pose error, at the cost of sensitivity near walls. Detection tolerance is
    likewise fixed rather than derived from particle spread.
18. **Relocalization is a simplified global search.** It assumes a known static
    map, a free kidnap pose, a stationary in-place rotation, and fixed residual
    thresholds. It is stress-tested on several poses during development but the
    benchmark uses one kidnap pose; symmetry failures remain possible in a
    repetitive map. There is no loop closure, no SLAM and no relocalization
    while driving.
19. **Dynamic-map cells are discarded on recovery.** `resume()` clears the
    dynamic map because cells confirmed in the old pose frame are phantom
    obstacles in the new one. This is correct for a kidnap, but it also throws
    away any genuinely detected obstacle that is still present; the next scans
    must reconfirm it.
20. **The residual threshold is fixed.** `residual_limit = 0.5 m` and
    `patience = 8` are tuned for the benchmark's 0.02 m range noise. A
    range-dependent threshold derived from particle spread would generalize
    better, at the cost of more machinery.

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

## 10. Phase 6 — Dynamic obstacles and recovery (completed)

**Outcome:** `DynamicObstacleMap`, `NavigationSupervisor` and `nearest_free_cell`
are implemented. A block dropped on the route mid-run is detected from
map-inconsistent LiDAR returns, confirmed with decaying evidence and a
hysteresis pair, inflated by clearance, and avoided by a single A* replan. The
reactive run reaches the goal with zero collisions while a blind baseline stops
at the block. A sealed map exercises the scan/backup/fail branch, which ends in
an explicit safe stop. Eighteen new tests cover detection, decay, clearance,
replanning, stall handling, recovery and validation.

### Goal
React to the environment changing after planning, and recover when the robot is
blocked or lost.

### Implemented scope
1. `src/atlas/perception/obstacle_detector.py` — map-consistency detection,
   decaying per-cell evidence, confirmation/release hysteresis, clearance
   growth, implicit world-wall handling and a copied planning grid.
2. `src/atlas/recovery/supervisor.py` — path-blocked replanning, nearest-free
   start snapping, stall detection, scan/backup/fail state machine and explicit
   terminal `ARRIVED`/`FAILED` states.
3. `tests/test_obstacle_detector.py` (8) and `tests/test_recovery.py` (10).
4. `scripts/phase6_demo.py` — reactive vs blind comparison, detection curve and
   dynamic map growth.
5. Docs: map-consistency test, evidence recursion, hysteresis, clearance,
   replanning and the recovery state machine.

### Acceptance (met)
- All prior tests still pass; total is 179.
- The robot detects the new block, replans once and reaches the goal with zero
  collisions (`0.10 m` final error).
- A blocked or sealed map ends in `FAILED` with a zero command, never a
  collision.
- README, mathematics, design decisions and handoff updated to Phase 6.

---

## 11. Phase 7 — Evaluation and metrics (completed)

**Outcome:** `src/atlas/evaluation/` provides metric functions, typed metric
objects, a validated scenario registry, five runners, suite execution and
cross-seed aggregation. `scripts/evaluate.py` runs the full suite (6 scenarios x
3 seeds, 18 runs) in about 9.4 s and exports JSON/CSV. Thirty-three new tests
cover the metric maths, scenario validation, each runner, reproducibility and
aggregation. The harness reproduces the Phase 3, 5 and 6 headline numbers.

### Goal
Turn the per-phase demo metrics into a reusable evaluation harness and an
aggregate benchmark, so changes can be compared on identical scenarios.

### Implemented scope
1. `metrics.py` — `rms`, nearest-segment projection, cross-track and wrapped
   heading errors, `summarize` with sample standard deviation, and five metric
   dataclasses with flat `as_dict()` output.
2. `scenarios.py` — world factories, six named scenarios (`kinematics`,
   `planning`, `tracking_pid`, `tracking_p`, `localization`, `recovery`) and
   validation for every field, including the paired injection arguments.
3. `harness.py` — one runner per scenario kind, a dispatcher, `run_suite`, and
   `aggregate` producing mean/std/min/max per metric.
4. `scripts/evaluate.py` — aggregate tables plus `--scenario`, `--seeds`,
   `--json`, `--csv` and `--list`.
5. `tests/test_metrics.py` (14) and `tests/test_evaluation.py` (19).
6. Docs: scenario model, metric derivations, aggregation, determinism,
   measured results and limitations.

### Acceptance (met)
- All prior tests still pass; total is 212.
- Every Phase 1-6 experiment is expressible as a scenario; one command runs the
  suite.
- Seeded runs are reproducible; deterministic metrics repeat exactly.
- Phase 0-6 behaviour is unchanged.

### Measured results
See section 3. Headlines: exact integration `6e-15 m`, A* 75.9% fewer expansions
than Dijkstra at identical cost, PID cross-track `0.0258 m` vs P-only `0.0513 m`,
filter RMS `0.0845 m` vs odometry `11.383 m`, recovery reaches the goal with zero
collisions and one replan.

---

## 12. Phase 8 — Integrated autonomy (completed)

**Outcome:** the navigation loop runs on the particle-filter estimate. The
supervisor plans, detects and replans from `pf.estimate()`, and the detector
interprets the scan in that same estimated frame via
`LidarScan.points_at(pose)`. Ground truth is read only after each step to score
odometry and filter error. A new `integrated` scenario reuses the Phase 6 map,
inflation and injection so the two runs compare directly. Five integration
tests plus a `points_at` test bring the suite to 217.

### Goal
Run the whole loop on the estimate and the live map, rather than proving each
subsystem separately on ground truth.

### Implemented scope
1. `LidarScan.points_at(pose)` reprojects measured body-frame hits through a
   supplied pose, so detection never consumes the ground-truth scan origin.
2. `DynamicObstacleMap.integrate(scan, pose=None)` accepts an optional pose; the
   supervisor passes the estimate it used for control, keeping one source of
   truth for the map, plan and trajectory.
3. The detector treats a `tolerance + 1` cell band along every world edge as
   known. At 10 m a 0.03 rad heading error moves a wall hit by 0.3 m, more than
   one 0.25 m cell; the band removes those false boundary detections while the
   injected block stays detectable.
4. `IntegratedMetrics` and `IntegratedTrace`; `simulate_integrated` and
   `run_integrated`; the `integrated` scenario, dispatcher entry and exports.
5. `scripts/phase8_demo.py` compares estimate-driven and ground-truth recovery
   and plots route, error and map growth.
6. `tests/test_integration.py` and a `points_at` test in `tests/test_lidar.py`.
7. Docs: `docs/mathematics.md` section 17 and a Phase 8 section in
   `docs/design_decisions.md`.

### Acceptance (met)
- All prior tests still pass; total is 217.
- The estimate-driven loop reaches the goal with zero collisions and one
  replan in the integrated scenario.
- Integrated scenarios are reproducible for a seed, and the benchmark runs them
  across seeds.
- Phase 0-7 behaviour is unchanged: the Phase 6 demo still reports 56 cells,
  zero collisions and one replan.

### Measured results
See section 3. Headlines: reached with zero collisions, `59.03 s`,
`0.061 m` final error, `0.0915 m` filter RMS against `11.301 m` odometry RMS,
`0.617 deg` heading RMS, and `65.7` detected cells.

### Optional next step (Phase 9)
Relocalization after a kidnapped-robot event: a global particle spread or an
explicit relocalization behaviour, plus a scenario that displaces the robot
mid-run and measures recovery time. This was proposed at the end of Phase 8 and
is implemented in section 13.

---

## 13. Phase 9 — Kidnapped-robot relocalization (completed)

**Outcome:** the robot now notices and recovers from a silent displacement.
`ParticleFilter.scan_residual` measures how well a scan fits the map at the
estimate; `LocalizationMonitor` declares the filter lost after `patience`
consecutive inconsistent scans; `Relocalizer` runs a coarse-to-fine global scan
match, keeps a spatially separated bank of pose hypotheses and accumulates their
scores over an in-place rotation; the winner reseeds the filter; and
`NavigationSupervisor.resume()` clears the now-untrustworthy dynamic map and
replans. In the new `relocalization` scenario the robot detects the kidnap in
`0.35 s`, reacquires its pose in `2.35 s`, and reaches the goal with zero
collisions and `0.092 m` post-recovery RMS.

### Goal
Recover from a kidnapped-robot event: detect that the filter is inconsistent,
find the true pose globally in the known map, and resume navigation without a
collision or a manual restart.

### Implemented scope
1. `ParticleFilter.scan_residual(scan, pose=None)` — mean clipped distance from
   projected endpoints to mapped obstacles, sharing the beam subset and clipping
   with `update`.
2. `src/atlas/localization/relocalization.py` — `PoseHypothesis`,
   `LocalizationMonitor` (residual limit, patience, latch, rearm) and
   `Relocalizer` (coarse-to-fine search, out-of-bounds penalty, separated
   candidate bank, observation accumulation, best hypothesis).
3. `NavigationSupervisor.resume(pose)` and `advance_clock(steps)` — clear the
   dynamic map and detections, replan, and keep supervisor event timestamps
   aligned while paused.
4. `RelocalizationMetrics`, `RelocalizationTrace`, `simulate_relocalization`,
   `run_relocalization`, dispatcher entry and exports.
5. `kidnap_step` / `kidnap_pose` scenario fields with paired validation, and the
   `relocalization` scenario (kidnap at step 200 to `(2.125, 16.125, 0.0)`, no
   obstacle injection).
6. `tests/test_relocalization.py` (13 tests) and `tests/test_evaluation.py`
   updates for the new kind, scenario count and validation.
7. `scripts/phase9_demo.py` — route/teleport plot, localization error and
   residual timeline with all recovery events.
8. Docs: `docs/mathematics.md` section 18 and the Phase 9 section of
   `docs/design_decisions.md`.

### Acceptance (met)
- All prior tests still pass; total is 230.
- Every seeded run detects the kidnap, relocalizes exactly once, and reaches the
  goal with zero collisions.
- Detection latency is below `0.5 s` and pose recovery below `3 s`.
- Post-recovery position RMS is below `0.1 m` and heading RMS below `1 deg`.
- Phase 0-8 behaviour is unchanged: Phase 6 recovery still reports 56 cells,
  zero collisions and one replan; Phase 8 integrated still reports `59.03 s`,
  `0.0608 m`, `65.7` cells and `0.0915 m` filter RMS.

### Measured results
See section 3. Headlines: `0.35 s` detection, `2.35 s` recovery, `10.531 m`
error at the kidnap, `0.0920 m` post-recovery RMS, `0.58 deg` post-recovery
heading RMS, `0.063 m` final error, zero collisions.

### Deliberate simplifications
- The robot stops and rotates to gather evidence.
- The global search uses the static map only.
- The kidnap pose must be free in the planning grid.
- The residual threshold and patience are fixed constants.
- One kidnap pose is benchmarked, though several were used during development.

---

## 14. Working agreement

Keep the existing conventions and do not change Phase 0-8 behaviour; the
current 230-test suite is the regression baseline. Commit implementation first,
docs and demo second. Do not push to `origin` until explicitly approved.
