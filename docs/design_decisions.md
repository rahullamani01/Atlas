# Design decisions

- Angles are in radians everywhere inside the code. Degrees only at the edges.
- Headings are wrapped to [-pi, pi).
- Pose is q = (x, y, theta); theta = 0 points along +x, counter-clockwise positive.
- Source layout: src/atlas/<component>/ with tests in tests/.

## Differential drive (Phase 1)

- Wheel convention: `omega_l` is the left wheel, `omega_r` is the right wheel,
  both in rad/s. Positive means the wheel drives the robot forward. With
  theta = 0, `omega_r > omega_l` turns counter-clockwise.
- `step_wheels` is the forward-kinematics entry point; `step(v, w)` is the body
  twist form. They share one integrator so results are identical.
- Integration default is `midpoint` (second-order). `euler` exists to show
  first-order error, `exact` is the reference for constant-twist experiments.
- Velocity limits clamp the body twist (v, w), not each wheel separately. This
  is documented as a simplification in docs/mathematics.md.
- Collision handling is discrete: after a step, if the body circle overlaps an
  obstacle or wall, the pose reverts and the collision is counted. This can
  miss tunnelling at high speed; sensor-based avoidance arrives in Phase 4.
- Robot geometry is validated in `__post_init__`: radius and separation must be
  positive, so kinematics never divides by zero.
- The simulator holds ground-truth pose only. No noise or estimation is mixed
  into Phase 1; that belongs to Phase 5.

## Planning (Phase 2)

- Grid storage is `blocked[row, col]`, with row indexing y and column indexing
  x. World-to-cell conversion floors, so cell centres sit at
  `origin + (index + 0.5) * resolution`.
- Obstacle rasterisation uses cell centres. Inflation is conservative by half a
  cell so planned paths keep the robot radius of clearance.
- Planning defaults to 8-connectivity with diagonal cost sqrt(2) and no corner
  cutting: a diagonal step requires both adjacent orthogonal cells to be free.
- A* uses the octile heuristic by default because it is exact for 8-connected
  uniform grids. Manhattan is rejected with 8-connectivity since it can
  overestimate and break optimality.
- Search endpoints must be in bounds and free; invalid input raises ValueError
  rather than returning an empty path, so callers cannot confuse "bad input"
  with "no route".
- Unreachable goals return `SearchResult(found=False, cost=inf)` and record the
  cells that were expanded.
- Path smoothing/trajectory generation is deliberately deferred to Phase 3;
  Phase 2 returns grid waypoints only.

## Control (Phase 3)

- PID derivative acts on the error signal directly (not on the measurement),
  with zero derivative on the first sample to avoid a kick.
- Anti-windup is conditional: integration freezes only while the output is
  saturated and the error would push it deeper into saturation. The saturated
  command is still returned.
- The path follower is carrot chasing with PID heading control, not pure
  pursuit curvature control. This keeps the phase focused on feedback and PID,
  and avoids a second tuning problem. Pure pursuit remains a future option.
- Default follower settings: lookahead 0.5 m, max speed 0.5 m/s, max yaw rate
  1.5 rad/s, goal tolerance 0.1 m, distance gain 0.8, minimum speed 0.05 m/s.
- When the carrot is behind the robot (|alpha| >= pi/2), linear speed is set to
  zero and the robot turns in place. This prevents driving away from the path.
- The follower consumes world-space waypoints. Grid paths are converted with
  `grid.cell_to_world`; no planner types leak into the controller.
- Paths are not smoothed in this phase. The demo plans with 0.25 m inflation
  (0.20 m body plus a 0.05 m tracking margin) so corner cutting cannot clip
  inflated obstacles.
- The Phase 3 demo injects a constant 0.2 rad/s wheel bias to expose integral
  action. This is an experiment-only disturbance, not a simulator feature.
- Tracking metrics are computed in the experiment script. A reusable
  `evaluation/` module is deferred to Phase 7.

## Sensors (Phase 4)

- LiDAR returns are distances, not points. `LidarScan` keeps body-frame angles,
  world-frame angles and the world sensor origin, so callers do not have to
  re-derive the transformation.
- A miss is encoded as exactly `max_range`, with `hit_mask` separating hits from
  misses. No `inf` or NaN values enter the API.
- Full-field scans span [-pi, pi); partial fields are centred on the robot
  heading. Body-frame angle 0 is always along +x.
- Ray casting is analytic (slab method for rectangles, closed form for walls),
  not sampled along the ray, so thin walls are not missed through step size.
- Range noise is Gaussian, independent per ray, applied only to finite hits and
  clipped to the sensor range. A seed can be supplied for reproducible scans.
- The sensor mount is a body-frame offset; the returned origin and world angles
  make scan points directly usable in world coordinates.
- The controller still uses ground-truth pose. Sensors are not wired into the
  navigation loop yet; that begins with localization in Phase 5.

## Localization (Phase 5)

- Odometry is a separate `Odometry` object that consumes wheel speeds. It never
  reads `Simulator` or `robot.pose`, so drift is an honest property of the
  estimator, not a side effect of sharing state with ground truth.
- Wheel calibration error is modelled as per-wheel positive scales rather than a
  single bias. A differential scale error creates yaw drift, which is the
  failure mode the filter is meant to fix.
- A particle filter was chosen over a histogram filter because the posterior can
  be multimodal (corridors, repeated geometry) and the map is small enough that
  a few hundred samples are cheap.
- The measurement model scores scan endpoints against a precomputed distance
  field rather than ray casting from every particle. This trades memory for a
  large speed gain and keeps the update vectorised.
- Only the free-facing boundary of obstacles contributes to the distance field.
  Interior blocked cells would otherwise look like perfect surfaces and attract
  particles into walls.
- World walls are handled analytically in the distance lookup, not by marking
  the border cells as sources. The analytic form removes the one-cell plateau
  that otherwise leaves a persistent 0.2-0.5 m position bias.
- The likelihood is annealed with `likelihood_scale = 0.2`. A product over 60
  beams is so sharp that the cloud collapses onto one random particle; the
  temperature keeps the posterior broad enough to track. This is a deliberate
  approximation, not a calibrated sensor model.
- At most 60 evenly spaced hits are used per update. Returns at `max_range` are
  excluded through the hit mask.
- Resampling is systematic and triggers only below `ESS < N/2`. Resampling every
  step throws away information and increases variance.
- Roughening jitter after resampling prevents duplicate particles from becoming
  a permanent delta function while process noise is small.
- The controller in the Phase 5 demo consumes the filter estimate, not ground
  truth. This is the first closed loop in the project with imperfect state.
- The filter is initialized with a small spread around the known start. Recovery
  experiments use a wider prior, and the documented capture range is limited by
  how much probability mass the initial cloud puts near the truth.
- Global initialization exists for experiments but is not used by the demo; full
  relocalization and recovery behaviours belong to Phase 6.

## Dynamic obstacles and recovery (Phase 6)

- Perception owns a **copy** of the planning grid. `DynamicObstacleMap` never
  mutates the caller's known map, so a failed experiment can be reset and the
  static map remains the reference.
- Unexpected returns are detected by map inconsistency: a LiDAR endpoint in a
  cell that the known map (plus a one-cell tolerance ring) says is free. This is
  cheap, uses the existing grid, and needs no learned classifier.
- The outer ring of the grid is treated as known even though the planning grid
  does not store world walls as blocked cells. Without this, every scan would
  "discover" the four walls.
- Evidence accumulates with decay and confirmation/release hysteresis instead of
  blocking on a single return. This rejects isolated noisy points and lets a
  temporary obstacle fade after it leaves the field of view.
- Confirmed cells are grown by a clearance radius before entering the planner,
  so a new obstacle is treated like a known one and the robot body fits around
  it. The Phase 6 demo uses 0.30 m clearance to match its planning inflation.
- Replanning is triggered only when the **current path** is blocked, not on
  every map change. Replanning for off-path clutter would cause path churn for
  no benefit.
- The replan start is snapped to the nearest free cell by a ring search. Without
  this, a new obstacle's clearance around the robot's own cell would make every
  `a_star` call fail.
- The recovery state machine is deliberately small: scan in place, replan, back
  up, replan, then stop. Each behaviour is bounded in time and distance.
- Failure is a safe stop, not a collision. The supervisor reports `FAILED` and
  commands zero; it never keeps following a path known to be blocked.
- The stall detector uses displacement over a window rather than instantaneous
  speed, so normal in-place turning does not look like a failure.
- The Phase 6 demo uses ground-truth pose to isolate perception and recovery.
  Phase 5 already demonstrated the same control loop on the filter estimate; the
  supervisor is agnostic to where the pose comes from.
- The Phase 6 demo plans with 0.30 m inflation (0.20 m body plus 0.10 m margin)
  instead of the Phase 3 value of 0.25 m. Replanning from arbitrary poses can
  produce sharper corners, and the larger margin keeps the follower clear of
  known obstacles. Earlier phases keep their original settings.
- The blind baseline is intentional: it shows the cost of ignoring perception,
  not a supported operating mode.
