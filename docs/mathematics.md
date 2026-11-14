# Mathematics

Derivations behind the modules, written before (or alongside) the code.
Conventions are listed in `docs/design_decisions.md`.

## 1. Pose and coordinate conventions

The robot pose is

```text
q = (x, y, θ)
```

where `(x, y)` is the midpoint of the wheel axle in the world frame and `θ` is
the heading measured from the positive x-axis, counter-clockwise positive.
Angles are always radians internally and wrapped to `[-π, π)`.

## 2. Differential drive geometry

```text
            y
            ^
            |
        L   |   wheel axle
     <------+------>
      left    right
```

- `r` — wheel radius [m]
- `L` — distance between wheel contact points (wheel separation) [m]
- `ω_l`, `ω_r` — left/right wheel angular velocities [rad/s]
- `v_l = r ω_l`, `v_r = r ω_r` — wheel contact-point speeds [m/s]

Each wheel rolls without slipping, so its contact point moves along the robot's
forward axis with speed `v_l` or `v_r`.

## 3. Forward kinematics: wheels → body twist

The body twist `(v, ω)` is the rigid-body velocity of the axle midpoint:
`v` is forward speed and `ω` is yaw rate.

**Linear velocity.** The midpoint speed is the average of the wheel speeds:

```text
v = (v_r + v_l) / 2 = r (ω_r + ω_l) / 2
```

**Angular velocity.** For a rotation about the instantaneous centre of
curvature, the wheel speeds differ by `ω L`:

```text
v_r - v_l = ω L
ω = (v_r - v_l) / L = r (ω_r - ω_l) / L
```

Sanity checks:

| Wheel speeds | Result |
|---|---|
| `ω_l = ω_r` | `v = r ω`, `ω = 0` — straight line |
| `ω_l = -ω_r` | `v = 0`, `ω = 2 r ω_r / L` — spin in place |
| `ω_r > ω_l` | `ω > 0` — counter-clockwise (left) turn |

These two equations are exactly `DifferentialDriveRobot.wheel_speeds_to_twist`.

## 4. Inverse kinematics: body twist → wheels

Invert the pair above. From `v = r(ω_r + ω_l)/2` and `ω = r(ω_r - ω_l)/L`:

```text
v_r = v + ω L / 2        ω_r = (v + ω L / 2) / r
v_l = v - ω L / 2        ω_l = (v - ω L / 2) / r
```

Implemented as `DifferentialDriveRobot.twist_to_wheel_speeds`, which returns
`(ω_l, ω_r)`. The map is linear and invertible for `r > 0`, `L > 0`:

```text
[ v ]   [ r/2   r/2 ] [ ω_l ]
[ ω ] = [ -r/L  r/L ] [ ω_r ]
```

The determinant is `r²/L ≠ 0`, which is why the round trip is exact (tested in
`test_wheel_speed_roundtrip`).

## 5. Nonholonomic constraint

A differential-drive robot cannot move sideways. With body-frame velocity
`(v_x, v_y)`, the no-slip constraint is

```text
v_y = 0   ⇔   -ẋ sin θ + ẏ cos θ = 0
```

Equivalently, the admissible velocities span only two dimensions `(v, ω)` out
of the three-dimensional pose space. This is why the robot needs to turn
before it can move in a new direction, and it is the reason planning and
control (Phases 2–3) cannot treat the robot as a free-flying point.

## 6. Motion model (pose ODE)

With constant body twist over a step, the pose evolves as

```text
ẋ = v cos θ
ẏ = v sin θ
θ̇ = ω
```

There are three implemented integrators (`method=` in `step`).

### 6.1 Forward Euler

```text
x_{k+1} = x_k + v cos θ_k Δt
y_{k+1} = y_k + v sin θ_k Δt
θ_{k+1} = θ_k + ω Δt
```

First-order accurate: the local error is `O(Δt²)`, so global error is
`O(Δt)`. The heading is held fixed for the whole step, which is why it cuts
the corner on curves.

### 6.2 Midpoint (default)

Evaluate the direction at the mid-step heading:

```text
θ_mid   = θ_k + ω Δt / 2
x_{k+1} = x_k + v cos θ_mid Δt
y_{k+1} = y_k + v sin θ_mid Δt
θ_{k+1} = θ_k + ω Δt
```

Second-order accurate globally `O(Δt²)`; cheap and stable for control loops.

### 6.3 Exact (closed form)

For constant `v` and `ω ≠ 0`, integrate the ODE analytically:

```text
x(t) = x_0 + (v/ω) [ sin(θ_0 + ω t) - sin θ_0 ]
y(t) = y_0 - (v/ω) [ cos(θ_0 + ω t) - cos θ_0 ]
θ(t) = θ_0 + ω t
```

For `ω = 0` this degenerates to a straight line:

```text
x(t) = x_0 + v cos θ_0 t
y(t) = y_0 + v sin θ_0 t
```

The exact form is used as the reference solution in experiments. It is exact
for piecewise-constant twist, not for real robots whose twist changes
continuously.

## 7. Circular motion

For `ω ≠ 0` the robot follows a circle of radius

```text
R = v / ω
```

around the instantaneous centre of curvature (ICC)

```text
ICC = ( x - R sin θ,  y + R cos θ )
```

A full loop takes `T = 2π / |ω|`. These relations are used to generate the
reference circle in `scripts/phase1_demo.py` and to check that one loop
returns to the start (`test_exact_circle_closes`).

## 8. Measured integration error

From `scripts/phase1_demo.py`, maximum position error over one full circle
(`v = 0.5 m/s`, `ω = 0.5 rad/s`, `R = 1 m`) against the exact solution:

| `Δt` [s] | Euler | Midpoint |
|---:|---:|---:|
| 0.50 | 2.51e-1 | 5.26e-3 |
| 0.20 | 9.97e-2 | 8.29e-4 |
| 0.10 | 4.99e-2 | 2.07e-4 |
| 0.05 | 2.50e-2 | 5.22e-5 |
| 0.01 | 5.00e-3 | 2.08e-6 |

Halving `Δt` halves Euler's error (first order) and quarters Midpoint's
(second order), matching the theory. At `Δt = 0.05 s` (20 Hz control) the
midpoint error is sub-millimetre over a full loop, which is far below any
sensor or odometry error introduced later.

## 9. Velocity saturation

`step` clamps the body twist to `max_linear_speed` and `max_angular_speed`
before integrating. This models a robot with finite actuators and keeps
experiments numerically sane. It is a simplification: real limits are on wheel
torque and wheel speed, and clamping `(v, ω)` independently is not the same as
clamping `(ω_l, ω_r)` independently. A future phase can move saturation to the
wheel level.

## 10. Assumptions and limitations

1. Wheels roll without slipping; no lateral or longitudinal slip.
2. The robot is a point for kinematics; `body_radius` is used only for
   collision checks.
3. Twist is constant within a timestep.
4. The floor is planar and the wheels stay in contact.
5. Pose integration is deterministic and noise-free — uncertainty is added in
   Phase 5 (localization), not here.

## 11. Occupancy grids and graph search (Phase 2)

### 11.1 Rasterisation

A grid with resolution `s`, origin `(o_x, o_y)` and `W × H` cells maps between
world and discrete coordinates as

```text
col = floor((x - o_x) / s)          x_centre = o_x + (col + 0.5) s
row = floor((y - o_y) / s)          y_centre = o_y + (row + 0.5) s
```

A cell is blocked when its centre lies inside an obstacle. Storing occupancy
as `blocked[row, col]` keeps array indexing `(row, col)` separate from world
`(x, y)`, so the two conventions cannot silently swap.

### 11.2 Inflation for robot size

Planning on a point robot would produce paths that scrape walls. Instead, every
blocked cell is grown by the robot body radius before searching. If a cell
centre is within

```text
r_inflate + s/2
```

of a blocked cell centre, it becomes blocked. The extra half-cell makes the
result conservative by at most one cell diagonal, so the returned path keeps at
least the body radius of clearance. The cost is that narrow gaps smaller than
the robot are correctly reported as impassable rather than producing an
optimistic path.

### 11.3 Graph formulation

- **Nodes:** free cells.
- **Edges:** 4- or 8-neighbourhood of free cells.
- **Costs:** `1` for orthogonal moves, `√2` for diagonal moves.

With 8-connectivity, a diagonal move between `(c, r)` and `(c±1, r±1)` is only
allowed when both adjacent orthogonal cells are free. Without that rule a path
can cut through the corner between two obstacles, which a circular body cannot
do. The check is skipped only when explicitly requested.

### 11.4 Dijkstra

Dijkstra expands nodes in increasing order of the exact cost from the start:

```text
g(n) = cost of the cheapest known path from start to n
```

It is optimal for non-negative edge costs and explores in all directions until
the goal is popped, so it is the reference implementation.

### 11.5 A*

A* orders the open set by

```text
f(n) = g(n) + h(n)
```

where `h(n)` estimates the remaining cost to the goal. With an admissible
heuristic (`h(n) ≤ true remaining cost`) and a consistent one
(`h(n) ≤ c(n, m) + h(m)`), the first time the goal is expanded its cost is
optimal.

Heuristics on a uniform grid (`dx = |Δcol|`, `dy = |Δrow|`):

```text
h_manhattan = dx + dy                          # admissible for 4-connectivity
h_euclidean = sqrt(dx² + dy²)                  # admissible, weaker
h_octile    = (dx + dy) + (√2 - 2) min(dx, dy)  # exact for 8-connectivity
```

Manhattan overestimates diagonal travel, so `a_star` rejects it when
`connectivity == 8`. Octile is exact, which is why A* with octile expands the
smallest corridor of cells in the experiments.

Equal `f` values are broken by lower `h`, which biases expansion toward the
goal and reduces the number of expanded nodes without affecting optimality.

### 11.6 Complexity

With `N` free cells and a binary heap, both algorithms run in
`O(N log N)` time and `O(N)` memory. The difference is practical: on the Phase 2
demo map (80 × 80 cells, 90.4% free, 0.20 m inflation) both find the same
29.02 m path, but Dijkstra expands 4,465 cells versus 1,074 for A* — a 76%
reduction.

### 11.7 Limitations

1. The grid is static; replanning for moving obstacles belongs to Phase 6.
2. Paths are sequences of cell centres, not smooth trajectories. Following
   them with a controller is Phase 3.
3. Occupancy is binary; probabilistic occupancy is a later extension.
4. Rasterisation at cell centres can miss very thin obstacles smaller than one
   cell. Inflating by at least half a cell mitigates this.

## 12. Feedback control and path following (Phase 3)

### 12.1 Tracking errors

For pose `(x, y, θ)` and a path polyline, three errors matter:

- **Distance to goal:** `d_goal = ||p_goal - p||`, used for stopping and speed.
- **Cross-track error:** the minimum distance from the robot position to any
  path segment (not merely the nearest waypoint). Measuring to waypoints alone
  overstates the error by up to half a cell diagonal.
- **Heading error:** for the nearest segment with direction `θ_path`,

  ```text
  ψ = wrap(θ_path - θ)
  ```

  so positive `ψ` means the path points to the robot's left.

### 12.2 Discrete PID

With error `e_k` and timestep `Δt`:

```text
I_k = clamp(I_{k-1} + e_k Δt,  ±I_max)
D_k = (e_k - e_{k-1}) / Δt
u_k = clamp(kp e_k + ki I_k + kd D_k, ±u_max)
```

The derivative is set to zero on the first sample to avoid a derivative kick
from an undefined previous error.

**Anti-windup.** While `|u_k| > u_max` and `e_k u_k > 0`, the stored integral
is frozen at `I_{k-1}`. The saturated command is still applied, but the
controller does not accumulate error it cannot act on. Without this, a long
saturated stretch leaves a large integral that must be unwound before the
output can reverse, causing overshoot.

For a constant disturbance `w_b`, a P-only heading controller settles where
`kp e = w_b`, leaving a steady offset. Integral action drives `e → 0` by
supplying the bias through `I`.

### 12.3 Carrot selection

The follower finds the nearest waypoint, then walks forward along the polyline
accumulating arc length until it reaches the lookahead distance `L_d`. If the
lookahead falls inside a segment, the point is linearly interpolated:

```text
carrot = a + t (b - a),   t = remaining / ||b - a||
```

This keeps the target on the path instead of jumping between sparse waypoints.

### 12.4 Control law

Let `α` be the bearing to the carrot in the robot frame:

```text
α = wrap(atan2(carrot_y - y, carrot_x - x) - θ)
ω = clamp(PID_heading(α), ±ω_max)
```

Linear speed slows with distance and is cut when the carrot is not ahead:

```text
if |α| >= π/2:   v = 0                    # rotate in place
else:            v = clamp(max(k_d d_goal cos α, v_min), 0, v_max)
```

The robot stops and reports `goal_reached` when `d_goal ≤ tolerance`.

Setting `v = 0` for `|α| ≥ π/2` prevents the robot from driving away from the
path while turning around. The cosine term reduces speed during sharp turns,
and `k_d d_goal` produces a smooth approach to the goal.

### 12.5 Tuning and stability notes

- At `Δt = 0.05 s` (20 Hz), `kp = 2` gives a responsive heading loop without
  visible oscillation on the demo map.
- `kd` damps heading-rate overshoot; it is sensitive to measurement noise, which
  matters once noisy sensors arrive in Phase 4.
- `ki` is small (`0.2`) because integral action only needs to remove slow bias;
  larger values make corner transients worse.
- Saturation is applied by both the PID and the follower, so commands never
  exceed the robot limits.

### 12.6 Measured result

Phase 3 demo, 29.17 m planned path, 20 Hz, with a deliberately injected
0.2 rad/s wheel bias (unmodelled yaw rate):

| Metric | P-only (`kp=2`) | PID (`kp=2, ki=0.2, kd=0.05`) |
|---|---:|---:|
| Reached goal | yes | yes |
| Collisions | 0 | 0 |
| Time [s] | 58.8 | 58.6 |
| Driven length [m] | 28.73 | 28.67 |
| Final error [m] | 0.099 | 0.098 |
| Cross-track RMS [m] | 0.0513 | **0.0258** |
| Cross-track max [m] | 0.1174 | 0.1011 |
| Heading RMS [deg] | 7.64 | 7.56 |
| Control effort [rad] | 14.5 | 14.1 |

Integral action halves the cross-track RMS under a constant bias, at slightly
lower control effort. The bias is injected only for the experiment; the
simulator itself has no wheel asymmetry yet.

### 12.7 Limitations

1. The controller is kinematic: it commands `(v, ω)` with no wheel dynamics,
   acceleration limits or actuator delay.
2. The path is followed as a polyline; there is no smoothing or time-optimal
   trajectory generation.
3. Obstacle avoidance beyond the planned path is not handled here. A moving
   obstacle or a stale path needs Phase 6 replanning.
4. The lookahead is fixed. Adaptive lookahead based on speed and curvature is a
   natural improvement.
5. Localization is assumed perfect. Phase 5 replaces the true pose with an
   estimate, at which point controller noise sensitivity matters.

## 13. LiDAR simulation (Phase 4)

### 13.1 Ray model

A ray starts at the sensor origin `o = (o_x, o_y)` and travels along a unit
direction `d = (cos φ, sin φ)`:

```text
p(t) = o + t d,    t >= 0
```

The measured range is the smallest positive `t` at which the ray meets an
obstacle or a world wall, capped at `max_range`.

### 13.2 Ray versus axis-aligned rectangle

Rectangles are axis-aligned, so each ray can be intersected with the rectangle's
x-slab and y-slab independently (the slab method).

For one axis with origin `o`, direction `d` and slab `[low, high]`:

```text
if |d| < ε:
    miss unless low <= o <= high
else:
    t1 = (low - o) / d
    t2 = (high - o) / d
    swap so t1 <= t2
    t_enter = max(t_enter, t1)
    t_exit  = min(t_exit, t2)
```

The ray hits the rectangle when `t_enter <= t_exit` and `t_exit >= 0`. The entry
distance is `t_enter`, except when the origin is inside the rectangle, where
`t_enter < 0` and the first surface is at `t_exit`. In this project the robot is
never inside an obstacle, but the case is handled.

### 13.3 World boundary

A ray starting inside the world always leaves it. The distance to each wall is

```text
t_x = (W - o_x) / d_x   if d_x > 0
t_x = (0 - o_x) / d_x   if d_x < 0
```

and similarly for `t_y`; the boundary distance is the smallest non-negative one.
Walls are implicit and do not need to be stored as obstacles.

### 13.4 Nearest return

For each ray:

```text
range = min(
    boundary distance,
    distance to every rectangle that is hit,
    max_range
)
```

A ray that hits nothing within `max_range` reports exactly `max_range`. This
keeps the output a fixed-size array and makes "no return" explicit through the
hit mask, rather than using `inf` in downstream code.

### 13.5 Scan geometry

For `n = round(fov / angle_increment)` rays, the actual angular step is
`Δ = fov / n` and body-frame angles are

```text
φ_i = -fov/2 + i Δ + angle_offset,    i = 0..n-1
```

World angles add the robot heading: `φ_world = φ_i + θ`. A full 360 degree scan
therefore spans `[-π, π)`. Narrower fields are centred on the heading, so a
180 degree sensor looks forward, not sideways.

A sensor mounted at body-frame offset `(a, b)` has world origin

```text
o_x = x + a cos θ - b sin θ
o_y = y + a sin θ + b cos θ
```

### 13.6 Range noise

When `range_sigma > 0`, each finite return is perturbed independently:

```text
r_measured = clip( r_true + N(0, sigma²), 0, max_range )
```

Noise is not added to no-return rays, because there is no surface to reflect
from. Clipping means a noisy return can reach `max_range` and be reported as a
miss, which is physically reasonable at the sensor's limit.

### 13.7 Measured behaviour

Phase 4 demo, 360 rays, 8 m range, `sigma = 0.03 m`:

| Pose | Hits | Min range | Noise RMS |
|---|---:|---:|---:|
| (2.0, 2.0, 0°) | 315/360 | 2.00 m | 0.028 m |
| (5.5, 12.0, 90°) | 255/360 | 0.50 m | 0.028 m |
| (12.0, 6.0, 180°) | 291/360 | 2.24 m | 0.030 m |
| (17.5, 17.5, -90°) | 250/360 | 2.50 m | 0.030 m |

The measured noise RMS recovers the configured `0.03 m` within sampling error.

### 13.8 Limitations

1. Rays are infinitely thin: small obstacles can be missed between rays, and
   there is no beam divergence or intensity return.
2. Only axis-aligned rectangles are supported, not arbitrary polygons or circles.
3. Reflections, transparency, multipath and mixed pixels are not modelled.
4. Noise is independent per ray and Gaussian; real sensors have range-dependent
   bias and outliers.
5. The scan is used for visualisation only so far. Nothing consumes it for
   mapping, localization or avoidance until Phases 5-6.

---

## 14. Odometry and localization (Phase 5)

### 14.1 Dead reckoning and wheel scale error

Odometry integrates wheel angular velocities with the geometry from Section 4:

```text
v_hat = r (omega_r + omega_l) / 2
w_hat = r (omega_r - omega_l) / L
```

Wheel miscalibration is modelled as a scale on each wheel:

```text
omega_l^measured = s_l omega_l^true,    omega_r^measured = s_r omega_r^true
```

so the estimated twist is

```text
v_hat = r (s_r omega_r + s_l omega_l) / 2
w_hat = r (s_r omega_r - s_l omega_l) / L
```

A pure differential error `delta = s_r - s_l` while driving straight
(`omega_l = omega_r = omega`, `v = r omega`) adds a yaw-rate bias

```text
w_bias = (v / L) (s_r - s_l)
```

which integrates into heading error and then into unbounded lateral drift. This
is the failure mode the filter has to correct; the scale errors are intentionally
part of the model rather than hidden simulator noise.

The pose update uses the exact constant-twist solution from Section 6.3 rather
than Euler integration, so odometry itself is not a source of discretisation
error.

### 14.2 Bayes filter recursion

Localization is the recursive Bayes filter

```text
bel(x_t) = eta * p(z_t | x_t) * integral p(x_t | x_{t-1}, u_t) bel(x_{t-1}) dx_{t-1}
```

where `x_t = (x, y, theta)` is the pose, `u_t` the odometry increment and `z_t`
the LiDAR scan. The particle filter represents `bel` by `N` weighted samples:

```text
bel(x) ~ { (x_i, w_i) },   i = 1..N,   sum w_i = 1
```

Prediction samples the motion model; the measurement update multiplies each
weight by `p(z | x_i)`; resampling concentrates samples where the posterior mass
is.

### 14.3 Motion model

Each particle is propagated independently through the same exact-arc update used
by odometry, but with noisy inputs:

```text
v_i = v_hat + N(0, sigma_v + alpha_v |v_hat|)
w_i = w_hat + N(0, sigma_w + alpha_w |w_hat|)
```

```text
if |w_i| < epsilon:
    x_i += v_i cos(theta_i) dt
    y_i += v_i sin(theta_i) dt
else:
    theta_new = theta_i + w_i dt
    x_i += (v_i / w_i) (sin(theta_new) - sin(theta_i))
    y_i -= (v_i / w_i) (cos(theta_new) - cos(theta_i))
```

Noise grows with speed through the `alpha` terms, so faster motion diffuses the
cloud more. Sampling the inputs (rather than adding noise to the resulting pose)
keeps every particle on a physically realisable arc.

### 14.4 Distance-to-obstacle field

For a scan endpoint `q`, the measurement model needs the distance to the nearest
surface. Computing this per particle per beam with ray casting is too slow, so a
field is precomputed once per map.

Sources are the free-facing boundary cells of obstacles, plus the world walls.
Interior blocked cells are deliberately **not** sources: otherwise a particle
pushed deep inside an obstacle would score a distance of zero and be rewarded
for an impossible pose. The field is built with an 8-connected chamfer
transform, with cardinal step `1` and diagonal step `sqrt(2)`, then multiplied
by the grid resolution:

```text
D[i, j] = resolution * min_source (chamfer distance in cells)
```

The distance used for scoring is

```text
d(x, y) = min( D[cell(x, y)],  distance to the nearest world wall )
```

The wall term is analytic, `min(x, W_x - x, y, W_y - y)`, which removes the
one-cell plateau that a rasterised border would otherwise create. Points outside
the grid have infinite distance and are treated as maximally unlikely.

### 14.5 Measurement model

For beam `k` with range `r_k` and body-frame angle `phi_k`, particle `i` places
the endpoint at

```text
q_ik = pose_i (+) ( mount + r_k [cos phi_k, sin phi_k] )
```

The endpoint likelihood uses the distance field:

```text
p(z | x_i) proportional to exp( -0.5 * lambda * sum_k ( min(d(q_ik), d_max) / sigma_z )^2 )
```

The cap `d_max` bounds the penalty so that a single grossly inconsistent beam
cannot dominate the sum. The temperature `lambda` is essential: treating all
beams as independent and multiplying their likelihoods makes the posterior
extremely sharp, so one of 500-800 particles wins by chance and the filter
collapses. `lambda = 0.2` flattens the likelihood enough that the cloud can move
coherently toward the true pose. This is an annealed likelihood, not a
probabilistically exact one, and it is documented as such.

To keep the update cheap, at most `max_beams` returns are used, evenly spaced
over the valid indices. The scan's hit mask excludes misses, so `max_range`
returns do not contribute.

### 14.6 Degeneracy and resampling

The effective sample size is

```text
ESS = 1 / sum_i w_i^2
```

and ranges from `1` (one particle carries all weight) to `N` (uniform weights).
Resampling triggers when `ESS < N / 2`, so the filter only duplicates particles
when the measurement has actually concentrated the posterior.

Systematic resampling draws one offset `u_0 ~ U(0, 1/N)` and uses
`u_i = u_0 + i/N`, selecting particles by the cumulative weight. It has lower
variance than multinomial resampling and preserves the sample count exactly.
A small Gaussian jitter (roughening) is added after resampling so duplicated
particles do not become permanently identical; without it, process noise is the
only way for the cloud to regain diversity.

### 14.7 Pose estimate

The reported pose is the weighted mean with a circular heading mean:

```text
x_hat = sum w_i x_i
y_hat = sum w_i y_i
theta_hat = atan2( sum w_i sin theta_i, sum w_i cos theta_i )
```

The circular form avoids the wrap-around error of averaging `+pi` and `-pi`
directly. Weighted marginal standard deviations of `x` and `y` are also exposed
as a cheap uncertainty estimate.

### 14.8 Measured behaviour

Closed-loop Phase 5 demo: 29.17 m route, 20 Hz, 800 particles, 60 beams per
update, LiDAR `sigma = 0.02 m`, wheel scales `0.99` and `1.01`.

| Metric | Odometry | Particle filter |
|---|---:|---:|
| Position RMS [m] | 11.395 | **0.085** |
| Position max [m] | 23.875 | **0.122** |
| Final position error [m] | 23.875 | **0.080** |
| Heading RMS [deg] | 64.87 | **0.19** |
| Final heading error [deg] | 110.04 | **0.07** |

The robot reaches the goal with zero collisions while the controller consumes
only the filter estimate. The 1% wheel mismatch accumulates almost two radians
of heading error over the 59 s run, which is why raw dead reckoning ends tens of
metres away.

Stationary recovery with a deliberately offset prior shows that the filter's
capture range depends on honest prior spread:

| Prior position std | Error after 1 update | Error after 30 updates |
|---:|---:|---:|
| 0.3 m | 0.45 m | 0.41 m |
| 0.5 m | 0.22 m | 0.12 m |
| 0.8 m | 0.07 m | 0.08 m |

A prior that excludes the true pose cannot be rescued by resampling, because no
particle is near the correct mode. Wider, honest uncertainty converges close to
the tracking accuracy. Global relocalization and recovery behaviours are Phase 6.

### 14.9 Limitations

1. The map is assumed known and static. There is no mapping, loop closure or
   dynamic-obstacle handling.
2. Localization is 2D; roll, pitch and sensor height are ignored.
3. The likelihood is annealed for robustness, so the reported posterior is not a
   calibrated probability and the particle spread underestimates true error.
4. The capture range is limited by particle coverage. A wrong prior wider than
   the cloud, or a kidnapped robot, requires global initialization or injection.
5. Beam endpoints are scored independently; there is no correlation model for
   surfaces, clutter or outliers.
6. The scan is sampled to `max_beams`, which can miss a small obstacle that only
   a few rays see.
7. Wheel scale error is constant. Real encoders also have random slip, which the
   motion noise only approximates.
8. There is no dynamic replanning; the planned path is fixed for the whole run.

---

## 15. Dynamic obstacles and recovery (Phase 6)

### 15.1 Detecting unexpected returns

Let `M(c) in {0, 1}` be the known occupancy of cell `c` and let
`q_1, ..., q_K` be the world-space LiDAR endpoints of one scan. The detector
does not trust `M` blindly: a return is *unexpected* when it lands in a cell
that the known map, including a one-cell tolerance ring around known obstacles,
marks free.

Two special cases are handled before accumulation:

1. **World walls are implicit.** The planning grid does not store the outer
   boundary as blocked cells, so returns from it would otherwise look new. The
   outer ring of cells is treated as known.
2. **Known obstacle surfaces are already blocked** or within the tolerance ring,
   including the inflation margin, so they do not generate evidence.

For each unexpected return, evidence is accumulated per cell. With `n_t(c)`
endpoints landing in `c` during scan `t`:

```text
E_{t+1}(c) = gamma E_t(c) + n_t(c),    0 <= gamma < 1
```

The decay term makes the memory finite. A persistent obstacle is re-observed
every scan and stays confirmed; a one-off noisy return fades away. Confirmation
uses hysteresis:

```text
C_t(c) = ( C_{t-1}(c) OR E_t(c) >= theta_confirm )
         AND NOT ( E_t(c) < theta_release )
```

with `theta_release < theta_confirm`. The blocked set added to the planner is
the confirmed set grown by a clearance radius and clipped against the known
map:

```text
B_t = dilate(C_t, ceil(delta_clear / resolution)) \ M
```

Growing by `delta_clear` is what gives the planner room for the robot body. The
planning grid used by A* is `M union B_t`. The detector returns only the cells
that became blocked on the current scan, so the supervisor knows when a replan
may be needed.

The default parameters are `gamma = 0.8`, `theta_confirm = 2`,
`theta_release = 0.5`, one cell of known-map tolerance and `0.25 m` clearance.
The Phase 6 demo uses `0.30 m` clearance to match its planning inflation.

### 15.2 Replanning

After a scan, if any cell on the current path is now blocked, the supervisor
replans. The start cell is the cell containing the current pose; if that cell is
blocked (for example because a new obstacle's clearance covers the robot), a
ring search returns the nearest free cell:

```text
for radius = 1, 2, ..., R:
    for cells on the square ring:
        if free, choose the closest one
return failure if no free cell is found
```

A* then searches `M union B_t` from that start to the unchanged goal. Success
replaces the path, resets the follower and continues driving. Failure enters the
recovery state machine.

Checking only the path cells is sufficient because consecutive grid path cells
are 8-neighbours and corner cutting is disabled; a blocked segment implies a
blocked cell on the path.

### 15.3 Recovery state machine

| State | Command | Exit condition |
|---|---|---|
| `DRIVING` | path-follower twist | goal reached, path blocked, or no progress |
| `SCANNING` | `(0, +scan_speed)` | `scan_steps` elapsed, then replan |
| `BACKING_UP` | `(backup_speed, 0)` | `backup_steps` elapsed, then replan |
| `ARRIVED` | `(0, 0)` | terminal |
| `FAILED` | `(0, 0)` | terminal |

Transitions:

```text
DRIVING --path blocked, replan fails--> SCANNING
DRIVING --no displacement for W steps--> SCANNING
SCANNING --replan succeeds--> DRIVING
SCANNING --replan fails--> BACKING_UP
BACKING_UP --replan succeeds--> DRIVING
BACKING_UP --replan fails or back is not free--> FAILED
DRIVING --goal within tolerance--> ARRIVED
```

Scanning rotates in place while still integrating scans, so a temporary or
partially observed obstacle has another chance to be resolved. Backing up is
short and checks both the planning grid and the world boundary before moving.
Failure is an explicit stop; the supervisor never keeps driving into a known
blocked route.

Default parameters in the implementation are `scan_steps = 30` at
`scan_speed = 1.0 rad/s`, `backup_steps = 20` at `backup_speed = -0.15 m/s`, and
a stall window of 120 steps with a 0.10 m displacement threshold. At 20 Hz this
is 1.5 s of scanning and 0.15 m of backing up.

### 15.4 Measured behaviour

Phase 6 demo: 29.17 m route, 20 Hz, `0.30 m` planning inflation, a
`0.60 x 0.60 m` block dropped on the route at `t = 4.0 s`.

| Metric | Reactive | Blind |
|---|---:|---:|
| Reached goal | **yes** | no |
| Collisions | **0** | 1 |
| Duration [s] | 58.8 | 10.0 |
| Final distance to goal [m] | **0.10** | 20.00 |
| Detected cells | 56 | - |
| Replans | 1 | 0 |

The block is visible from 3.2 m away, so the first post-injection scan provides
enough evidence to confirm it and the replan happens one control step later
(`t = 4.05 s`). The robot detours to the right of the block and still reaches the
goal with zero collisions. The blind baseline keeps following the stale path and
stops at the block after 10 s, 20 m from the goal.

A statically sealed map (no route from the start) exercises the no-path branch:
the supervisor scans for three steps, backs up for two, then reports `FAILED`
with a zero command. No collision is attempted.

Detection latency depends on how many rays hit a cell per scan. A nearby face
seen by many rays confirms in one scan; a distant or grazing surface needs
several scans as evidence accumulates.

### 15.5 Limitations

1. Only observed surfaces are added to the map. Occluded or unseen parts of a
   new obstacle remain unknown, and a planner can route through space that is
   actually blocked.
2. Moving objects are treated as static cells. There is no object tracking,
   velocity estimation or prediction.
3. Evidence decay can forget a real obstacle once it leaves the field of view.
4. The known map is assumed correct; mapping errors and changes to known
   obstacles are not repaired.
5. Replanning is grid A* with the existing follower. There is no path smoothing,
   kinodynamic planning, or time-optimal recovery.
6. The recovery sequence is fixed, not learned or adapted to the failure type.
7. The Phase 6 demo uses ground-truth pose to isolate perception and recovery.
   The supervisor accepts any pose source, but combining it with the Phase 5
   estimate and a dynamic distance field is future work.
8. A newly confirmed obstacle is inflated by a fixed clearance; very tight
   corridors can therefore be declared impassable even when a smaller robot
   could pass.

---

## 16. Evaluation and metrics (Phase 7)

### 16.1 Scenarios as descriptions

An experiment is described by a `Scenario` dataclass: a world **factory**, start
and goal points, resolution, inflation, timestep, step cap, controller choice, a
yaw disturbance, and an optional mid-run obstacle injection. The world is a
factory rather than a shared object so each run starts from the same map and an
injected obstacle cannot leak into the next seed.

Two kinds of scenario are deliberately mixed:

- **Deterministic** — kinematics and planning do not consume a seed. Repeating
  them is expected to produce identical metrics apart from wall-clock time.
- **Stochastic** — localization and recovery use the seed for LiDAR range noise
  and (where present) particle-filter sampling. Aggregation across seeds is what
  exposes run-to-run spread.

The suite prints every scenario with the same aggregation table, so a change in
one layer cannot quietly use different reporting rules.

### 16.2 Tracking metrics

Let the planned route be the polyline `P = (p_0, ..., p_n)` and a driven pose be
`q = (x, y, theta)`. For each segment `(a, b)` the projection parameter is

```text
t* = clamp( ((q - a) . (b - a)) / ||b - a||^2,  0, 1 )
```

and the cross-track distance is

```text
d(q) = || q - (a + t* (b - a)) ||
```

The segment heading is `psi = atan2(b_y - a_y, b_x - a_x)`, so the heading error
is `wrap(psi - theta)`. A pose is scored against the segment with the smallest
`d`. Over `N` samples:

```text
cross-track RMS = sqrt( (1/N) sum_i d_i^2 )
cross-track max = max_i d_i
heading RMS     = sqrt( (1/N) sum_i wrap(psi_i - theta_i)^2 )
```

The remaining tracking metrics are:

- **final error** — Euclidean distance from the last pose to the route goal.
- **control effort** — `sum |w_i| dt`, the total commanded rotation.
- **driven length** — arc length of the logged pose sequence.
- **duration** — simulated time, `steps * dt`, not wall-clock time.
- **collisions / reached** — simulator collision count and follower success.

Cross-track error is only meaningful for non-self-intersecting routes; a route
that crosses itself makes "nearest segment" ambiguous.

### 16.3 Localization metrics

At each step, with ground-truth pose `p_t` and estimates `p_hat_t` from the
particle filter and from raw odometry:

```text
e_t = || p_hat_t - p_t ||_2
position RMS = sqrt( (1/N) sum_t e_t^2 )
position max = max_t e_t
heading RMS  = degrees( sqrt( (1/N) sum_t wrap(theta_hat_t - theta_t)^2 ) )
```

The odometry and filter metrics are computed over the **same** steps and the
same commanded motion, so the comparison isolates the measurement update rather
than a different route or controller.

### 16.4 Planning metrics

The runner computes both searches on the same inflated grid and reports:

```text
path cost [m]        = A* cost * resolution
waypoints            = len(A* path)
expansion reduction  = 1 - expanded(A*) / expanded(Dijkstra)
```

The runner raises if A* and Dijkstra disagree on cost, which would indicate an
inadmissible or inconsistent heuristic. Wall-clock milliseconds are reported for
the Phase 2 comparison but are machine-dependent and excluded from the
reproducibility claim.

### 16.5 Aggregation across seeds

For metric values `x_1, ..., x_n` the benchmark reports

```text
mean = (1/n) sum x_i
std  = sqrt( (1/(n-1)) sum (x_i - mean)^2 )      (n > 1)
min, max
```

The standard deviation uses `ddof = 1`, so it estimates the spread of the
run-to-run distribution rather than describing only the sampled values. With a
single seed it is reported as zero. These are descriptive statistics over a
small sample: they are not confidence intervals and no significance test is
applied.

### 16.6 Determinism

Given the same scenario and seed, a run must produce identical metrics. This is
achieved by:

1. constructing a fresh world per run;
2. seeding the LiDAR and the particle filter from the scenario seed;
3. keeping wall-clock time out of the primary metrics.

The recovery injection is applied at a fixed control step, so the same run
receives the same obstacle at the same simulated time. The tracking scenario is
deterministic even under the injected yaw bias because the simulator and
controller have no random terms.

### 16.7 Measured behaviour

Full suite, three seeds (0, 1, 2), 18 runs, about 9.4 s on the development
machine:

| Scenario | Headline metric | Mean | Std |
|---|---|---:|---:|
| kinematics | Euler error over one circle [m] | 2.10e-4 | 0 |
| | midpoint error [m] | 8.76e-7 | 0 |
| | exact error [m] | 6.27e-15 | 0 |
| planning | A* cost [m] | 29.021 | 0 |
| | waypoints | 103 | 0 |
| | cells expanded (A* / Dijkstra) | 1074 / 4465 | 0 |
| | expansion reduction | 75.9% | 0 |
| tracking, P-only | cross-track RMS [m] | 0.0513 | 0 |
| tracking, PID | cross-track RMS [m] | **0.0258** | 0 |
| | final error [m] | 0.0982 | 0 |
| | collisions | 0 | 0 |
| localization | odometry position RMS [m] | 11.383 | 0.010 |
| | filter position RMS [m] | **0.0845** | 0.0004 |
| | filter heading RMS [deg] | 0.181 | 0.005 |
| | collisions | 0 | 0 |
| recovery | reached | 1.0 | 0 |
| | collisions | 0 | 0 |
| | replans | 1 | 0 |
| | detected cells | 56 | 0 |
| | final error [m] | 0.0987 | 0 |

The PID cross-track result reproduces the Phase 3 demo, and the localization and
recovery results reproduce the Phase 5 and Phase 6 demos. The harness therefore
agrees with the narrative experiments rather than replacing them with a second,
divergent implementation.

### 16.8 Limitations

1. The suite is small: one map, one route, five scenario kinds and three seeds.
   It is a regression harness, not a broad robotics benchmark.
2. Aggregation reports descriptive statistics only. There are no confidence
   intervals, paired tests or effect sizes.
3. Cross-track error uses nearest-segment projection, which is ambiguous for
   self-intersecting routes.
4. Wall-clock timings depend on the machine and are excluded from the
   reproducibility claim.
5. Recovery is still evaluated on ground-truth pose. The estimate-driven
   supervisor is proposed Phase 8.
6. Collision count is binary per run; the metrics do not record minimum
   clearance, jerk, comfort or actuator saturation.
7. Seeds are varied for stochastic components, but the map and start pose are
   fixed, so map uncertainty is not represented.
8. The per-phase demo scripts remain narrative experiments. They share the same
   underlying components but do not call the harness, so a future change could
   drift unless both are run.

---

## 17. Integrated autonomy (Phase 8)

### 17.1 The closed loop

Phase 6 evaluated recovery on ground-truth pose and Phase 5 evaluated control on
the particle-filter estimate, but the two loops were never run together. Phase 8
connects them:

```text
LiDAR scan -> particle filter -> pose estimate
                    |                  |
                    v                  v
             map consistency      path follower
                    |                  |
                    v                  v
             dynamic map ------> A* replan -> wheel commands
```

Only the simulator and the error metrics see the true pose. The supervisor, the
detector and the controller all consume the estimate produced by the filter.

The integrated runner is the same loop as `run_recovery`, with three changes:

1. the supervisor receives `pf.estimate()` instead of the simulator pose;
2. the wheel commands are converted through the odometry model before the
   filter's predict step;
3. the detector interprets the scan at the estimated pose rather than the
   ground-truth sensor origin.

### 17.2 Interpreting a scan at the estimated pose

A `LidarScan` stores body-frame angles `alpha_i`, ranges `r_i`, the mount offset
`b`, and the sensor origin where the scan was generated. `scan.points()` uses
that stored origin. For integrated navigation the detector must not use it,
because it is ground truth.

Instead, `scan.points_at(pose)` reconstructs each hit from the measurement and
the estimate:

```text
body_i   = b + r_i [cos(alpha_i), sin(alpha_i)]
world_i  = [x_est, y_est] + R(theta_est) body_i
```

with

```text
R(theta) = [[cos(theta), -sin(theta)],
            [ sin(theta),  cos(theta)]]
```

This is a change of coordinate frame, not a new measurement. The ranges and
body angles are unchanged, so the detector still sees what the sensor measured;
only the assumed sensor pose moves from truth to estimate.

### 17.3 Boundary band

World walls are implicit in the planning grid: the grid stores free space up to
the boundary, and the detector used to treat only the outermost cell ring as
known. With an estimated pose, a return from a boundary wall can land one or two
cells inside that ring, especially at long range where a small heading error
produces a large lateral displacement:

```text
lateral error ~ range * heading error
```

At 10 m and 0.03 rad (1.7 degrees) this is 0.3 m, or more than one 0.25 m cell.
Those returns were being confirmed as new obstacles. The detector now marks a
band of `tolerance + 1` cells along every world edge as known. With the default
`tolerance = 1` this is a 0.5 m band, which removed the false boundary
detections without hiding the injected block 2.3 m away from the wall.

### 17.4 Metrics

The integrated run reports the navigation metrics from Phase 6 plus the
localization metrics from Phase 5:

```text
reached, failed, collisions, duration, final_error, replans, detected_cells
odometry_rms, odometry_max, filter_rms, filter_max, filter_heading_rms_deg
```

The true pose is used only after each step to compute the odometry and filter
errors. The controller never receives it.

### 17.5 Measured results

Full benchmark, three seeds (0, 1, 2), 800 particles, 60 beams:

| Metric | Mean | Std |
|---|---:|---:|
| goal reached | 1.0 | 0 |
| collisions | 0 | 0 |
| duration [s] | 59.03 | 0.03 |
| final error [m] | 0.061 | 0.004 |
| replans | 1 | 0 |
| detected cells | 65.7 | 2.9 |
| odometry position RMS [m] | 11.301 | 0.006 |
| filter position RMS [m] | 0.0915 | 0.0008 |
| filter position max [m] | 0.136 | 0.011 |
| filter heading RMS [deg] | 0.617 | 0.007 |

The ground-truth recovery run from Phase 6 reports `0.0987 m` final error and 56
detected cells. The estimate-driven run is therefore slightly more accurate at
the goal and detects a few extra cells near the new block, but still finishes
with zero collisions and one replan. A single run takes about 2.1 s, so the
three-seed scenario is roughly 6 s of the benchmark.

### 17.6 Limitations

1. The particle filter still scores scans against the **static** map. The
   dynamic map is used for planning and detection, not for the measurement
   likelihood. A map-aware likelihood was prototyped but did not improve the
   filter in this scenario, so it is left as future work.
2. The boundary band is a fixed geometric margin, not a function of the
   particle covariance. It trades sensitivity near world edges for robustness
   to pose error.
3. Detection tolerance is fixed at one cell plus the boundary band. It is not
   derived from the current particle spread.
4. The filter starts from a known pose with a small offset. In this phase
   kidnapped-robot recovery and global relocalization are not yet implemented;
   Phase 9 adds them as a separate detection-and-search pipeline.
5. One map, one route and one obstacle injection are evaluated. The integrated
   scenario is a regression target, not a broad benchmark.
6. The simulation remains 2D with no wheel dynamics, slip or actuator lag.
7. Runtime is reported only as a rough wall-clock figure; the benchmark's
   reproducibility claim covers simulated time, counts and geometric errors.

## 18. Kidnapped-robot relocalization (Phase 9)

A particle filter that starts near the true pose tracks it well, but it cannot
recover from an arbitrary displacement. After a kidnap every particle is in the
wrong place, the measurement update rewards whichever wrong hypothesis happens
to fit, and no amount of resampling moves the cloud to the new pose. The fix is
not a larger cloud but a different inference step: detect that the filter is
inconsistent, run a global search over the known map, confirm the best
candidate with more than one scan, and only then restart the filter.

### 18.1 Scan residual

The filter already projects measured endpoints into the world at a candidate
pose to score them against the distance field. The same projection gives a
scalar diagnostic. For the subset `B` of valid beams used by the filter, with
body-frame endpoints `b_i = (r_i cos a_i, r_i sin a_i) + o` (`o` is the sensor
mount), a pose `p` maps them to

```text
x_i = p.x + cos(p.theta) b_i.x - sin(p.theta) b_i.y
y_i = p.y + sin(p.theta) b_i.x + cos(p.theta) b_i.y
```

and the residual is

```text
r(p) = (1 / |B|) sum_i min(D(x_i, y_i), d_max)
```

where `D` is the distance to the nearest mapped obstacle and `d_max = 1.0 m`
caps the contribution of a badly wrong beam. At the true pose the residual is
the LiDAR noise floor, about `0.03 m` clean and `0.13 m` with the 0.02 m range
noise used in the benchmark. After a 10.5 m kidnap it jumps to about `0.85 m`.
The gap is wide enough that a fixed threshold separates the two regimes
without leaning on a calibrated noise model.

`ParticleFilter.scan_residual(scan, pose=None)` returns `r` at the estimate by
default, or at a supplied pose for diagnostics. It shares the beam subset,
distance field and `max_distance` clipping with `update`, so the diagnostic and
the likelihood cannot disagree about what a scan means.

### 18.2 Lost detection

A single bad scan is normal: a doorway, a moving object or a transient range
outlier can all raise the residual. `LocalizationMonitor` therefore requires a
sustained failure. With limit `r_limit` and patience `N`:

```text
if r > r_limit:   bad_steps += 1
else:             bad_steps  = 0

trigger = (bad_steps >= N) and not already_triggered
```

One acceptable scan clears the streak, so only a persistent mismatch triggers a
search. The monitor latches after triggering, which matters because the
relocalizer runs for tens of steps while the filter is deliberately not
updated; without the latch the same failure would restart the search every
step. `reset()` rearms it after a successful recovery.

The benchmark uses `r_limit = 0.5 m` and `N = 8` at 20 Hz, so detection takes at
most `0.40 s`. The measured latency is `0.35 s` (`7` steps), because the
kidnap happens after the scan for that step has already been scored.

### 18.3 Coarse-to-fine global search

Global localization is a search over the map, not a filter update. The search
space is every free cell centre crossed with a heading grid. For the 80 x 80
grid at 0.25 m that is roughly 20,000 free cells; a full product with 36
headings is over 700,000 poses. Two reductions make it cheap and, more
importantly, keep it well behaved:

1. **Coarse positions.** Free cells are subsampled with `stride = 4`, giving a
   1 m position lattice, and paired with headings every 10 degrees. That is
   about 12,000 poses per scan, scored in roughly 10-20 ms with a vectorised
   projection.
2. **Coarse separation.** Instead of taking the top `M` scores, candidates are
   accepted in ascending-score order only if they are more than 1.5 m from
   every already-accepted candidate. This prevents three near-identical poses
   from consuming the whole budget. The default keeps `M = 4` hypotheses.

Each coarse candidate is then refined on a local grid: `+/- 0.5 m` in `0.25 m`
steps and `+/- 10` degrees in 5 degree steps, taking the best pose in that
neighbourhood, then deduplicating refined poses that are within 1.0 m of each
other. The refinement matters because a 1 m coarse lattice is coarser than the
tolerance the filter needs.

The score for a candidate is the same mean clipped residual as section 18.1,
with one addition.

### 18.4 Out-of-bounds endpoints

The distance field covers the map and merges analytic world-wall distance. A
projected endpoint can still fall outside the map rectangle when a candidate
pose is wrong: the beam is measured in one place and cast from another. The
obvious fix is to clamp the endpoint to the boundary and sample there, but that
is wrong. Clamping turns every escaped beam into a point on the wall, which is
exactly the obstacle the distance field reports at distance zero. A wrong pose
whose beams fly out of the map then scores as well as, or better than, the true
pose. During development this produced candidates that matched the scan better
than the true pose and made the whole search fail.

The implemented penalty measures how far outside the endpoint actually is:

```text
d_out = hypot(max(xmin - x, 0, x - xmax),
              max(ymin - y, 0, y - ymax))
penalty = max(d_out - wall_tolerance, 0)
```

with `wall_tolerance = 0.10 m`. The tolerance absorbs endpoints that legitimately
land a few centimetres outside because of range noise or the grid origin, while
a beam that escapes by metres is penalised by metres and clipped at `d_max`.
This change alone was the difference between a search that reliably found the
true pose and one that did not.

### 18.5 Multi-hypothesis confirmation

One scan is ambiguous. Corridors repeat, rooms have similar corners, and a 360
degree scan from a symmetric pose can fit several places in a 20 x 20 map. The
relocalizer therefore does not commit after one match. It keeps the candidate
bank from `search()`, and while the robot rotates in place it calls

```text
observe(scan, dtheta):
    accumulated[k] += score(scan, pose_k)
    observations   += 1
    pose_k.theta   += dtheta      for every k
```

Two details make the accumulation correct. The scan is scored **before** the
hypothesis heading is advanced, because the scan was measured at the current
heading. And the heading of each hypothesis is advanced by the same rotation
the robot is about to perform, so hypotheses stay aligned with the vehicle
frame rather than being re-searched each step.

After `K` observations the winning hypothesis is

```text
k* = argmin_k accumulated[k]
best = Pose(pose_{k*}, accumulated[k*] / observations)
```

Averaging by the number of observations keeps the reported residual on the same
scale as a single-scan score. Accumulation is a simple sum rather than a
recursive likelihood because the poses are not being integrated: each candidate
is a fixed world pose with a rotating heading, and the question is only which
one explains the whole rotation. In the benchmark the wrong hypotheses keep a
mean residual near `0.2 m` across the rotation while the true one sits at
`0.03 m`, so 40 steps of rotation widen a single-scan margin into a decisive
one.

### 18.6 Recovery sequence

The end-to-end loop in `simulate_relocalization` is:

1. Drive normally from the particle-filter estimate.
2. At `kidnap_step`, set the true pose to `kidnap_pose` without touching
   odometry. The filter keeps predicting from wheel commands, so its estimate
   does not move with the robot.
3. Score every scan. Once the monitor latches, run the global search on that
   scan and switch to `RELOCALIZE_SPEED = 0.6 rad/s` in place.
4. For `RELOCALIZE_STEPS = 40` steps, accumulate evidence. The filter is not
   updated during this window; updating it at the wrong pose would corrupt the
   particles and bias the eventual seed.
5. Seed the filter at the winning pose with `position_std = 0.10 m` and
   `heading_std = 0.03 rad`, then settle for `RELOCALIZE_SETTLE_STEPS = 10`
   steps with detection off. The filter needs a few scans to tighten around the
   seed before it can be trusted for control.
6. Call `NavigationSupervisor.resume(estimate)`. This clears the dynamic map and
   every detected cell, resets the follower, and replans from the corrected
   pose. Cells confirmed in the old frame describe obstacles at the wrong
   coordinates, so keeping them would block the route with phantom obstacles.
7. Rearm the monitor and continue to the goal.

`NavigationSupervisor.advance_clock()` counts the paused steps so supervisor
event timestamps stay aligned with the caller's step index. Without it the
relocalization window would disappear from the supervisor's stall accounting and
from the plotted timeline.

The emergency rotation is a deliberate simplification: the robot stops
translating while it gathers evidence. A production system would prefer to keep
moving, but a stationary rotation makes the geometric confirmation exact and
keeps the recovery bounded.

### 18.7 Metrics

`RelocalizationMetrics` adds the recovery timing and error decomposition to the
navigation metrics:

```text
detection_latency         time from kidnap to the lost decision [s]
recovery_time             time from kidnap to error < 0.5 m [s]
error_at_kidnap           estimate error immediately after the teleport [m]
post_kidnap_rms           error over the whole lost interval [m]
post_recovery_rms         error after recovery to the goal [m]
post_recovery_heading_rms heading error after recovery [deg]
filter_max                worst position error over the run [m]
filter_heading_rms_deg    heading error over the whole run [deg]
```

`post_kidnap_rms` includes the interval in which the filter is knowingly wrong,
so it is a measure of how long the robot stays lost, not of estimator quality.
`post_recovery_rms` and `post_recovery_heading_rms` are the meaningful
steady-state figures. The overall heading RMS is inflated by the lost interval
and should not be compared with the Phase 8 tracking result.

### 18.8 Measured results

Three seeds, 800 particles, 60 beams, kidnap at step 200 to
`(2.125, 16.125, 0.0)`:

| Metric | Mean | Std |
|---|---:|---:|
| goal reached | 1.0 | 0 |
| collisions | 0 | 0 |
| relocalizations | 1 | 0 |
| detection latency [s] | 0.35 | 0 |
| recovery time [s] | 2.35 | 0 |
| error at kidnap [m] | 10.531 | 0.005 |
| post-kidnap RMS [m] | 2.592 | 0.002 |
| post-recovery RMS [m] | 0.0920 | 0.0005 |
| post-recovery heading RMS [deg] | 0.58 | 0.09 |
| overall heading RMS [deg] | 14.24 | 0.03 |
| final error [m] | 0.063 | 0.003 |
| duration [s] | 47.38 | 0.03 |
| replans | 2.7 | 0.6 |
| detected cells at the end | 0 | 0 |

The `2.35 s` recovery is dominated by the deliberate search window: `0.35 s` to
detect, then 40 steps (`2.0 s`) of rotation and accumulation, then the
settling phase, with `resume()` about `0.5 s` after the pose is reacquired. The
whole search itself costs roughly 15 ms per scan, so computation is not the
bottleneck; evidence gathering is.

`detected_cells` at the end is zero because the dynamic map is cleared on
resume and this scenario injects no obstacle. During the lost interval the
detector does confirm phantom cells (up to about 190 in a step when the wrong
pose sweeps walls into the map), which is exactly why `resume()` discards them.
That number is intentionally not a success metric for this scenario.

### 18.9 Limitations

1. **One kidnap pose per scenario.** The search is stress-tested on several
   poses during development, but the benchmark scenario uses one. A broader
   claim needs a grid of kidnap poses and resulting success rates.
2. **The map is known and static.** The search scores against the static
   likelihood field. A changed world would need the dynamic map or a SLAM map,
   and this phase deliberately keeps the dynamic map out of the global search.
3. **Stationary confirmation.** The robot stops to rotate. It does not
   relocalize while driving, and it assumes the rotation is unblocked.
4. **Single-scan ambiguity is handled by rotation, not by a probabilistic
   multi-scan filter.** Accumulation works well here but is not a full
   multi-hypothesis Bayes filter; there is no explicit posterior over places.
5. **The kidnap pose must be free in the planning grid.** The scenario
   validates this, but the runner does not search for a nearby free pose.
6. **Fixed thresholds.** The residual limit and patience are constants tuned on
   this noise level. A range-dependent or covariance-aware threshold would
   generalize better.
7. **One map, one route, three seeds.** As with the rest of the benchmark, this
   is a regression target rather than a broad evaluation.
8. **No loop closure or map repair in this phase.** The search scores against
   the static map, and repeated failures are not escalated to a mapping
   behaviour. Phase 10 addresses map repair, but loop closure remains out of
   scope.

## 19. Occupancy mapping and map repair (Phase 10)

Phases 5-9 score every scan against a map that is assumed correct. A phantom
obstacle makes the robot avoid free space, and a missing obstacle makes it plan
straight through solid geometry. Phase 10 treats the map as a random field that
LiDAR updates: free space is evidence against occupancy, and an endpoint is
evidence for it. The same filter that localizes the robot then feeds the map,
and the planner consumes the repaired map.

This is mapping and map repair at the estimated pose, not full SLAM. There is no
joint posterior over poses and the map and no pose-graph loop closure. The
distinction is deliberate and is restated in section 19.7.

### 19.1 Log-odds occupancy

For a cell `m`, let

```text
L(m) = log( p(m = occupied) / (1 - p(m = occupied)) )
```

so `L = 0` is an even bet, `L > 0` favours occupied and `L < 0` favours free.
The map starts from the prior occupancy grid with

```text
L_prior(m) = +1.5    if the prior marks m occupied
L_prior(m) = -0.75   if the prior marks m free
```

A positive prior is not symmetric with the free prior because the downstream
planner is conservative: an unobserved occupied cell is safer than an
unobserved free one. The benchmark then applies an independent beam model. For
beam `k` from sensor origin `o` at world angle `a`, with measured range `r` and
maximum range `R`:

```text
hit  = r < R - epsilon
free_length = r - free_margin    if hit
free_length = r                  otherwise
```

Every grid cell crossed by the ray up to `free_length` receives

```text
L(m) <- L(m) + l_miss,        l_miss = -0.45
```

and the cell containing the endpoint

```text
e = o + r [cos a, sin a]
```

receives

```text
L(m) <- L(m) + l_hit,         l_hit = +0.85
```

The update is clamped to `[-4.0, +4.0]` to bound the influence of any one cell
and keep the map responsive to later evidence. Occupancy is the threshold

```text
occupied(m) = known(m) and L(m) > 0
```

A cell with `L = 0` is not occupied; unknown cells are stored explicitly in a
boolean `known` mask. The default prior marks the whole raster known, so
`known_ratio = 1`. The planning grid maps unknown cells to free. That optimism
is intentional: the planner should not refuse a route merely because the robot
has not looked there yet. The cost is that an unknown obstacle can invalidate a
route later, which is exactly what the replanning loop is for.

Rays are sampled every half cell rather than with a closed-form grid traversal.
The sample indices within one beam are deduplicated, so a cell crossed twice by
the same beam is penalised once. Incidences from different beams still add,
which lets a surface observed by several returns become occupied faster than a
grazing ray suggests.

With these constants, one hit on a prior-free cell gives `-0.75 + 0.85 = +0.10`
and therefore adds it to the map. A prior-occupied phantom cell at `+1.5` needs
four independent misses to reach `1.5 - 4(0.45) = -0.30` and disappear. The
asymmetry is useful in this scenario: a missed obstacle is added quickly, while
a phantom is removed only after repeated free observations. The free margin of
`0.15 m` keeps the ray from carving the cell that the same beam reports as
occupied, which would otherwise cancel the hit at the endpoint.

### 19.2 Map repair as an observed-surface error

A mapped obstacle is reconstructed from returns, so its interior is not
observed; only its free-facing surface is. Comparing a mapped grid with a solid
rasterised obstacle over all blocked cells would count every unobserved
interior cell as an error and understate repair. The metric therefore restricts
the comparison to surface cells: blocked cells with at least one free
four-neighbour. For predicted map `P` and truth `T`, both restricted to surface
sets `S(P)` and `S(T)`,

```text
E(P, T) = | (S(P) symmetric-difference S(T)) and observed |
```

The `observed` mask contains every cell the robot has carved or hit. Without it,
a phantom that the robot never approached would count as an error for both maps,
and a genuine repair in the visited region would look small. With it, the metric
answers the question that matters during operation: over the part of the world
the robot has actually measured, how many surface cells are wrong?

The headline repair number is

```text
repair_ratio = (E_prior - E_final) / E_prior
```

when `E_prior > 0`, and `1.0` when the prior is already perfect. It is a
fraction of the observable prior error that mapping removed, not a global map
accuracy claim.

### 19.3 Planning on the evolving map

`MappingNavigator` wraps a `PathFollower` and a `LogOddsMap`. The default
inflation is the scenario's `0.30 m`, matching the body radius plus margin used
elsewhere. On each control step it

1. integrates the scan at the pose estimate,
2. bumps a `map_version` counter when the occupancy changed,
3. replans if newly added cells intersect the current route,
4. otherwise replans on a fixed interval when phantom cells were removed,
5. follows the current route with the existing carrot-chasing controller.

The planning grid is

```text
G_t = inflate( threshold(L_t), inflation )
```

and is cached until `map_version` changes. Inflation is computed by the same
`OccupancyGrid.inflated` routine as Phases 2-9, so the clearance claim is
identical.

The replan policy separates the urgent case from the cosmetic one. Added
evidence that blocks the route is safety-critical and triggers an immediate
replan. Removed evidence makes the old route valid but possibly longer; it
triggers at most one best-effort replan per `replan_interval` steps. A failed
best-effort replan keeps the old path. A failed urgent replan stops the robot
and sets `failed`, never a collision. The navigator also refuses to plan when
the goal itself has become blocked, which can happen if a map update adds cells
around the goal; this is a guard rather than a scenario expectation.

The start cell for planning is snapped to the nearest free cell, reusing the
recovery module's `nearest_free_cell`. This matters when newly added inflation
surrounds the estimated pose: the planner needs a valid graph node, but the
controller still uses the true estimate.

### 19.4 Coupling the filter to the repaired map

The particle filter scores endpoints against a precomputed distance field built
from one grid. If the map changes and the field does not, newly discovered
obstacles keep scoring as measurement outliers even after they are in the map.
`ParticleFilter.replace_grid(grid)` swaps the grid, rebuilds the
`DistanceField`, and leaves the particles and weights untouched. The harness
calls it whenever `map_version` changes.

This is a pragmatic split. The map is updated in the mapping module at the
filter estimate, and the filter consumes the updated map without trying to
infer a joint pose-map posterior. The loop is therefore:

```text
scan + estimate -> LogOddsMap -> planning grid -> MappingNavigator
                      |                              |
                      +--> replace_grid --> ParticleFilter
```

### 19.5 Scenario and metrics

The `slam` scenario uses the standard 20 x 20 world as truth but gives the robot
a different prior:

- truth contains a block at `(6, 14, 7, 15)`, in the gap above the long wall;
- the prior omits that block;
- the prior contains a phantom rectangle at `(1.0, 5.0, 2.2, 8.0)` that is free
  in truth.

The start is `(1.125, 1.125)` and the goal `(18.125, 18.125)`, with the same
`0.25 m` raster and `0.30 m` inflation as the other navigation scenarios. The
prior route must detour around the phantom and crosses the wall gap at the
height where the missing block sits, so the scenario exercises both directions
of repair and a route change.

`MappingMetrics` reports the navigation outcome plus:

```text
map_updates              steps whose occupancy changed
added_cells              unique cells that became occupied
removed_cells            unique cells that became free
known_ratio              fraction of the map marked known
observed_ratio           fraction carved or hit
prior_map_error          E_prior on the finally observed surface
final_map_error          E_final on the same surface
map_error_reduction      E_prior - E_final [cells]
map_repair_ratio         reduction / E_prior
filter_rms, filter_max   position error against truth [m]
filter_heading_rms_deg   heading error against truth [deg]
```

The error timeline is sampled every ten steps and uses the observed mask at that
moment. It is a diagnostic curve, not a monotone guarantee: newly observed
regions can add error later, and a moving surface can flicker.

### 19.6 Measured results

**Pending.** The Phase 10 implementation, tests and demo were authored with an
explicit no-run instruction, so this document does not report measured map
repair, runtime, localization error or added/removed cell counts. The
acceptance targets are:

- the robot reaches the goal with zero collisions;
- at least one replan follows the map change;
- both `added_cells` and `removed_cells` are non-zero;
- observed-surface map error falls (`final_map_error < prior_map_error`);
- filter position RMS stays consistent with Phases 8-9 (well below `0.5 m`).

Run `python scripts/phase10_demo.py` and `pytest` to fill in the table. Until
then, no Phase 10 number should be treated as a regression baseline.

### 19.7 Limitations

1. **No loop closure and no pose-graph optimization.** The map is built at the
   particle-filter estimate. Small pose errors accumulate into a smeared map on
   a long loop, and revisiting a place does not correct earlier geometry. This
   phase delivers map repair, not full SLAM.
2. **A prior map is still required.** The module repairs an existing grid; it
   does not build a map from an unknown world or decide where to explore.
3. **Static-world assumption.** Occupancy is a per-cell log-odds with no object
   persistence or dynamics. A moving obstacle leaves hits along its path and
   free evidence behind it, which appears as a smear rather than a tracked
   object.
4. **Simple inverse sensor model.** Log-odds increments are fixed constants with
   no range-dependent uncertainty, beam-width model or ray-cast-through-
   uncertainty treatment. This keeps the update vectorisable and explainable,
   but it is not a calibrated probabilistic sensor model.
5. **Surface-only scoring.** The map error deliberately ignores obstacle
   interiors and unobserved regions. It measures repair of what was observed,
   not global map accuracy.
6. **Coverage is passive.** The route is chosen for navigation, not for
   information gain, so cells far from the route remain at their prior values.
7. **Single scenario, single route.** As with the rest of the benchmark, this is
   a regression target rather than a broad evaluation of mapping performance.
8. **Verification is deferred.** The no-run policy means the measured numbers in
   section 19.6 are still open.
