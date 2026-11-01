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
