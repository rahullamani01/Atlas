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
