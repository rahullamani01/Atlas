# Atlas (Project ATLAS)

Autonomous Navigation and Learning System: a simulation-based mobile robot
project in Python (motion model, planning, control, sensing, localization).

## Status

| Phase | Scope | State |
|---|---|---|
| 0 | Geometry utilities, conventions, project scaffold | complete |
| 1 | Differential-drive kinematics and motion simulation | complete |
| 2 | Occupancy grids, Dijkstra, A* path planning | complete |
| 3 | PID control and closed-loop path following | complete |
| 4 | LiDAR simulation and range uncertainty | complete |
| 5 | Odometry and particle-filter localization | complete |
| 6 | Dynamic obstacles and recovery | complete |
| 7 | Evaluation and metrics | complete |

Phase 7 turns the per-phase demo measurements into a reusable evaluation package
and one aggregate benchmark. Six scenarios (kinematics, planning, PID and P-only
tracking, localization, recovery) run across several seeds and report mean,
standard deviation, minimum and maximum for every metric. The harness reproduces
the Phase 3, 5 and 6 numbers rather than redefining them, so a change that
regresses tracking, localization or recovery shows up in one command.

## Layout

```text
src/atlas/
├── geometry.py                  # angles, distance, transforms
├── robot/
│   ├── differential_drive.py    # pose, wheel <-> twist, motion integration
│   └── odometry.py              # dead reckoning with wheel-scale error
├── planning/
│   ├── grid.py                  # occupancy grid, inflation, neighbours
│   └── search.py                # Dijkstra, A*, heuristics, path reconstruction
├── control/
│   ├── pid.py                   # PID with clamps and anti-windup
│   └── path_follower.py         # carrot chasing with PID heading control
├── sensors/
│   └── lidar.py                 # 2D ray casting, field of view, range noise
├── localization/
│   ├── likelihood_field.py      # distance-to-obstacle field
│   └── particle_filter.py       # motion/measurement models, resampling
├── perception/
│   └── obstacle_detector.py     # map-inconsistent returns, evidence, clearance
├── recovery/
│   └── supervisor.py            # replanning and the recovery state machine
├── evaluation/
│   ├── metrics.py               # RMS, cross-track, summaries, metric dataclasses
│   ├── scenarios.py             # named scenario registry and validation
│   └── harness.py               # runners, suite execution, aggregation
└── simulation/
    ├── world.py                 # boundaries and rectangular obstacles
    └── simulator.py             # timestep loop, collisions, trajectory log
tests/                           # 212 tests across geometry, motion, planning, control, sensing, localization, perception, recovery, evaluation
scripts/
├── phase1_demo.py               # kinematics and integration-error experiments
├── phase2_demo.py               # Dijkstra vs A* on an inflated grid
├── phase3_demo.py               # P-only vs PID closed-loop tracking
├── phase4_demo.py               # LiDAR scans at four poses, clean vs noisy
├── phase5_demo.py               # odometry vs particle-filter localization
├── phase6_demo.py               # reactive vs blind navigation with a new obstacle
└── evaluate.py                  # Phase 7 scenario suite with JSON/CSV export
docs/
├── design_decisions.md
├── mathematics.md               # derivations for kinematics, search, control, sensing, localization, recovery, evaluation
└── handoff.md                   # project state and next-stage plan
```

## Setup

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"
pytest
```

## Demos

```bash
python scripts/phase1_demo.py
python scripts/phase2_demo.py
python scripts/phase3_demo.py
python scripts/phase4_demo.py
python scripts/phase5_demo.py
python scripts/phase6_demo.py
python scripts/evaluate.py
```

- **Phase 1:** straight line, in-place rotation, and integration error against
  the exact solution, plus circle and collision plots.
- **Phase 2:** builds an 80 × 80 occupancy grid, plans with Dijkstra and A*,
  and reports cost, expanded cells and runtime. Plot:
  `experiments/plots/phase2_planning.png`.
- **Phase 3:** plans the same map, then follows the path at 20 Hz with P-only
  and PID heading control under a wheel bias. Reports time, final error,
  cross-track and heading RMS, control effort and collisions. Plot:
  `experiments/plots/phase3_tracking.png`.
- **Phase 4:** scans the same map from four poses with a 360° LiDAR, showing
  occlusion, misses and Gaussian range noise. Plot:
  `experiments/plots/phase4_lidar.png`.
- **Phase 5:** drives the Phase 3 route under a 1% wheel-scale mismatch, using
  the particle-filter estimate for control. Compares odometry drift with the
  fused estimate and shows recovery from offset priors. Plot:
  `experiments/plots/phase5_localization.png`.
- **Phase 6:** drops a new block on the route mid-run and compares reactive
  navigation against a blind baseline. Reports detection, replanning, collisions
  and goal distance, and shows the dynamic obstacle map growing. Plot:
  `experiments/plots/phase6_recovery.png`.
- **Phase 7:** runs all six scenarios over several seeds and prints aggregate
  tables (mean/std/min/max) for kinematics, planning, tracking, localization and
  recovery. `--scenario`, `--seeds`, `--json` and `--csv` select subsets and
  export results.

## Motion model in brief

Given wheel angular velocities `omega_l`, `omega_r`, wheel radius `r` and
separation `L`:

```text
v = r (omega_r + omega_l) / 2
w = r (omega_r - omega_l) / L
```

Drive the robot with either form:

```python
from atlas.robot.differential_drive import DifferentialDriveRobot, Pose

robot = DifferentialDriveRobot(pose=Pose(0, 0, 0))
robot.step_wheels(omega_l=10.0, omega_r=10.0, dt=0.05)  # forward
robot.step(v=0.5, w=0.0, dt=0.05)                       # same, as a twist
```

## Planning in brief

```python
from atlas.planning import OccupancyGrid, a_star
from atlas.simulation.world import RectObstacle, World

world = World(20, 20, [RectObstacle(6, 0, 7, 14)])
grid = OccupancyGrid.from_world(world, resolution=0.25, inflate_radius=0.2)

start = grid.world_to_cell(1.0, 1.0)
goal = grid.world_to_cell(18.0, 18.0)

result = a_star(grid, start, goal)   # octile heuristic, 8-connected
if result.found:
    print(result.cost * grid.resolution, "metres", len(result.path), "waypoints")
```

## Control in brief

```python
from atlas.control import PID, PathFollower
from atlas.planning import a_star
from atlas.robot.differential_drive import Pose

result = a_star(grid, start, goal)
path = [grid.cell_to_world(cell) for cell in result.path]

follower = PathFollower(
    lookahead=0.5,
    heading_pid=PID(kp=2.0, ki=0.2, kd=0.05, output_limit=1.5),
)
command = follower.compute(Pose(1.125, 1.125, 0.0), path, dt=0.05)
print(command.v, command.w)          # m/s, rad/s
```

## Sensing in brief

```python
import math

from atlas.sensors import Lidar
from atlas.robot.differential_drive import Pose

lidar = Lidar(max_range=8.0, fov=2 * math.pi,
              angle_increment=math.pi / 180,
              range_sigma=0.03, seed=7)
scan = lidar.scan(world, Pose(2.0, 2.0, 0.0))

print(scan.ray_count, scan.hits, scan.ranges.min())
print(scan.points().shape)           # world-space hit points, (N, 2)
```

## Localization in brief

```python
from atlas.localization import ParticleFilter
from atlas.robot.odometry import Odometry

odometry = Odometry(left_scale=0.99, right_scale=1.01)
pf = ParticleFilter(grid, n_particles=800, max_beams=60,
                    likelihood_scale=0.2, seed=4)
pf.initialize(Pose(1.125, 1.125, 0.0), position_std=0.10, heading_std=0.05)

odometry.update_wheels(omega_l, omega_r, dt=0.05)
v, w = odometry.wheel_speeds_to_twist(omega_l, omega_r)

pf.predict(v, w, 0.05)
pf.update(lidar.scan(world, robot.pose))
pose = pf.estimate()                 # weighted mean, circular heading
print(pose.x, pose.y, pose.theta)
```

## Recovery in brief

```python
from atlas.perception import DynamicObstacleMap
from atlas.recovery import NavigationSupervisor

# Perception owns a copy of the known grid and grows it from surprise returns.
detector = DynamicObstacleMap(grid, clearance=0.30)
supervisor = NavigationSupervisor(grid, goal_cell, detector=detector)
supervisor.plan(pose)                # False if the goal is already unreachable

while not (supervisor.goal_reached or supervisor.failed):
    scan = lidar.scan(world, robot.pose)
    command = supervisor.step(pose, scan, dt=0.05)   # replans when blocked
    robot.step(command.v, command.w, dt=0.05)
    pose = pf.estimate()

print(supervisor.state, supervisor.replan_count, len(supervisor.detected_cells))
# a scan/backup recovery ends in NavState.FAILED with a zero command,
# never in a collision
```

## Evaluation in brief

```python
from atlas.evaluation import aggregate, get_scenario, run_suite

# Six scenarios, three seeds: 18 runs, no shared mutable state between runs.
results = run_suite(seeds=(0, 1, 2))
summary = aggregate(results)

print(summary["tracking_pid"]["cross_track_rms"].mean)
print(summary["localization"]["filter_rms"].as_dict())
# {'count': 3, 'mean': 0.0845, 'std': 0.0004, ...}
```

```bash
python scripts/evaluate.py --scenario planning --seeds 0 1 2
python scripts/evaluate.py --json experiments/results/phase7_metrics.json
```

Full derivations, error analysis and assumptions are in
[docs/mathematics.md](docs/mathematics.md).
