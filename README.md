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
| 4 | LiDAR simulation and sensor uncertainty | next |

Phase 3 adds a generic PID controller with saturation and anti-windup, and a
carrot-chasing path follower that turns a planned grid path into body velocity
commands. The robot drives the Phase 2 map end to end, with tracking metrics
recorded in the demo.

## Layout

```text
src/atlas/
├── geometry.py                  # angles, distance, transforms
├── robot/differential_drive.py  # pose, wheel <-> twist, motion integration
├── planning/
│   ├── grid.py                  # occupancy grid, inflation, neighbours
│   └── search.py                # Dijkstra, A*, heuristics, path reconstruction
├── control/
│   ├── pid.py                   # PID with clamps and anti-windup
│   └── path_follower.py         # carrot chasing with PID heading control
└── simulation/
    ├── world.py                 # boundaries and rectangular obstacles
    └── simulator.py             # timestep loop, collisions, trajectory log
tests/                           # 105 tests across geometry, motion, planning, control
scripts/
├── phase1_demo.py               # kinematics and integration-error experiments
├── phase2_demo.py               # Dijkstra vs A* on an inflated grid
└── phase3_demo.py               # P-only vs PID closed-loop tracking
docs/
├── design_decisions.md
├── mathematics.md               # derivations for kinematics, search and control
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

Full derivations, error analysis and assumptions are in
[docs/mathematics.md](docs/mathematics.md).
