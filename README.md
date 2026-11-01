# Atlas (Project ATLAS)

Autonomous Navigation and Learning System: a simulation-based mobile robot
project in Python (motion model, planning, control, sensing, localization).

## Status

| Phase | Scope | State |
|---|---|---|
| 0 | Geometry utilities, conventions, project scaffold | complete |
| 1 | Differential-drive kinematics and motion simulation | complete |
| 2 | Occupancy grids, Dijkstra, A* path planning | complete |
| 3 | Motion control and trajectory tracking | next |

Phase 2 provides grid environments rasterised from the simulator's world,
conservative obstacle inflation for the robot body, 4/8-connected graph search
with Dijkstra and A*, path reconstruction, and reachability reporting.

## Layout

```text
src/atlas/
├── geometry.py                  # angles, distance, transforms
├── robot/differential_drive.py  # pose, wheel <-> twist, motion integration
├── planning/
│   ├── grid.py                  # occupancy grid, inflation, neighbours
│   └── search.py                # Dijkstra, A*, heuristics, path reconstruction
└── simulation/
    ├── world.py                 # boundaries and rectangular obstacles
    └── simulator.py             # timestep loop, collisions, trajectory log
tests/                           # 73 tests: geometry, motion, collision, planning
scripts/
├── phase1_demo.py               # kinematics and integration-error experiments
└── phase2_demo.py               # Dijkstra vs A* on an inflated grid
docs/
├── design_decisions.md
└── mathematics.md               # derivations for kinematics and search
```

## Setup

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"
pytest
```

## Phase 1 demo

```bash
python scripts/phase1_demo.py
```

Prints a straight-line check, an in-place rotation, and a table of integration
error against the exact solution, then plots the circle and a collision run.

## Phase 2 demo

```bash
python scripts/phase2_demo.py
```

Builds an 80 × 80 occupancy grid from a world with walls, inflates obstacles by
the robot radius, plans with Dijkstra and A*, and reports path cost, expanded
cells and runtime. The comparison plot is written to
`experiments/plots/phase2_planning.png`.

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
from atlas.planning import OccupancyGrid, a_star, dijkstra
from atlas.simulation.world import RectObstacle, World

world = World(20, 20, [RectObstacle(6, 0, 7, 14)])
grid = OccupancyGrid.from_world(world, resolution=0.25, inflate_radius=0.2)

start = grid.world_to_cell(1.0, 1.0)
goal = grid.world_to_cell(18.0, 18.0)

result = a_star(grid, start, goal)   # octile heuristic, 8-connected
if result.found:
    print(result.cost * grid.resolution, "metres", len(result.path), "waypoints")
```

Full derivations, error analysis and assumptions are in
[docs/mathematics.md](docs/mathematics.md).
