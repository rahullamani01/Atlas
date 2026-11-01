# Atlas (Project ATLAS)

Autonomous Navigation and Learning System: a simulation-based mobile robot
project in Python (motion model, planning, control, sensing, localization).

## Status

| Phase | Scope | State |
|---|---|---|
| 0 | Geometry utilities, conventions, project scaffold | complete |
| 1 | Differential-drive kinematics and motion simulation | complete |
| 2 | Environment and path planning | next |

Phase 1 provides a differential-drive robot model with forward/inverse
kinematics, three integrators (Euler, midpoint, exact), velocity limits, a 2D
world with static obstacles and collision checks, and a simulator that records
trajectories.

## Layout

```text
src/atlas/
├── geometry.py                  # angles, distance, transforms
├── robot/differential_drive.py  # pose, wheel <-> twist, motion integration
└── simulation/
    ├── world.py                 # boundaries and rectangular obstacles
    └── simulator.py             # timestep loop, collisions, trajectory log
tests/                           # 39 tests covering geometry, motion, collision
scripts/phase1_demo.py           # numeric + plot experiments for Phase 1
docs/
├── design_decisions.md
└── mathematics.md               # derivations for the motion model
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

## Model in brief

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

Full derivations, error analysis and assumptions are in
[docs/mathematics.md](docs/mathematics.md).
