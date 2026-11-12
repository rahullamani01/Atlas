"""Named evaluation scenarios shared by tests, the benchmark and future work (Phase 7).

A scenario is a description, not a mutable run. `world` is a factory so every
run gets a fresh world and an obstacle injection can never leak between seeds.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from atlas.simulation.world import RectObstacle, World

KINDS = (
    "kinematics",
    "planning",
    "tracking",
    "localization",
    "recovery",
    "integrated",
)
CONTROLLERS = ("p", "pid")


def demo_world() -> World:
    """The 20 x 20 map used by the Phase 2-6 demos."""
    return World(20, 20, [
        RectObstacle(6, 0, 7, 14),     # vertical wall with a gap at the top
        RectObstacle(10, 9, 17, 10),   # horizontal wall
        RectObstacle(13, 2, 15, 4),    # free-standing block
    ])


@dataclass(frozen=True)
class Scenario:
    """One reproducible experiment configuration.

    ``kind`` selects the runner: kinematics, planning, tracking, localization or
    recovery. ``controller`` and ``yaw_bias`` only affect tracking runs.
    ``inject_step`` / ``inject_rect`` add a new obstacle after the given control
    step, which is how the recovery scenario changes the world mid-run.
    """

    name: str
    kind: str
    world: Callable[[], World] = demo_world
    start: tuple[float, float] = (1.125, 1.125)
    goal: tuple[float, float] = (18.125, 18.125)
    resolution: float = 0.25
    inflation: float = 0.25
    dt: float = 0.05
    max_steps: int = 6000
    controller: str = "pid"
    yaw_bias: float = 0.0
    inject_step: int | None = None
    inject_rect: tuple[float, float, float, float] | None = None

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("name must be non-empty")
        if self.kind not in KINDS:
            raise ValueError(f"kind must be one of {KINDS}")
        if self.resolution <= 0:
            raise ValueError("resolution must be positive")
        if self.inflation < 0:
            raise ValueError("inflation must be non-negative")
        if self.dt <= 0:
            raise ValueError("dt must be positive")
        if self.max_steps < 1:
            raise ValueError("max_steps must be positive")
        if self.controller not in CONTROLLERS:
            raise ValueError(f"controller must be one of {CONTROLLERS}")
        if (self.inject_step is None) != (self.inject_rect is None):
            raise ValueError(
                "inject_step and inject_rect must be provided together"
            )
        if self.inject_step is not None:
            if self.inject_step < 0:
                raise ValueError("inject_step must be non-negative")
            xmin, ymin, xmax, ymax = self.inject_rect
            if xmin >= xmax or ymin >= ymax:
                raise ValueError("inject_rect must have positive width and height")


SCENARIOS: dict[str, Scenario] = {
    "kinematics": Scenario(
        name="kinematics",
        kind="kinematics",
        max_steps=126,
    ),
    "planning": Scenario(
        name="planning",
        kind="planning",
        inflation=0.20,
        max_steps=1,
    ),
    "tracking_pid": Scenario(
        name="tracking_pid",
        kind="tracking",
        controller="pid",
        yaw_bias=0.2,
    ),
    "tracking_p": Scenario(
        name="tracking_p",
        kind="tracking",
        controller="p",
        yaw_bias=0.2,
    ),
    "localization": Scenario(
        name="localization",
        kind="localization",
    ),
    "recovery": Scenario(
        name="recovery",
        kind="recovery",
        inflation=0.30,
        inject_step=80,
        inject_rect=(2.32, 5.7, 2.92, 6.3),
    ),
    "integrated": Scenario(
        name="integrated",
        kind="integrated",
        inflation=0.30,
        inject_step=80,
        inject_rect=(2.32, 5.7, 2.92, 6.3),
    ),
}


def get_scenario(name: str) -> Scenario:
    """Return a named scenario or raise a clear ``KeyError``."""
    try:
        return SCENARIOS[name]
    except KeyError as error:
        raise KeyError(f"unknown scenario: {name!r}") from error


def default_scenarios() -> list[Scenario]:
    """The full suite, in the order the benchmark prints it."""
    return list(SCENARIOS.values())
