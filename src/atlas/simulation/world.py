"""2D world with boundaries and rectangular static obstacles."""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class RectObstacle:
    xmin: float
    ymin: float
    xmax: float
    ymax: float

    def collides_with_circle(self, cx: float, cy: float, radius: float) -> bool:
        """Closest point on the rectangle to the circle centre, then compare distance."""
        nx = min(max(cx, self.xmin), self.xmax)
        ny = min(max(cy, self.ymin), self.ymax)
        return (cx - nx) ** 2 + (cy - ny) ** 2 < radius ** 2


@dataclass
class World:
    width: float
    height: float
    obstacles: list[RectObstacle] = field(default_factory=list)

    def add_obstacle(self, obstacle: RectObstacle) -> None:
        self.obstacles.append(obstacle)

    def in_collision(self, x: float, y: float, radius: float) -> bool:
        """True if a circular robot at (x, y) hits a boundary or an obstacle."""
        if x - radius < 0 or x + radius > self.width:
            return True
        if y - radius < 0 or y + radius > self.height:
            return True
        return any(o.collides_with_circle(x, y, radius) for o in self.obstacles)