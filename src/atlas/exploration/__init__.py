"""Next-best-view exploration on an unknown map (Phase 13)."""
from atlas.exploration.viewpoint import (
    DEFAULT_MIN_GAIN,
    DEFAULT_RADIUS,
    DEFAULT_SPACING,
    Viewpoint,
    candidate_cells,
    select_viewpoint,
    unknown_area,
)

__all__ = [
    "DEFAULT_MIN_GAIN",
    "DEFAULT_RADIUS",
    "DEFAULT_SPACING",
    "Viewpoint",
    "candidate_cells",
    "select_viewpoint",
    "unknown_area",
]
