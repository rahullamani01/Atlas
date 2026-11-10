"""Generic PID controller (Phase 3).

The discrete form is

    integral += error * dt
    derivative = (error - previous_error) / dt
    output = kp * error + ki * integral + kd * derivative

with output clamping and conditional anti-windup: while the output is
saturated, integration is frozen if the error would push it further into
saturation.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


@dataclass
class PID:
    """A PID controller with output and integral saturation.

    Parameters
    ----------
    kp, ki, kd:
        Proportional, integral and derivative gains.
    output_limit:
        Symmetric absolute limit on the returned value, or None for no limit.
    integral_limit:
        Symmetric absolute limit on the integral term, or None for no limit.
    """

    kp: float
    ki: float = 0.0
    kd: float = 0.0
    output_limit: float | None = None
    integral_limit: float | None = None

    _integral: float = field(default=0.0, init=False, repr=False)
    _previous_error: float | None = field(default=None, init=False, repr=False)

    def __post_init__(self) -> None:
        if self.output_limit is not None and self.output_limit <= 0:
            raise ValueError("output_limit must be positive")
        if self.integral_limit is not None and self.integral_limit < 0:
            raise ValueError("integral_limit must be non-negative")

    def update(self, error: float, dt: float) -> float:
        """Advance one step and return the control output."""
        if not math.isfinite(error):
            raise ValueError("error must be finite")
        if dt <= 0:
            raise ValueError("dt must be positive")

        proportional = self.kp * error
        derivative = (
            0.0
            if self._previous_error is None
            else (error - self._previous_error) / dt
        )

        candidate_integral = self._integral + error * dt
        if self.integral_limit is not None:
            candidate_integral = _clamp(candidate_integral, self.integral_limit)

        output = proportional + self.ki * candidate_integral + self.kd * derivative

        saturated = (
            self.output_limit is not None and abs(output) > self.output_limit
        )
        if not (saturated and error * output > 0):
            # Freeze integration only when it drives further into saturation;
            # the saturated command is still returned below.
            self._integral = candidate_integral

        if self.output_limit is not None:
            output = _clamp(output, self.output_limit)

        self._previous_error = error
        return output

    def reset(self) -> None:
        """Clear integral and derivative history."""
        self._integral = 0.0
        self._previous_error = None

    @property
    def integral(self) -> float:
        return self._integral
