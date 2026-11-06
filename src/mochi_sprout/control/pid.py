"""Scalar PID with derivative on measurement and conditional anti-windup."""

from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True)
class PIDConfig:
    """Gains in output/error, output/(error·s), output·s/error."""

    kp: float
    ki: float = 0.0
    kd: float = 0.0
    output_min: float = -float("inf")
    output_max: float = float("inf")

    def __post_init__(self) -> None:
        if not all(isfinite(v) and v >= 0 for v in (self.kp, self.ki, self.kd)):
            raise ValueError("PID gains must be finite and nonnegative")
        if self.output_min >= self.output_max:
            raise ValueError("output_min must be less than output_max")


class PID:
    """Simulator-independent PID for a scalar target and measurement.

    A call to update advances controller state by dt seconds. Output and error
    units are supplied by the caller; for Day 2 they are N·m and radians.
    """

    def __init__(self, config: PIDConfig):
        self.config = config
        self.reset()

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_measurement: float | None = None
        self.saturated = False

    def update(self, target: float, measurement: float, dt: float) -> float:
        if not (isfinite(target) and isfinite(measurement) and isfinite(dt) and dt > 0):
            raise ValueError("target, measurement, and positive dt must be finite")
        error = target - measurement
        derivative = (
            0.0 if self.previous_measurement is None
            else -(measurement - self.previous_measurement) / dt
        )
        self.previous_measurement = measurement
        candidate_integral = self.integral + error * dt
        raw = (self.config.kp * error
               + self.config.ki * candidate_integral
               + self.config.kd * derivative)
        # Do not integrate if the error would drive an already-clipped output
        # farther into saturation. Keep integrating when the error unwinds it.
        if not ((raw > self.config.output_max and error > 0)
                or (raw < self.config.output_min and error < 0)):
            self.integral = candidate_integral
        self.saturated = raw > self.config.output_max or raw < self.config.output_min
        return min(self.config.output_max, max(self.config.output_min, raw))
