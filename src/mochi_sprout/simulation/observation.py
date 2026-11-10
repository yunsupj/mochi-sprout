"""Simulator-independent angle measurement, noise, and causal history."""

from bisect import bisect_right
from dataclasses import dataclass
from math import isfinite, radians
from random import Random


@dataclass(frozen=True)
class Observation:
    angle_rad: float
    raw_angle_rad: float
    measurement_time_s: float
    age_s: float


class ObservationHistory:
    """Return the newest sample available at decision_time - delay.

    Before the requested history exists, return the initial sample at t=0.
    """

    def __init__(self, delay_s: float, update_dt_s: float):
        if (not isfinite(delay_s) or delay_s < 0 or not isfinite(update_dt_s)
                or update_dt_s <= 0
                or abs(delay_s / update_dt_s - round(delay_s / update_dt_s)) > 1e-9):
            raise ValueError("delay must be a nonnegative update-interval multiple")
        self.delay_s = delay_s
        self.reset()

    def reset(self) -> None:
        self.times: list[float] = []
        self.values: list[float] = []

    def append(self, time_s: float, value: float) -> None:
        if (not isfinite(time_s) or not isfinite(value) or time_s < 0
                or self.times and time_s <= self.times[-1]):
            raise ValueError("samples must be finite and strictly time ordered")
        self.times.append(time_s)
        self.values.append(value)

    def at(self, decision_time_s: float) -> tuple[float, float]:
        if not self.times or not isfinite(decision_time_s):
            raise ValueError("history needs a sample and finite decision time")
        index = bisect_right(self.times, decision_time_s - self.delay_s + 1e-12) - 1
        index = max(0, index)
        if self.times[index] > decision_time_s + 1e-12:
            raise ValueError("future sample cannot be observed")
        return self.times[index], self.values[index]


class ObservationPipeline:
    """Generate one noisy angle sample per controller decision and delay it."""

    def __init__(self, noise_sd_deg: float, delay_s: float, update_dt_s: float, seed: int):
        if not isfinite(noise_sd_deg) or noise_sd_deg < 0:
            raise ValueError("noise standard deviation must be finite and nonnegative")
        self.noise_sd_rad = radians(noise_sd_deg)
        self.seed = seed
        self.history = ObservationHistory(delay_s, update_dt_s)
        self.reset()

    def reset(self) -> None:
        self.rng = Random(self.seed)
        self.history.reset()

    def observe(self, decision_time_s: float, true_angle_rad: float) -> Observation:
        if not isfinite(true_angle_rad):
            raise ValueError("true angle must be finite")
        raw = true_angle_rad + (self.rng.gauss(0.0, self.noise_sd_rad)
                                if self.noise_sd_rad else 0.0)
        self.history.append(decision_time_s, raw)
        measurement_time, angle = self.history.at(decision_time_s)
        return Observation(angle, raw, measurement_time, decision_time_s - measurement_time)
