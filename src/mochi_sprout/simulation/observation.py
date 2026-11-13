"""Simulator-independent angle measurement, noise, and causal history."""

from bisect import bisect_right
from dataclasses import dataclass
from math import isfinite, radians
from heapq import heappop, heappush
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


class EMA:
    """Scalar exponential moving average; first accepted value initializes it."""

    def __init__(self, alpha: float):
        if not isfinite(alpha) or not 0 < alpha <= 1:
            raise ValueError("alpha must be in (0, 1]")
        self.alpha = alpha
        self.reset()

    def reset(self) -> None:
        self.value: float | None = None

    def update(self, value: float) -> float:
        if not isfinite(value):
            raise ValueError("measurement must be finite")
        self.value = (value if self.value is None else
                      self.alpha * value + (1 - self.alpha) * self.value)
        return self.value


@dataclass(frozen=True)
class DeliveredObservation(Observation):
    scheduled_delay_s: float
    arrival_time_s: float
    accepted_raw_rad: float
    filtered_angle_rad: float
    accepted_count: int
    dropped_count: int
    out_of_order_count: int
    stale: bool
    used_initial_prior: bool


class DeliveryPipeline:
    """Seeded measurement generation, delivery scheduling, and accepted-sample EMA.

    Measurements are generated at decision ticks. At a tick, all arrived
    samples are processed in (arrival time, generation index) order. Older
    measurement timestamps are dropped; equal timestamps cannot occur here.
    Until the first delivery, the caller's known initial-angle prior is used,
    not a sensor reading. Day 4 uses zero for its zero initial angle.
    """

    def __init__(self, noise_sd_deg: float, delays_s: tuple[float, ...],
                 update_dt_s: float, seed: int, alpha: float = 1.0,
                 initial_angle_rad: float = 0.0):
        if (not isfinite(noise_sd_deg) or noise_sd_deg < 0 or not delays_s
                or not isfinite(initial_angle_rad)
                or not isfinite(update_dt_s) or update_dt_s <= 0
                or any(not isfinite(d) or d < 0 or
                       abs(d / update_dt_s - round(d / update_dt_s)) > 1e-9
                       for d in delays_s)):
            raise ValueError("invalid noise, update interval, or delay schedule")
        self.noise_sd_rad = radians(noise_sd_deg)
        self.delays_s = delays_s
        self.seed = seed
        self.initial_angle_rad = initial_angle_rad
        self.filter = EMA(alpha)
        self.reset()

    def reset(self) -> None:
        self.noise_rng = Random(self.seed)
        self.delay_rng = Random(self.seed + 1_000_003)
        self.filter.reset()
        self.pending: list[tuple[float, int, float, float]] = []
        self.sequence = 0
        self.latest_time_s = 0.0
        self.latest_arrival_s = 0.0
        self.latest_raw_rad = self.initial_angle_rad
        self.has_measurement = False
        self.last_decision_s = -1.0

    def observe(self, decision_time_s: float, true_angle_rad: float) -> DeliveredObservation:
        if (not isfinite(decision_time_s) or not isfinite(true_angle_rad)
                or decision_time_s < 0 or decision_time_s <= self.last_decision_s):
            raise ValueError("decision times must increase and state must be finite")
        self.last_decision_s = decision_time_s
        raw = true_angle_rad + (self.noise_rng.gauss(0, self.noise_sd_rad)
                                if self.noise_sd_rad else 0.0)
        delay = self.delay_rng.choice(self.delays_s)
        arrival = decision_time_s + delay
        heappush(self.pending, (arrival, self.sequence, decision_time_s, raw))
        self.sequence += 1
        accepted = dropped = out_of_order = 0
        while self.pending and self.pending[0][0] <= decision_time_s + 1e-12:
            arrival_time, _, measurement_time, value = heappop(self.pending)
            if self.has_measurement and measurement_time <= self.latest_time_s:
                dropped += 1
                out_of_order += 1
                continue
            self.latest_time_s = measurement_time
            self.latest_arrival_s = arrival_time
            self.latest_raw_rad = value
            self.has_measurement = True
            self.filter.update(value)
            accepted += 1
        filtered = (self.filter.value if self.filter.value is not None
                    else self.initial_angle_rad)
        return DeliveredObservation(
            angle_rad=filtered, raw_angle_rad=raw,
            measurement_time_s=self.latest_time_s,
            age_s=decision_time_s - self.latest_time_s,
            scheduled_delay_s=delay,
            arrival_time_s=self.latest_arrival_s,
            accepted_raw_rad=self.latest_raw_rad,
            filtered_angle_rad=filtered,
            accepted_count=accepted, dropped_count=dropped,
            out_of_order_count=out_of_order, stale=accepted == 0,
            used_initial_prior=not self.has_measurement,
        )
