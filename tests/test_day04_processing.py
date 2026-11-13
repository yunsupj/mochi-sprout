import math

import pytest

from mochi_sprout.experiments.day02 import ExperimentConfig
from mochi_sprout.experiments.day03 import Scenario as Day3Scenario, run_scenario as day3_run
from mochi_sprout.experiments.day04 import Scenario, run_scenario
from mochi_sprout.simulation.observation import DeliveryPipeline, EMA


class ScheduledDelays:
    def __init__(self, values):
        self.values = iter(values)

    def choice(self, _):
        return next(self.values)


def test_ema_arithmetic_validation_and_reset() -> None:
    ema = EMA(0.2)
    assert ema.value is None
    assert ema.update(10) == 10
    assert ema.update(20) == pytest.approx(12)
    ema.reset()
    assert ema.update(-2) == -2
    assert EMA(1).update(3) == 3
    for alpha in (0, -0.1, 1.1, float("nan")):
        with pytest.raises(ValueError):
            EMA(alpha)


def test_no_filter_clean_baseline_equals_day3() -> None:
    config = ExperimentConfig(duration_s=2, steady_window_s=1)
    _, old = day3_run(Day3Scenario("baseline"), 11, config)
    _, new = run_scenario(Scenario("baseline"), 11, config)
    assert all(all(new_row[k] == value for k, value in old_row.items())
               for old_row, new_row in zip(old, new, strict=True))


def test_noise_seed_reproducibility_and_filter_reset() -> None:
    pipeline = DeliveryPipeline(0.4, (0,), 0.01, 11, alpha=0.2)
    first = [pipeline.observe(i * .01, .5) for i in range(20)]
    pipeline.reset()
    assert first == [pipeline.observe(i * .01, .5) for i in range(20)]
    other = DeliveryPipeline(0.4, (0,), .01, 23, alpha=.2)
    assert first != [other.observe(i * .01, .5) for i in range(20)]
    assert first[0].filtered_angle_rad == first[0].raw_angle_rad
    assert all(row.age_s == 0 for row in first)


def test_filter_does_not_update_on_stale_decision() -> None:
    pipeline = DeliveryPipeline(0, (0, .05), .01, 11, alpha=.2)
    pipeline.delay_rng = ScheduledDelays((0, .05))
    first = pipeline.observe(0, 1)
    stale = pipeline.observe(.01, 10)
    assert first.accepted_count == 1
    assert stale.accepted_count == 0 and stale.stale
    assert stale.raw_angle_rad == 10
    assert stale.filtered_angle_rad == first.filtered_angle_rad == 1


def test_delivery_timestamps_initialization_and_no_future_visibility() -> None:
    pipeline = DeliveryPipeline(0, (.05,), .01, 11)
    first = pipeline.observe(0, 1)
    assert first.used_initial_prior and first.stale
    assert first.angle_rad == first.measurement_time_s == first.arrival_time_s == 0
    for i in range(1, 5):
        row = pipeline.observe(i * .01, 2)
        assert row.used_initial_prior and row.accepted_count == 0
    delivered = pipeline.observe(.05, 3)
    assert delivered.measurement_time_s == 0
    assert delivered.arrival_time_s == pytest.approx(.05)
    assert delivered.age_s == pytest.approx(.05)
    assert delivered.angle_rad == 1
    assert not delivered.used_initial_prior
    with pytest.raises(ValueError):
        pipeline.observe(.05, 4)
    custom_prior = DeliveryPipeline(0, (.05,), .01, 11, initial_angle_rad=.7)
    assert custom_prior.observe(0, 1).angle_rad == .7


def test_multiple_arrivals_tie_breaking_and_out_of_order_drop() -> None:
    pipeline = DeliveryPipeline(0, (.01, .03, .05), .01, 11, alpha=.5)
    pipeline.delay_rng = ScheduledDelays((.05, .04, .03, .02, .01, .01))
    for i in range(5):
        pipeline.observe(i * .01, float(i))
    at_five = pipeline.observe(.05, 5)
    # At .05, samples t=0 through .04 arrive together in generation order.
    assert at_five.accepted_count == 5
    assert at_five.measurement_time_s == pytest.approx(.04)
    assert at_five.angle_rad == pytest.approx(3.0625)
    assert at_five.dropped_count == 0
    pipeline = DeliveryPipeline(0, (.01, .09), .01, 11)
    pipeline.delay_rng = ScheduledDelays((.09, .01) + (.01,) * 9)
    for i in range(10):
        result = pipeline.observe(i * .01, float(i))
    assert result.out_of_order_count == result.dropped_count == 1
    assert result.measurement_time_s == pytest.approx(.08)
    assert result.arrival_time_s <= .09 + 1e-12


def test_jitter_seed_and_physical_error_separation() -> None:
    config = ExperimentConfig(duration_s=2, steady_window_s=1)
    scenario = Scenario("jitter", noise_sd_deg=.4, delays_ms=(10, 30, 50, 70, 90), alpha=.2)
    first, rows = run_scenario(scenario, 11, config)
    again, repeated = run_scenario(scenario, 11, config)
    different, other = run_scenario(scenario, 23, config)
    assert rows == repeated and first == again
    assert rows != other and first != different
    updates = [r for r in rows if r["controller_updated"]]
    assert all(r["measurement_time_s"] <= r["arrival_time_s"] + 1e-12
               and r["arrival_time_s"] <= r["time_s"] + 1e-12 for r in updates)
    assert all(r["error_rad"] == r["target_rad"] - r["qpos_rad"] for r in rows)
    assert first["dropped_count"] > 0
    assert first["filtered_error_rms_deg"] > 0


def test_delay_and_alpha_validation() -> None:
    for delays in ((), (-.01,), (.015,), (float("nan"),)):
        with pytest.raises(ValueError):
            DeliveryPipeline(0, delays, .01, 11)
