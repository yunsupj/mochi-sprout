import math

import pytest

from mochi_sprout.simulation.observation import ObservationHistory, ObservationPipeline


def test_zero_noise_identity_and_units() -> None:
    identity = ObservationPipeline(0, 0, 0.01, 11)
    assert identity.observe(0, 0.5).angle_rad == 0.5
    noisy = ObservationPipeline(0.4, 0, 0.01, 11)
    assert noisy.noise_sd_rad == pytest.approx(math.radians(0.4))
    sample = noisy.observe(0, 0.5)
    assert sample.angle_rad == sample.raw_angle_rad
    assert sample.measurement_time_s == sample.age_s == 0


def test_seeded_noise_reproducibility_and_reset() -> None:
    pipeline = ObservationPipeline(0.4, 0, 0.01, 11)
    first = [pipeline.observe(i * 0.01, 0.5).angle_rad for i in range(20)]
    pipeline.reset()
    second = [pipeline.observe(i * 0.01, 0.5).angle_rad for i in range(20)]
    different = ObservationPipeline(0.4, 0, 0.01, 23)
    third = [different.observe(i * 0.01, 0.5).angle_rad for i in range(20)]
    assert first == second
    assert first != third
    assert any(value != 0.5 for value in first)


def test_fixed_delay_timing_initialization_and_no_future_leakage() -> None:
    history = ObservationHistory(0.03, 0.01)
    history.append(0, 0)
    for index in range(1, 6):
        history.append(index * 0.01, float(index))
        time, value = history.at(index * 0.01)
        assert time == pytest.approx(max(0, index - 3) * 0.01)
        assert value == max(0, index - 3)
        assert time <= index * 0.01
    assert history.at(0.025) == (0, 0)
    history.reset()
    with pytest.raises(ValueError):
        history.at(0)
    with pytest.raises(ValueError):
        ObservationHistory(0.015, 0.01)
    with pytest.raises(ValueError):
        history.append(-0.01, 1)


def test_zero_delay_uses_current_sample() -> None:
    pipeline = ObservationPipeline(0, 0, 0.01, 11)
    for index in range(10):
        sample = pipeline.observe(index * 0.01, index * 0.1)
        assert sample.angle_rad == pytest.approx(index * 0.1)
        assert sample.age_s == 0
