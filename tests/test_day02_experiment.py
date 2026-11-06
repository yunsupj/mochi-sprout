import math

import pytest

from mochi_sprout.experiments.day02 import (
    ExperimentConfig, controller_configs, run_comparison,
)


def test_configuration_and_deterministic_reproduction() -> None:
    config = ExperimentConfig()
    assert config.target_after_step_rad == pytest.approx(math.pi / 6)
    gains = controller_configs(3)
    assert set(gains) == {"P", "PI", "PD", "PID"}
    assert {c.kp for c in gains.values()} == {6}
    first, first_rows = run_comparison(config)
    second, second_rows = run_comparison(config)
    assert first_rows == second_rows
    assert first["metrics"] == second["metrics"]
    assert all(len(rows) == 5001 for rows in first_rows.values())
    assert first["compiled_model_checks"]["gravity_hold_torque_nm_at_target"] < 3
