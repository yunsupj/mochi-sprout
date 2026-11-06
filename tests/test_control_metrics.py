import math

import pytest

from mochi_sprout.evaluation import evaluate_step_response


def test_step_response_metrics() -> None:
    rows = []
    for t, target, angle, torque, saturated in [
        (0, 0, 0, 0, False),
        (0.5, 1, 0, 2, True),
        (1, 1, 1.2, 1, False),
        (1.5, 1, 1, 1, False),
        (2, 1, 1, 1, False),
    ]:
        rows.append({"time_s": t, "target_rad": target, "qpos_rad": angle,
                     "qvel_rad_s": 0, "error_rad": target - angle,
                     "torque_nm": torque, "controller_updated": t < 2,
                     "saturated": saturated})
    metrics = evaluate_step_response(rows, step_time_s=0.5, target_rad=1,
                                     steady_window_s=0.5, settling_band_rad=0.05)
    assert metrics["steady_state_mae_deg"] == 0
    assert metrics["overshoot_deg"] == pytest.approx(math.degrees(0.2))
    assert metrics["settling_time_s"] == pytest.approx(1.0)
    assert metrics["rmse_after_step_deg"] == pytest.approx(math.degrees(math.sqrt(1.04 / 4)))
    assert metrics["max_abs_torque_nm"] == 2
    assert metrics["rms_torque_nm"] == pytest.approx(math.sqrt(1.5))
    assert metrics["saturation_fraction"] == pytest.approx(0.25)
    assert metrics["status"] == "finite_settled"
