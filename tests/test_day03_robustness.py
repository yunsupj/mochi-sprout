import math

import mujoco
import pytest

from mochi_sprout.control import PID
from mochi_sprout.evaluation.robustness_metrics import aggregate_metrics, evaluate_robustness
from mochi_sprout.experiments.day02 import ExperimentConfig, controller_configs, target_at_time
from mochi_sprout.experiments.day03 import Scenario, run_scenario
from mochi_sprout.simulation.controlled_pendulum import (
    PendulumModelConfig, make_controlled_model, run_episode,
)
from mochi_sprout.simulation.observation import ObservationPipeline


def test_zero_delay_zero_noise_reproduces_day2_pid() -> None:
    config = ExperimentConfig()
    controller = PID(controller_configs(3)["PID"])
    direct = run_episode(controller, lambda t: target_at_time(t, config),
                         duration_s=10, controller_dt_s=0.01)
    via_observation = run_episode(
        controller, lambda t: target_at_time(t, config),
        duration_s=10, controller_dt_s=0.01,
        observation=ObservationPipeline(0, 0, 0.01, 11),
    )
    for a, b in zip(direct, via_observation, strict=True):
        assert all(b[key] == value for key, value in a.items())


def test_inertia_only_scaling_and_gravity_holding_torque() -> None:
    reference = make_controlled_model()
    initial = mujoco.MjData(reference)
    mujoco.mj_forward(reference, initial)
    inertia = float(initial.M[0])
    for factor in (1, 1.5, 2):
        model = make_controlled_model(PendulumModelConfig(inertia_factor=factor))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        assert data.M[0] == pytest.approx(inertia * factor)
        assert model.body_mass[1] == reference.body_mass[1]
        assert model.body_ipos[1].tolist() == reference.body_ipos[1].tolist()
        assert model.dof_damping[0] == reference.dof_damping[0]
        assert model.actuator_ctrlrange[0].tolist() == reference.actuator_ctrlrange[0].tolist()
        assert model.body_inertia[1, 0] == pytest.approx(model.body_inertia[1, 1])
        assert model.body_inertia[1, 2] > 0
        data.qpos[0] = math.pi / 6
        mujoco.mj_forward(model, data)
        assert data.qfrc_bias[0] == pytest.approx(1.22625)
    with pytest.raises(ValueError):
        make_controlled_model(PendulumModelConfig(inertia_factor=0.5))


def test_sensor_observation_separate_from_ground_truth_and_seed_reproducibility() -> None:
    scenario = Scenario("noise", noise_sd_deg=0.4)
    short = ExperimentConfig(duration_s=1, steady_window_s=0.5)
    first, rows = run_scenario(scenario, 11, short)
    again, repeated = run_scenario(scenario, 11, short)
    different, other = run_scenario(scenario, 23, short)
    assert rows == repeated
    assert first == again
    assert rows != other
    assert first["metrics"] != different["metrics"]
    assert any(row["raw_sensor_rad"] != row["qpos_rad"]
               for row in rows if row["controller_updated"])
    assert all(row["error_rad"] == row["target_rad"] - row["qpos_rad"] for row in rows)
    assert rows[0]["qpos_rad"] == 0


def test_delayed_observation_changes_physical_trajectory() -> None:
    config = ExperimentConfig(duration_s=2, steady_window_s=1)
    _, direct = run_scenario(Scenario("baseline"), 11, config)
    _, delayed = run_scenario(Scenario("delay", delay_ms=50), 11, config)
    assert any(abs(a["qpos_rad"] - b["qpos_rad"]) > 1e-6
               for a, b in zip(direct, delayed, strict=True))
    update_rows = [row for row in delayed if row["controller_updated"]]
    assert update_rows[0]["measurement_age_s"] == 0
    assert update_rows[1]["measurement_age_s"] == pytest.approx(0.01)
    assert update_rows[10]["measurement_age_s"] == pytest.approx(0.05)
    assert all(row["measurement_time_s"] <= row["time_s"] for row in update_rows)


def test_robustness_metrics_use_true_state_and_unsettled_is_explicit() -> None:
    rows = []
    for t in (0, 0.5, 1.0):
        rows.append({"time_s": t, "target_rad": 1 if t else 0,
                     "qpos_rad": 0, "qvel_rad_s": 0,
                     "error_rad": 1 if t else 0, "torque_nm": 0,
                     "controller_updated": t < 1, "saturated": False,
                     "raw_sensor_rad": 1, "observed_rad": 1,
                     "measurement_time_s": t, "measurement_age_s": 0})
    metrics = evaluate_robustness(rows, step_time_s=0.5, target_rad=1,
                                  steady_window_s=0.5, settling_band_rad=0.01)
    assert metrics["steady_state_mae_deg"] == pytest.approx(math.degrees(1))
    assert metrics["settling_time_s"] is None
    # A corrupted observation-based error field cannot change physical metrics.
    for row in rows:
        row["error_rad"] = 0
    assert evaluate_robustness(rows, step_time_s=0.5, target_rad=1,
                               steady_window_s=0.5, settling_band_rad=0.01) == metrics
    aggregate = aggregate_metrics({11: metrics, 23: metrics})
    assert aggregate["unsettled_count"] == 2
    assert aggregate["numeric"]["settling_time_s"] is None
    assert aggregate["numeric"]["steady_state_mae_deg"]["population_sd"] == 0


def test_aggregation_reports_seed_variability_and_partial_settling() -> None:
    aggregate = aggregate_metrics({
        11: {"steady_state_mae_deg": 1.0, "settling_time_s": None,
             "status": "finite_unsettled"},
        23: {"steady_state_mae_deg": 3.0, "settling_time_s": 2.0,
             "status": "finite_settled"},
        37: {"steady_state_mae_deg": 5.0, "settling_time_s": None,
             "status": "failed_nonfinite_or_over_360deg"},
    })
    assert aggregate["seed_count"] == 3
    assert aggregate["unsettled_count"] == 2
    assert aggregate["status_counts"]["failed_nonfinite_or_over_360deg"] == 1
    assert aggregate["numeric"]["steady_state_mae_deg"] == {
        "mean": 3.0, "population_sd": pytest.approx(math.sqrt(8 / 3)),
        "min": 1.0, "max": 5.0, "observed_count": 3,
    }
    assert aggregate["numeric"]["settling_time_s"]["observed_count"] == 1


def test_torque_change_uses_adjacent_controller_updates_only() -> None:
    rows = []
    for time, torque, update in [(0, 0, True), (0.005, 0, False),
                                 (0.01, 3, True), (0.015, 3, False),
                                 (0.02, 7, True), (0.025, 7, False)]:
        rows.append({"time_s": time, "target_rad": 1, "qpos_rad": 0,
                     "qvel_rad_s": 0, "error_rad": 1, "torque_nm": torque,
                     "controller_updated": update, "saturated": False,
                     "raw_sensor_rad": 0, "observed_rad": 0,
                     "measurement_time_s": time, "measurement_age_s": 0})
    metrics = evaluate_robustness(rows, step_time_s=0.01, target_rad=1,
                                  steady_window_s=0.005, settling_band_rad=0.01)
    assert metrics["rms_torque_change_nm_per_update"] == pytest.approx(math.sqrt(12.5))
