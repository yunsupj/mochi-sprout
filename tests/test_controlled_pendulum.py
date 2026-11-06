import math

import mujoco
import pytest

from mochi_sprout.control import PID, PIDConfig
from mochi_sprout.simulation.controlled_pendulum import (
    make_controlled_model, run_episode,
)


def test_actuator_and_physics_configuration() -> None:
    model = make_controlled_model()
    assert (model.nq, model.nv, model.nu) == (1, 1, 1)
    assert model.body_mass[1] == pytest.approx(1.0)
    assert model.dof_damping[0] == pytest.approx(0.01)
    assert model.opt.timestep == pytest.approx(0.002)
    assert model.actuator_gear[0, 0] == pytest.approx(1.0)
    assert model.actuator_ctrlrange[0] == pytest.approx([-3.0, 3.0])
    data = mujoco.MjData(model)
    data.qpos[0] = math.pi / 6
    mujoco.mj_forward(model, data)
    assert data.qfrc_bias[0] == pytest.approx(1.22625)
    assert data.qfrc_bias[0] < model.actuator_ctrlrange[0, 1]


def test_controller_reset_and_update_schedule() -> None:
    controller = PID(PIDConfig(kp=1))
    rows = run_episode(controller, lambda t: 0 if t < 0.5 else math.pi / 6,
                       duration_s=1, controller_dt_s=0.01)
    assert len(rows) == 501
    assert sum(row["controller_updated"] for row in rows) == 100
    assert rows[0]["qpos_rad"] == rows[0]["qvel_rad_s"] == 0
    assert rows[250]["time_s"] == pytest.approx(0.5)
    assert rows[250]["target_rad"] == pytest.approx(math.pi / 6)
    assert rows[250]["torque_nm"] == pytest.approx(math.pi / 6)
    assert run_episode(controller, lambda t: 0 if t < 0.5 else math.pi / 6,
                       duration_s=1, controller_dt_s=0.01) == rows
