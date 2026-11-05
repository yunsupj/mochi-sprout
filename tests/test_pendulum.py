import math

import mujoco
import pytest

from mochi_sprout.simulation.pendulum import DURATION_S, make_model, simulate


def test_model_loads() -> None:
    model = make_model()
    assert model.nq == 1
    assert model.nv == 1
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "hinge") == 0


def test_simulation_advances_in_time() -> None:
    rows = simulate()
    assert rows[0]["time_s"] == 0.0
    assert rows[-1]["time_s"] == pytest.approx(DURATION_S, abs=1e-12)
    assert all(b["time_s"] > a["time_s"] for a, b in zip(rows, rows[1:]))


def test_state_stays_finite() -> None:
    for gravity in (-9.81, -3.0):
        assert all(
            math.isfinite(row[key])
            for row in simulate(gravity)
            for key in ("qpos_rad", "qvel_rad_s")
        )


def test_repeated_runs_are_consistent() -> None:
    first = simulate()
    second = simulate()
    for a, b in zip(first, second, strict=True):
        for key in ("time_s", "qpos_rad", "qvel_rad_s"):
            assert a[key] == pytest.approx(b[key], abs=1e-12)


def test_gravity_changes_trajectory() -> None:
    normal = simulate(-9.81)
    reduced = simulate(-3.0)
    assert normal[0] == reduced[0]
    assert max(
        abs(a["qpos_rad"] - b["qpos_rad"])
        for a, b in zip(normal, reduced, strict=True)
    ) > 0.01
