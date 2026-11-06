import ast
from pathlib import Path

import pytest

from mochi_sprout.control import PID, PIDConfig
import mochi_sprout.control.pid as pid_module


def test_p_calculation() -> None:
    controller = PID(PIDConfig(kp=2))
    assert controller.update(1.0, 0.2, 0.1) == pytest.approx(1.6)


def test_integral_accumulation_and_reset() -> None:
    controller = PID(PIDConfig(kp=0, ki=2))
    assert controller.update(1, 0, 0.1) == pytest.approx(0.2)
    assert controller.update(1, 0, 0.1) == pytest.approx(0.4)
    controller.reset()
    assert controller.update(1, 0, 0.1) == pytest.approx(0.2)


def test_derivative_on_measurement_sign_and_no_setpoint_kick() -> None:
    controller = PID(PIDConfig(kp=0, kd=1))
    assert controller.update(0, 0, 0.1) == 0
    assert controller.update(1, 0, 0.1) == 0
    assert controller.update(1, 0.1, 0.1) == pytest.approx(-1)


def test_output_saturation() -> None:
    controller = PID(PIDConfig(kp=10, output_min=-2, output_max=2))
    assert controller.update(1, 0, 0.1) == 2
    assert controller.saturated


def test_antiwindup_freezes_integral_while_driving_into_limit() -> None:
    controller = PID(PIDConfig(kp=0, ki=10, output_min=-1, output_max=1))
    for _ in range(100):
        controller.update(10, 0, 0.1)
    assert controller.integral == 0
    assert controller.saturated
    assert controller.update(-0.1, 0, 0.1) == pytest.approx(-0.1)
    assert controller.integral == pytest.approx(-0.01)


def test_controller_has_no_simulator_import() -> None:
    imports = [node for node in ast.walk(ast.parse(Path(pid_module.__file__).read_text()))
               if isinstance(node, (ast.Import, ast.ImportFrom))]
    names = [node.module for node in imports if isinstance(node, ast.ImportFrom)]
    names += [alias.name for node in imports if isinstance(node, ast.Import)
              for alias in node.names]
    assert not any(name.startswith(("mujoco", "mochi_sprout.simulation")) for name in names)


def test_invalid_timestep_rejected() -> None:
    with pytest.raises(ValueError):
        PID(PIDConfig(kp=1)).update(1, 0, 0)
