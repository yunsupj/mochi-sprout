"""MuJoCo integration for the Day 1 pendulum with a torque motor."""

from collections.abc import Callable
from dataclasses import dataclass
from math import isfinite
from typing import Protocol

import mujoco

from .pendulum import MJCF
from .observation import ObservationPipeline


@dataclass(frozen=True)
class PendulumModelConfig:
    """Physical settings; controller gains live in a separate configuration."""

    torque_limit_nm: float = 3.0
    gravity_z_m_s2: float = -9.81
    inertia_factor: float = 1.0


class ScalarController(Protocol):
    saturated: bool

    def reset(self) -> None: ...
    def update(self, target: float, measurement: float, dt: float) -> float: ...


def make_controlled_model(config: PendulumModelConfig = PendulumModelConfig()) -> mujoco.MjModel:
    if (not isfinite(config.torque_limit_nm) or config.torque_limit_nm <= 0
            or not isfinite(config.gravity_z_m_s2)
            or not isfinite(config.inertia_factor) or config.inertia_factor < 1):
        raise ValueError("invalid physical model configuration")
    actuator = (
        '<actuator><motor name="hinge_torque" joint="hinge" gear="1" '
        f'ctrllimited="true" ctrlrange="{-config.torque_limit_nm} {config.torque_limit_nm}"/>'
        '</actuator></mujoco>'
    )
    model = mujoco.MjModel.from_xml_string(MJCF.replace("</mujoco>", actuator))
    model.opt.gravity[2] = config.gravity_z_m_s2
    if config.inertia_factor != 1:
        # Preserve mass and COM. Add equal transverse central inertia to keep
        # the diagonal tensor positive and satisfy its triangle inequalities.
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        delta = (config.inertia_factor - 1) * float(data.M[0])
        model.body_inertia[1, 0:2] += delta
        mujoco.mj_setConst(model, data)
    return model


def run_episode(
    controller: ScalarController | None,
    target_at_time: Callable[[float], float],
    *,
    duration_s: float,
    controller_dt_s: float,
    model_config: PendulumModelConfig = PendulumModelConfig(),
    observation: ObservationPipeline | None = None,
) -> list[dict[str, float | bool]]:
    """Sample every physics step; hold each torque for one controller interval."""
    model = make_controlled_model(model_config)
    physics_dt = float(model.opt.timestep)
    steps = round(duration_s / physics_dt)
    update_steps = round(controller_dt_s / physics_dt)
    if (steps <= 0 or update_steps <= 0
            or abs(steps * physics_dt - duration_s) > 1e-10
            or abs(update_steps * physics_dt - controller_dt_s) > 1e-10):
        raise ValueError("duration and controller interval must be physics-step multiples")

    data = mujoco.MjData(model)  # qpos=qvel=0 is the requested initial state.
    mujoco.mj_forward(model, data)
    if controller is not None:
        controller.reset()
    if observation is not None:
        observation.reset()
    rows: list[dict[str, float | bool]] = []
    last_observation: dict = {}
    for step in range(steps):
        target = target_at_time(step * physics_dt)
        updated = step % update_steps == 0
        if updated:
            if observation is not None:
                sample = observation.observe(step * physics_dt, float(data.qpos[0]))
                last_observation = {
                    "raw_sensor_rad": sample.raw_angle_rad,
                    "observed_rad": sample.angle_rad,
                    "measurement_time_s": sample.measurement_time_s,
                    "measurement_age_s": sample.age_s,
                }
            data.ctrl[0] = (
                0.0 if controller is None
                else controller.update(
                    target,
                    float(data.qpos[0]) if observation is None else sample.angle_rad,
                    controller_dt_s,
                )
            )
        row = {
            "time_s": float(data.time),
            "target_rad": target,
            "qpos_rad": float(data.qpos[0]),
            "qvel_rad_s": float(data.qvel[0]),
            "error_rad": target - float(data.qpos[0]),
            "torque_nm": float(data.ctrl[0]),
            "controller_updated": updated,
            "saturated": bool(controller.saturated) if controller is not None else False,
        }
        if observation is not None:
            row.update(last_observation)
            if not updated:
                row["raw_sensor_rad"] = None
        rows.append(row)
        mujoco.mj_step(model, data)
    target = target_at_time(duration_s)
    row = {
        "time_s": float(data.time),
        "target_rad": target,
        "qpos_rad": float(data.qpos[0]),
        "qvel_rad_s": float(data.qvel[0]),
        "error_rad": target - float(data.qpos[0]),
        "torque_nm": float(data.ctrl[0]),
        "controller_updated": False,
        "saturated": bool(controller.saturated) if controller is not None else False,
    }
    if observation is not None:
        row.update(last_observation)
        row["raw_sensor_rad"] = None
    rows.append(row)
    return rows
