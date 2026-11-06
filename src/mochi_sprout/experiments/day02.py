"""Compare open-loop baseline and P/PI/PD/PID on a torque pendulum."""

import argparse
import csv
import json
import math
import platform
from dataclasses import asdict, dataclass
from datetime import datetime
from importlib.metadata import version
from pathlib import Path
from zoneinfo import ZoneInfo

import mujoco

from mochi_sprout.control import PID, PIDConfig
from mochi_sprout.evaluation import evaluate_step_response
from mochi_sprout.simulation.controlled_pendulum import (
    PendulumModelConfig, make_controlled_model, run_episode,
)


DAY1_REVISION = "b387c803a7ddf6192ace782515fff94377a7ff95"


@dataclass(frozen=True)
class ExperimentConfig:
    duration_s: float = 10.0
    controller_dt_s: float = 0.01
    step_time_s: float = 0.5
    target_after_step_rad: float = math.pi / 6
    settling_band_rad: float = math.pi / 90  # ±2 degrees
    steady_window_s: float = 2.0


def target_at_time(time_s: float, config: ExperimentConfig) -> float:
    return 0.0 if time_s < config.step_time_s else config.target_after_step_rad


def controller_configs(limit: float) -> dict[str, PIDConfig]:
    """Common Kp=6; add Ki=3 or Kd=1.0 for PI/PD/PID."""
    shared = {"kp": 6.0, "output_min": -limit, "output_max": limit}
    return {
        "P": PIDConfig(**shared),
        "PI": PIDConfig(**shared, ki=3.0),
        "PD": PIDConfig(**shared, kd=1.0),
        "PID": PIDConfig(**shared, ki=3.0, kd=1.0),
    }


def model_check(model_config: PendulumModelConfig, target_rad: float) -> dict:
    model = make_controlled_model(model_config)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    inertia = float(data.M[0])
    data.qpos[0] = target_rad
    mujoco.mj_forward(model, data)
    hold_torque = float(data.qfrc_bias[0])
    data.ctrl[0] = model_config.torque_limit_nm
    mujoco.mj_forward(model, data)
    result = {
        "mass_kg": float(model.body_mass[1]),
        "rod_length_m": 0.5,
        "body_com_from_hinge_m": model.body_ipos[1].tolist(),
        "effective_hinge_inertia_kg_m2": inertia,
        "joint_damping_nm_s_per_rad": float(model.dof_damping[0]),
        "actuator_gear": float(model.actuator_gear[0, 0]),
        "actuator_ctrlrange_nm": model.actuator_ctrlrange[0].tolist(),
        "gravity_z_m_s2": float(model.opt.gravity[2]),
        "physics_timestep_s": float(model.opt.timestep),
        "integrator": "RK4",
        "positive_angle_com_x_m_at_target": float(data.xipos[1, 0]),
        "gravity_hold_torque_nm_at_target": hold_torque,
        "positive_max_ctrl_joint_torque_nm": float(data.qfrc_actuator[0]),
    }
    if not (result["mass_kg"] == 1.0 and result["actuator_gear"] == 1.0
            and result["gravity_hold_torque_nm_at_target"] > 0
            and result["positive_max_ctrl_joint_torque_nm"] > hold_torque):
        raise ValueError("pendulum or actuator assumptions did not hold")
    return result


def run_comparison(
    config: ExperimentConfig = ExperimentConfig(),
    model_config: PendulumModelConfig = PendulumModelConfig(),
) -> tuple[dict, dict[str, list[dict]]]:
    physical = model_check(model_config, config.target_after_step_rad)
    gains = controller_configs(model_config.torque_limit_nm)
    trajectories = {}
    metrics = {}
    for name in ("baseline", "P", "PI", "PD", "PID"):
        controller = None if name == "baseline" else PID(gains[name])
        rows = run_episode(
            controller, lambda t: target_at_time(t, config),
            duration_s=config.duration_s, controller_dt_s=config.controller_dt_s,
            model_config=model_config,
        )
        trajectories[name] = rows
        metrics[name] = evaluate_step_response(
            rows, step_time_s=config.step_time_s,
            target_rad=config.target_after_step_rad,
            steady_window_s=config.steady_window_s,
            settling_band_rad=config.settling_band_rad,
        )
    summary = {
        "experiment_id": "day02-pid-step-comparison",
        "recorded_at": datetime.now(ZoneInfo("America/Los_Angeles")).isoformat(),
        "source_base_revision": DAY1_REVISION,
        "python_version": platform.python_version(),
        "mujoco_version": mujoco.__version__,
        "dependency_versions": {
            "numpy": version("numpy"),
            "matplotlib": version("matplotlib"),
            "pytest": version("pytest"),
        },
        "operating_system": platform.platform(),
        "experiment_config": asdict(config),
        "model_config": asdict(model_config),
        "compiled_model_checks": physical,
        "controller_configs": {name: asdict(c) for name, c in gains.items()},
        "gain_selection": (
            "Choose a natural frequency near 8 rad/s: I*8²=5.53 N·m/rad, "
            "rounded to shared Kp=6. Choose damping ratio near 0.7: "
            "2*0.7*sqrt(Kp*I)=1.01 N·m·s/rad, rounded to Kd=1. "
            "Choose integral time Kp/Ki=2 s, giving Ki=3 N·m/(rad·s). "
            "The compiled hinge inertia I=0.08639 kg·m² was inspected "
            "before these common gains and four controller comparisons."
        ),
        "metrics": metrics,
    }
    return summary, trajectories


def write_artifacts(output_dir: Path) -> dict:
    summary, trajectories = run_comparison()
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    with (output_dir / "trajectories.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=[
                "controller", "time_s", "target_rad", "qpos_rad", "qvel_rad_s",
                "error_rad", "torque_nm", "controller_updated", "saturated",
            ],
            lineterminator="\n",
        )
        writer.writeheader()
        for name, rows in trajectories.items():
            writer.writerows({"controller": name, **row} for row in rows)
    plot_trajectories(trajectories, output_dir / "comparison.png")
    return summary


def plot_trajectories(trajectories: dict, output: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, (angle_axis, torque_axis) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    colors = {"baseline": "#777777", "P": "#d97706", "PI": "#7c3aed", "PD": "#0284c7", "PID": "#15803d"}
    for name, rows in trajectories.items():
        time = [r["time_s"] for r in rows]
        angle_axis.plot(time, [math.degrees(r["qpos_rad"]) for r in rows], label=name, color=colors[name], linewidth=1.5)
        torque_axis.plot(time, [r["torque_nm"] for r in rows], label=name, color=colors[name], linewidth=1.2)
    first = next(iter(trajectories.values()))
    angle_axis.plot([r["time_s"] for r in first], [math.degrees(r["target_rad"]) for r in first], "k--", label="target", linewidth=1)
    angle_axis.set(ylabel="Hinge angle (deg)", title="Day 2 pendulum step response")
    torque_axis.set(xlabel="Simulation time (s)", ylabel="Commanded torque (N·m)")
    angle_axis.legend(ncol=3)
    for axis in (angle_axis, torque_axis):
        axis.grid(alpha=0.25)
    figure.tight_layout()
    figure.savefig(output, dpi=160)
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path("experiments/day02"))
    args = parser.parse_args()
    summary = write_artifacts(args.output_dir)
    print(json.dumps(summary["metrics"], indent=2))


if __name__ == "__main__":
    main()
