"""Headless, open-loop MuJoCo pendulum experiment."""

import argparse
import json
import platform
import sys
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

import mujoco


MJCF = """
<mujoco model="day01_pendulum">
  <option timestep="0.002" gravity="0 0 -9.81" integrator="RK4"/>
  <worldbody>
    <body name="bob" pos="0 0 0">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0.01"/>
      <geom name="rod" type="capsule" fromto="0 0 0 0 0 -0.5"
            size="0.025" mass="1"/>
    </body>
  </worldbody>
</mujoco>
"""

INITIAL_ANGLE_RAD = 0.5
INITIAL_VELOCITY_RAD_S = 0.0
DURATION_S = 2.0
SAMPLE_EVERY_STEPS = 50
GRAVITY_Z_M_S2 = {"A_default": -9.81, "B_modified": -3.0}


def make_model(gravity_z: float = -9.81) -> mujoco.MjModel:
    model = mujoco.MjModel.from_xml_string(MJCF)
    model.opt.gravity[2] = gravity_z
    return model


def simulate(gravity_z: float = -9.81) -> list[dict[str, float]]:
    """Return time, hinge position, and velocity from one deterministic run."""
    model = make_model(gravity_z)
    data = mujoco.MjData(model)
    data.qpos[0] = INITIAL_ANGLE_RAD
    data.qvel[0] = INITIAL_VELOCITY_RAD_S
    mujoco.mj_forward(model, data)

    steps = round(DURATION_S / model.opt.timestep)
    trajectory = [_sample(data)]
    for step in range(1, steps + 1):
        mujoco.mj_step(model, data)
        if step % SAMPLE_EVERY_STEPS == 0 or step == steps:
            trajectory.append(_sample(data))
    return trajectory


def _sample(data: mujoco.MjData) -> dict[str, float]:
    return {
        "time_s": float(data.time),
        "qpos_rad": float(data.qpos[0]),
        "qvel_rad_s": float(data.qvel[0]),
    }


def experiment_record() -> dict:
    trajectories = {name: simulate(g) for name, g in GRAVITY_Z_M_S2.items()}
    normal = trajectories["A_default"]
    modified = trajectories["B_modified"]
    return {
        "experiment_id": "day01-pendulum-gravity",
        "recorded_at": datetime.now(ZoneInfo("America/Los_Angeles")).isoformat(),
        "python_version": platform.python_version(),
        "mujoco_version": mujoco.__version__,
        "operating_system": platform.platform(),
        "git_commit_sha": None,
        "model": {
            "mjcf": MJCF.strip(),
            "joint": "hinge around y axis",
            "rod_length_m": 0.5,
            "rod_mass_kg": 1.0,
            "joint_damping": 0.01,
            "integrator": "RK4",
        },
        "initial_conditions": {
            "qpos_rad": INITIAL_ANGLE_RAD,
            "qvel_rad_s": INITIAL_VELOCITY_RAD_S,
        },
        "timestep_s": make_model().opt.timestep,
        "duration_s": DURATION_S,
        "sample_interval_steps": SAMPLE_EVERY_STEPS,
        "gravity_z_m_s2": GRAVITY_Z_M_S2,
        "trajectories": trajectories,
        "observations": {
            "final_qpos_rad": {name: rows[-1]["qpos_rad"] for name, rows in trajectories.items()},
            "final_qvel_rad_s": {name: rows[-1]["qvel_rad_s"] for name, rows in trajectories.items()},
            "max_abs_qpos_difference_rad": max(
                abs(a["qpos_rad"] - b["qpos_rad"])
                for a, b in zip(normal, modified, strict=True)
            ),
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("experiments/day01/results.json"))
    args = parser.parse_args()
    record = experiment_record()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(record["observations"], indent=2))
    print(f"Saved {args.output}", file=sys.stderr)


if __name__ == "__main__":
    main()
