"""One-factor-at-a-time PID robustness study with a combined stress case."""

import argparse
import csv
import json
import math
import platform
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

import mujoco

from mochi_sprout.control import PID
from mochi_sprout.evaluation.robustness_metrics import aggregate_metrics, evaluate_robustness
from mochi_sprout.simulation.controlled_pendulum import (
    PendulumModelConfig, make_controlled_model, run_episode,
)
from mochi_sprout.simulation.observation import ObservationPipeline

from .day02 import ExperimentConfig, controller_configs, model_check, target_at_time


SEEDS = (11, 23, 37, 53, 71)
BASE_MAIN_REVISION = "dc24cd5868e3caf548befe0f8e7c8b1415454437"


@dataclass(frozen=True)
class Scenario:
    name: str
    inertia_factor: float = 1.0
    noise_sd_deg: float = 0.0
    delay_ms: int = 0


SCENARIOS = (
    Scenario("baseline"),
    Scenario("inertia_1_5x", inertia_factor=1.5),
    Scenario("inertia_2x", inertia_factor=2.0),
    Scenario("noise_0_2deg", noise_sd_deg=0.2),
    Scenario("noise_0_4deg", noise_sd_deg=0.4),
    Scenario("delay_10ms", delay_ms=10),
    Scenario("delay_30ms", delay_ms=30),
    Scenario("delay_50ms", delay_ms=50),
    Scenario("combined", inertia_factor=2.0, noise_sd_deg=0.4, delay_ms=50),
)


def run_scenario(
    scenario: Scenario, seed: int, config: ExperimentConfig = ExperimentConfig(),
) -> tuple[dict, list[dict]]:
    physical = PendulumModelConfig(inertia_factor=scenario.inertia_factor)
    model = make_controlled_model(physical)
    observation = ObservationPipeline(
        scenario.noise_sd_deg, scenario.delay_ms / 1000,
        config.controller_dt_s, seed,
    )
    controller = PID(controller_configs(physical.torque_limit_nm)["PID"])
    rows = run_episode(
        controller, lambda t: target_at_time(t, config),
        duration_s=config.duration_s, controller_dt_s=config.controller_dt_s,
        model_config=physical, observation=observation,
    )
    metrics = evaluate_robustness(
        rows, step_time_s=config.step_time_s,
        target_rad=config.target_after_step_rad,
        steady_window_s=config.steady_window_s,
        settling_band_rad=config.settling_band_rad,
    )
    checks = model_check(physical, config.target_after_step_rad)
    checks["body_principal_inertia_kg_m2"] = model.body_inertia[1].tolist()
    return {"metrics": metrics, "compiled_model": checks}, rows


def run_study(config: ExperimentConfig = ExperimentConfig()) -> tuple[dict, dict[str, list[dict]]]:
    results = {}
    representatives = {}
    for scenario in SCENARIOS:
        seeds = SEEDS if scenario.noise_sd_deg else (SEEDS[0],)
        per_seed = {}
        for seed in seeds:
            result, rows = run_scenario(scenario, seed, config)
            per_seed[str(seed)] = result["metrics"]
            if seed == SEEDS[0]:
                representatives[scenario.name] = rows
                physical = result["compiled_model"]
        results[scenario.name] = {
            "scenario": asdict(scenario),
            "compiled_model": physical,
            "per_seed": per_seed,
            "aggregate": aggregate_metrics({int(k): v for k, v in per_seed.items()}),
        }
    summary = {
        "experiment_id": "day03-pid-robustness",
        "recorded_at": datetime.now(ZoneInfo("America/Los_Angeles")).isoformat(),
        "source_base_revision": BASE_MAIN_REVISION,
        "python_version": platform.python_version(),
        "mujoco_version": mujoco.__version__,
        "experiment_config": asdict(config),
        "pid_config": asdict(controller_configs(3)["PID"]),
        "seeds": list(SEEDS),
        "noise_assumption": "Independent zero-mean Gaussian angle noise at each 10 ms controller update",
        "delay_policy": "Newest sample at or before decision time minus requested delay; initial sample held until available",
        "results": results,
    }
    return summary, representatives


def _line(axis, rows: list[dict], field: str, label: str, *, degrees: bool = False) -> None:
    x = [r["time_s"] for r in rows]
    y = [math.degrees(r[field]) if degrees else r[field] for r in rows]
    axis.plot(x, y, label=label, linewidth=1.2)


def plot_study(trajectories: dict[str, list[dict]], output_dir: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    def save(name: str, title: str, ylabel: str, lines: list[tuple[str, str, bool]]) -> None:
        fig, ax = plt.subplots(figsize=(9, 4.5))
        for scenario, field, in_degrees in lines:
            label = scenario.replace("_", " ")
            if name == "noise_observation.png":
                label = "true angle" if field == "qpos_rad" else "controller observation"
            _line(ax, trajectories[scenario], field, label, degrees=in_degrees)
        ax.set(title=title, xlabel="Simulation time (s)", ylabel=ylabel)
        ax.grid(alpha=0.25)
        ax.legend()
        fig.tight_layout()
        fig.savefig(output_dir / name, dpi=140)
        plt.close(fig)

    save("inertia_angle.png", "Angle response versus hinge inertia", "Angle (deg)",
         [(name, "qpos_rad", True) for name in ("baseline", "inertia_1_5x", "inertia_2x")])
    save("noise_observation.png", "True and observed angle, 0.4° noise, seed 11", "Angle (deg)",
         [("noise_0_4deg", "qpos_rad", True), ("noise_0_4deg", "observed_rad", True)])
    save("noise_torque.png", "Command torque under sensor noise, seed 11", "Torque (N·m)",
         [(name, "torque_nm", False) for name in ("baseline", "noise_0_2deg", "noise_0_4deg")])
    save("delay_angle.png", "Angle response versus fixed sensor delay", "Angle (deg)",
         [(name, "qpos_rad", True) for name in ("baseline", "delay_10ms", "delay_30ms", "delay_50ms")])
    save("combined_angle.png", "Baseline versus combined disturbances, seed 11", "Angle (deg)",
         [(name, "qpos_rad", True) for name in ("baseline", "combined")])


def write_artifacts(output_dir: Path) -> dict:
    summary, trajectories = run_study()
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    fields = (
        "scenario", "seed", "time_s", "target_rad", "qpos_rad", "qvel_rad_s",
        "error_rad", "raw_sensor_rad", "observed_rad", "measurement_time_s",
        "measurement_age_s", "torque_nm", "saturated",
    )
    with (output_dir / "representative_updates.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        for name, rows in trajectories.items():
            for row in rows:
                if row["controller_updated"]:
                    writer.writerow({key: (name if key == "scenario" else SEEDS[0] if key == "seed"
                                         else row[key]) for key in fields})
    plot_study(trajectories, output_dir)
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path("experiments/day03"))
    args = parser.parse_args()
    summary = write_artifacts(args.output_dir)
    for name, record in summary["results"].items():
        mae = record["aggregate"]["numeric"]["steady_state_mae_deg"]
        print(f"{name}: final-window MAE {mae['mean']:.3f}° (range {mae['min']:.3f}–{mae['max']:.3f}°)")


if __name__ == "__main__":
    main()
