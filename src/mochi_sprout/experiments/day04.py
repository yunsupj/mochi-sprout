"""Reproducible EMA filtering and variable observation-delay study."""

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
from mochi_sprout.evaluation.processing_metrics import evaluate_processing
from mochi_sprout.evaluation.robustness_metrics import aggregate_metrics
from mochi_sprout.simulation.controlled_pendulum import run_episode
from mochi_sprout.simulation.observation import DeliveryPipeline

from .day02 import ExperimentConfig, controller_configs, target_at_time
from .day03 import SEEDS


BASE_MAIN_REVISION = "3a7db4782c22612c77d93892bc88e80f9392b332"
JITTER_MS = (10, 30, 50, 70, 90)


@dataclass(frozen=True)
class Scenario:
    name: str
    noise_sd_deg: float = 0.0
    delays_ms: tuple[int, ...] = (0,)
    alpha: float = 1.0


SCENARIOS = (
    Scenario("baseline"),
    Scenario("noise_unfiltered", noise_sd_deg=0.4),
    Scenario("noise_ema_0_8", noise_sd_deg=0.4, alpha=0.8),
    Scenario("noise_ema_0_2", noise_sd_deg=0.4, alpha=0.2),
    Scenario("noise_ema_0_1", noise_sd_deg=0.4, alpha=0.1),
    Scenario("fixed_50ms", delays_ms=(50,)),
    Scenario("jitter", delays_ms=JITTER_MS),
    Scenario("combined_unfiltered", noise_sd_deg=0.4, delays_ms=JITTER_MS),
    Scenario("combined_ema_0_2", noise_sd_deg=0.4, delays_ms=JITTER_MS, alpha=0.2),
)


def run_scenario(scenario: Scenario, seed: int,
                 config: ExperimentConfig = ExperimentConfig()) -> tuple[dict, list[dict]]:
    pipeline = DeliveryPipeline(
        scenario.noise_sd_deg, tuple(d / 1000 for d in scenario.delays_ms),
        config.controller_dt_s, seed, alpha=scenario.alpha,
    )
    controller = PID(controller_configs(3)["PID"])
    rows = run_episode(
        controller, lambda t: target_at_time(t, config),
        duration_s=config.duration_s, controller_dt_s=config.controller_dt_s,
        observation=pipeline,
    )
    metrics = evaluate_processing(
        rows, step_time_s=config.step_time_s,
        target_rad=config.target_after_step_rad,
        steady_window_s=config.steady_window_s,
        settling_band_rad=config.settling_band_rad,
    )
    return metrics, rows


def run_study(config: ExperimentConfig = ExperimentConfig()) -> tuple[dict, dict[str, list[dict]]]:
    results = {}
    representatives = {}
    for scenario in SCENARIOS:
        stochastic = scenario.noise_sd_deg > 0 or len(scenario.delays_ms) > 1
        seeds = SEEDS if stochastic else (SEEDS[0],)
        per_seed = {}
        for seed in seeds:
            metrics, rows = run_scenario(scenario, seed, config)
            per_seed[str(seed)] = metrics
            if seed == SEEDS[0]:
                representatives[scenario.name] = rows
        results[scenario.name] = {
            "scenario": asdict(scenario), "per_seed": per_seed,
            "aggregate": aggregate_metrics({int(k): v for k, v in per_seed.items()}),
        }
    return {
        "experiment_id": "day04-filtering-jitter",
        "recorded_at": datetime.now(ZoneInfo("America/Los_Angeles")).isoformat(),
        "source_base_revision": BASE_MAIN_REVISION,
        "python_version": platform.python_version(),
        "mujoco_version": mujoco.__version__,
        "config": asdict(config),
        "pid_config": asdict(controller_configs(3)["PID"]),
        "seeds": list(SEEDS),
        "delay_policy": "Uniform discrete per-sample delay; process arrivals by arrival time then generation index; reject older measurement timestamps",
        "filter_policy": "EMA initialized by first accepted sample; zero-angle prior before first delivery",
        "results": results,
    }, representatives


def plot_study(trajectories: dict[str, list[dict]], output_dir: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    def save(name: str, title: str, field: str, cases: tuple[str, ...],
             *, degrees: bool = False) -> None:
        fig, ax = plt.subplots(figsize=(9, 4.5))
        for case in cases:
            rows = trajectories[case]
            y = [math.degrees(r[field]) if degrees else r[field] for r in rows]
            ax.plot([r["time_s"] for r in rows], y, label=case, linewidth=1.1)
        ax.set(title=title, xlabel="Simulation time (s)",
               ylabel="Angle (deg)" if degrees else "Torque (N·m)")
        ax.grid(alpha=0.25); ax.legend()
        fig.tight_layout(); fig.savefig(output_dir / name, dpi=140); plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 4.5))
    rows = trajectories["noise_ema_0_2"]
    for field, label in (("qpos_rad", "true"), ("raw_sensor_rad", "raw"),
                         ("filtered_rad", "filtered")):
        chosen = [r for r in rows if r["controller_updated"]]
        ax.plot([r["time_s"] for r in chosen],
                [math.degrees(r[field]) for r in chosen], label=label, linewidth=1)
    ax.set(title="Raw, filtered, and true angle; alpha 0.2; seed 11",
           xlabel="Simulation time (s)", ylabel="Angle (deg)")
    ax.grid(alpha=0.25); ax.legend(); fig.tight_layout()
    fig.savefig(output_dir / "raw_filtered_angle.png", dpi=140); plt.close(fig)
    save("filter_torque.png", "Torque versus EMA alpha; seed 11", "torque_nm",
         ("noise_unfiltered", "noise_ema_0_8", "noise_ema_0_2", "noise_ema_0_1"))
    save("filter_tracking.png", "Physical angle versus EMA alpha; seed 11", "qpos_rad",
         ("baseline", "noise_unfiltered", "noise_ema_0_8", "noise_ema_0_2", "noise_ema_0_1"), degrees=True)
    save("delay_tracking.png", "Fixed 50 ms versus jitter; seed 11", "qpos_rad",
         ("baseline", "fixed_50ms", "jitter"), degrees=True)
    save("combined_tracking.png", "Noise and jitter with selected EMA; seed 11", "qpos_rad",
         ("baseline", "combined_unfiltered", "combined_ema_0_2"), degrees=True)


def write_artifacts(output_dir: Path) -> dict:
    summary, trajectories = run_study()
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    fields = ("scenario", "seed", "time_s", "qpos_rad", "target_rad", "error_rad",
              "raw_sensor_rad", "accepted_raw_rad", "filtered_rad", "observed_rad",
              "measurement_time_s", "arrival_time_s", "scheduled_delay_s",
              "measurement_age_s", "torque_nm",
              "accepted_count", "dropped_count", "out_of_order_count", "stale",
              "used_initial_prior", "saturated")
    with (output_dir / "representative_updates.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        for name, rows in trajectories.items():
            for row in rows:
                if row["controller_updated"]:
                    writer.writerow({key: name if key == "scenario" else SEEDS[0] if key == "seed"
                                     else row[key] for key in fields})
    plot_study(trajectories, output_dir)
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path("experiments/day04"))
    args = parser.parse_args()
    summary = write_artifacts(args.output_dir)
    for name, result in summary["results"].items():
        metric = result["aggregate"]["numeric"]["steady_state_mae_deg"]
        print(f"{name}: MAE {metric['mean']:.3f}° [{metric['min']:.3f}, {metric['max']:.3f}]")


if __name__ == "__main__":
    main()
