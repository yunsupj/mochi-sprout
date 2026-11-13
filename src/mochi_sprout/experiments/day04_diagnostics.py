"""Bounded, non-mutating diagnosis of the Day 4 combined-case failure."""

import argparse
import csv
import json
import math
from dataclasses import asdict
from pathlib import Path
from statistics import mean

from mochi_sprout.control import PID
from mochi_sprout.evaluation.processing_metrics import evaluate_processing
from mochi_sprout.simulation.controlled_pendulum import run_episode
from mochi_sprout.simulation.observation import DeliveryPipeline

from .day02 import ExperimentConfig, controller_configs, target_at_time
from .day03 import SEEDS
from .day04 import JITTER_MS, Scenario, run_scenario


DIAGNOSTICS = (
    Scenario("jitter_ema_0_2_no_noise", delays_ms=JITTER_MS, alpha=.2),
    Scenario("fixed_50ms_noise_ema_0_2", noise_sd_deg=.4, delays_ms=(50,), alpha=.2),
    Scenario("jitter_noise_ema_0_8", noise_sd_deg=.4, delays_ms=JITTER_MS, alpha=.8),
)
REFERENCE = (
    Scenario("combined_unfiltered", noise_sd_deg=.4, delays_ms=JITTER_MS),
    Scenario("combined_ema_0_2", noise_sd_deg=.4, delays_ms=JITTER_MS, alpha=.2),
)


class TracePID(PID):
    """Read controller state around update; delegate output unchanged to PID."""

    def __init__(self):
        super().__init__(controller_configs(3)["PID"])
        self.trace: list[dict] = []

    def reset(self) -> None:
        super().reset()
        self.trace = []

    def update(self, target: float, measurement: float, dt: float) -> float:
        error = target - measurement
        derivative = (0 if self.previous_measurement is None else
                      -(measurement - self.previous_measurement) / dt)
        p = self.config.kp * error
        i_candidate = self.config.ki * (self.integral + error * dt)
        d = self.config.kd * derivative
        before = self.integral
        torque = super().update(target, measurement, dt)
        self.trace.append({
            "p_nm": p, "i_candidate_nm": i_candidate, "d_nm": d,
            "unclipped_nm": p + i_candidate + d, "torque_nm": torque,
            "integral_before_rad_s": before,
            "integral_after_rad_s": self.integral,
            "saturated": self.saturated,
        })
        return torque


def traced_run(scenario: Scenario, seed: int) -> tuple[dict, list[dict], list[dict]]:
    config = ExperimentConfig()
    controller = TracePID()
    pipeline = DeliveryPipeline(
        scenario.noise_sd_deg, tuple(d / 1000 for d in scenario.delays_ms),
        config.controller_dt_s, seed, alpha=scenario.alpha,
    )
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
    updates = [r for r in rows if r["controller_updated"]]
    assert len(updates) == len(controller.trace) == 1000
    for row, trace in zip(updates, controller.trace, strict=True):
        assert row["torque_nm"] == trace["torque_nm"]
    return metrics, rows, controller.trace


def audit_deliveries(rows: list[dict]) -> dict:
    """Independently reconstruct generation, delivery, and acceptance counts."""
    updates = [r for r in rows if r["controller_updated"]]
    events = sorted((i * .01 + r["scheduled_delay_s"], i, i * .01)
                    for i, r in enumerate(updates))
    delivered_times = []
    accepted_measurement_times = []
    accepted = dropped = cursor = 0
    latest = -1.0
    for i, row in enumerate(updates):
        decision = i * .01
        while cursor < len(events) and events[cursor][0] <= decision + 1e-12:
            arrival, _, measurement = events[cursor]
            delivered_times.append(arrival)
            if measurement > latest:
                accepted += 1
                latest = measurement
                accepted_measurement_times.append(measurement)
            else:
                dropped += 1
            cursor += 1
    assert accepted == sum(r["accepted_count"] for r in updates)
    assert dropped == sum(r["dropped_count"] for r in updates)
    def intervals(values: list[float]) -> dict:
        gaps = [b - a for a, b in zip(values, values[1:])]
        return {"count": len(gaps), "min_s": min(gaps), "mean_s": mean(gaps),
                "max_s": max(gaps), "zero_count": sum(g == 0 for g in gaps)}
    return {
        "generated": len(updates), "delivered": len(delivered_times),
        "accepted": accepted, "dropped_outdated": dropped,
        "pending_at_last_decision": len(events) - cursor,
        "drop_fraction_of_delivered": dropped / len(delivered_times),
        "delivery_interarrival": intervals(delivered_times),
        "accepted_measurement_timestamp_intervals": intervals(accepted_measurement_times),
    }


def make_diagnostics() -> tuple[dict, list[dict]]:
    results = {}
    excerpts = []
    for scenario in REFERENCE + DIAGNOSTICS:
        per_seed = {}
        for seed in SEEDS:
            metrics, rows, trace = traced_run(scenario, seed)
            canonical_metrics, canonical_rows = run_scenario(scenario, seed)
            assert rows == canonical_rows and metrics == canonical_metrics
            updates = [r for r in rows if r["controller_updated"]]
            audit = audit_deliveries(rows)
            first_90 = next((r["time_s"] for r in rows if abs(math.degrees(r["qpos_rad"])) >= 90), None)
            first_360 = next((r["time_s"] for r in rows if abs(math.degrees(r["qpos_rad"])) >= 360), None)
            per_seed[str(seed)] = {
                "status": metrics["status"],
                "final_window_mae_deg": metrics["steady_state_mae_deg"],
                "max_abs_true_angle_deg": max(abs(math.degrees(r["qpos_rad"])) for r in rows),
                "first_abs_90deg_time_s": first_90,
                "first_abs_360deg_time_s": first_360,
                "saturation_fraction": metrics["saturation_fraction"],
                "saturated_controller_updates": sum(r["saturated"] for r in updates),
                "saturation_duration_s": sum(r["saturated"] for r in updates) * .01,
                "mean_observation_age_s": metrics["mean_measurement_age_s"],
                "max_observation_age_s": metrics["max_measurement_age_s"],
                "delivery_audit": audit,
            }
            if scenario.name == "combined_ema_0_2" and seed in (11, 23):
                # Fixed threshold defines an onset independently of the plot.
                comparison = run_scenario(REFERENCE[0], seed)[1]
                divergence = next(i for i, (a, b) in enumerate(zip(rows, comparison, strict=True))
                                  if a["time_s"] >= .5 and
                                  abs(math.degrees(a["qpos_rad"] - b["qpos_rad"])) >= 5)
                center = round(divergence / 5)
                per_seed[str(seed)]["first_5deg_divergence_time_s"] = rows[divergence]["time_s"]
                for i in range(max(0, center - 5), min(len(updates), center + 6)):
                    previous = updates[i - 1]["measurement_time_s"] if i else None
                    excerpts.append({
                        "seed": seed, "time_s": updates[i]["time_s"],
                        "true_angle_deg": math.degrees(updates[i]["qpos_rad"]),
                        "true_velocity_rad_s": updates[i]["qvel_rad_s"],
                        "target_deg": math.degrees(updates[i]["target_rad"]),
                        "raw_sensor_deg": math.degrees(updates[i]["raw_sensor_rad"]),
                        "filtered_deg": math.degrees(updates[i]["filtered_rad"]),
                        "measurement_time_s": updates[i]["measurement_time_s"],
                        "previous_measurement_time_s": previous,
                        "arrival_time_s": updates[i]["arrival_time_s"],
                        "observation_age_s": updates[i]["measurement_age_s"],
                        "accepted_count": updates[i]["accepted_count"],
                        "dropped_count": updates[i]["dropped_count"],
                        **trace[i],
                    })
        results[scenario.name] = {"scenario": asdict(scenario), "per_seed": per_seed}
    # The filter must leave the underlying noise and delay draws unchanged.
    for seed in SEEDS:
        _, raw_rows, _ = traced_run(REFERENCE[0], seed)
        _, filtered_rows, _ = traced_run(REFERENCE[1], seed)
        raw_updates = [r for r in raw_rows if r["controller_updated"]]
        filtered_updates = [r for r in filtered_rows if r["controller_updated"]]
        assert all(
            a["scheduled_delay_s"] == b["scheduled_delay_s"] and
            math.isclose(a["raw_sensor_rad"] - a["qpos_rad"],
                         b["raw_sensor_rad"] - b["qpos_rad"], abs_tol=1e-13)
            for a, b in zip(raw_updates, filtered_updates, strict=True)
        )
    return {
        "experiment_id": "day04-combined-instability-diagnosis",
        "source_day04_commit": "0783b954ce235333a4432c5ad2a75c4df6d5d818",
        "divergence_rule": "first physics sample after target step with >=5 deg true-angle difference from paired unfiltered run",
        "trace_rule": "eleven controller updates centered on the threshold crossing; PID terms computed from pre-update state, then original PID.update called",
        "seeds": list(SEEDS), "results": results,
    }, excerpts


def write_artifacts(output_dir: Path) -> dict:
    summary, excerpts = make_diagnostics()
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "diagnostics.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    with (output_dir / "diagnostic_onsets.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(excerpts[0]), lineterminator="\n")
        writer.writeheader(); writer.writerows(excerpts)
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path("experiments/day04"))
    args = parser.parse_args()
    summary = write_artifacts(args.output_dir)
    for name, result in summary["results"].items():
        print(name, {seed: row["status"] for seed, row in result["per_seed"].items()})


if __name__ == "__main__":
    main()
