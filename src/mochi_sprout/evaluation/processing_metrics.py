"""Observation-delivery diagnostics alongside true-state tracking metrics."""

from math import degrees, sqrt
from statistics import mean

from .robustness_metrics import evaluate_robustness


def evaluate_processing(rows: list[dict], *, step_time_s: float, target_rad: float,
                        steady_window_s: float, settling_band_rad: float) -> dict:
    result = evaluate_robustness(
        rows, step_time_s=step_time_s, target_rad=target_rad,
        steady_window_s=steady_window_s, settling_band_rad=settling_band_rad,
    )
    updates = [row for row in rows[:-1] if row["controller_updated"]]
    def rms(values: list[float]) -> float:
        return sqrt(sum(v * v for v in values) / len(values))
    result.update({
        "filtered_error_rms_deg": degrees(rms([
            row["filtered_rad"] - row["qpos_rad"] for row in updates])),
        "accepted_count": sum(row["accepted_count"] for row in updates),
        "dropped_count": sum(row["dropped_count"] for row in updates),
        "out_of_order_count": sum(row["out_of_order_count"] for row in updates),
        "stale_decision_fraction": sum(row["stale"] for row in updates) / len(updates),
        "initial_prior_decisions": sum(row["used_initial_prior"] for row in updates),
        "mean_scheduled_delay_s": mean(row["scheduled_delay_s"] for row in updates),
    })
    return result
