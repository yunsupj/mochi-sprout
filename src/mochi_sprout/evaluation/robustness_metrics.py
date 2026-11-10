"""Ground-truth tracking plus observation and update-to-update diagnostics."""

from math import degrees, sqrt
from statistics import mean, pstdev

from .control_metrics import evaluate_step_response


def _rms(values: list[float]) -> float:
    return sqrt(sum(value * value for value in values) / len(values))


def evaluate_robustness(
    rows: list[dict], *, step_time_s: float, target_rad: float,
    steady_window_s: float, settling_band_rad: float,
) -> dict:
    # Keep physical tracking tied to ground truth even if a future adapter
    # supplies an observation-based or stale error_rad field.
    true_rows = [{**row, "error_rad": row["target_rad"] - row["qpos_rad"]}
                 for row in rows]
    metrics = evaluate_step_response(
        true_rows, step_time_s=step_time_s, target_rad=target_rad,
        steady_window_s=steady_window_s, settling_band_rad=settling_band_rad,
    )
    updates = [row for row in rows[:-1] if row["controller_updated"]]
    if not updates or any("observed_rad" not in row for row in updates):
        raise ValueError("robustness metrics require controller observations")
    raw_error = [row["raw_sensor_rad"] - row["qpos_rad"] for row in updates]
    observed_error = [row["observed_rad"] - row["qpos_rad"] for row in updates]
    torque_changes = [b["torque_nm"] - a["torque_nm"]
                      for a, b in zip(updates, updates[1:])]
    derivative_error = [
        ((b["observed_rad"] - a["observed_rad"])
         - (b["qpos_rad"] - a["qpos_rad"])) / (b["time_s"] - a["time_s"])
        for a, b in zip(updates, updates[1:])
    ]
    ages = [row["measurement_age_s"] for row in updates]
    metrics.update({
        "raw_sensor_error_rms_deg": degrees(_rms(raw_error)),
        "controller_observation_error_rms_deg": degrees(_rms(observed_error)),
        "derivative_input_error_rms_rad_s": _rms(derivative_error),
        "rms_torque_change_nm_per_update": _rms(torque_changes),
        "mean_measurement_age_s": mean(ages),
        "max_measurement_age_s": max(ages),
    })
    return metrics


def aggregate_metrics(per_seed: dict[int, dict]) -> dict:
    """Summarize numeric metrics; retain null settling as an explicit count."""
    if not per_seed:
        raise ValueError("at least one seed is required")
    first = next(iter(per_seed.values()))
    summary = {"seed_count": len(per_seed),
               "status_counts": {status: sum(row["status"] == status for row in per_seed.values())
                                 for status in sorted({row["status"] for row in per_seed.values()})},
               "settled_count": sum(row["settling_time_s"] is not None
                                    for row in per_seed.values()),
               "unsettled_count": sum(row["settling_time_s"] is None
                                      for row in per_seed.values()),
               "numeric": {}}
    for key, value in first.items():
        if isinstance(value, (int, float)) or value is None and key == "settling_time_s":
            values = [row[key] for row in per_seed.values() if row[key] is not None]
            summary["numeric"][key] = (
                None if not values else {
                    "mean": mean(values), "population_sd": pstdev(values),
                    "min": min(values), "max": max(values), "observed_count": len(values),
                }
            )
    return summary
