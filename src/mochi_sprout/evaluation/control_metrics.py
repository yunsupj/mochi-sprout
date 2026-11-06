"""Metrics for a sampled, fixed-target step response."""

from math import degrees, isfinite, sqrt


def evaluate_step_response(
    rows: list[dict], *, step_time_s: float, target_rad: float,
    steady_window_s: float = 2.0, settling_band_rad: float,
) -> dict:
    """Return metrics in degrees, seconds, and N·m.

    Error metrics sample states at and after the target step. Effort uses
    controls held over each physics interval, excluding the final endpoint.
    Settling requires all future sampled states to stay in the band.
    Saturation counts only controller updates, including those before the step.
    """
    if len(rows) < 2 or any(b["time_s"] <= a["time_s"] for a, b in zip(rows, rows[1:])):
        raise ValueError("rows must have increasing time")
    final_time = rows[-1]["time_s"]
    if not (0 <= step_time_s < final_time and 0 < steady_window_s <= final_time - step_time_s):
        raise ValueError("invalid step time or steady window")
    post = [r for r in rows if r["time_s"] >= step_time_s - 1e-12]
    steady = [r for r in post if r["time_s"] >= final_time - steady_window_s - 1e-12]
    updates = [r for r in rows[:-1] if r["controller_updated"]]
    effort = rows[:-1]
    settled_at = next((
        index for index, row in enumerate(post)
        if all(abs(future["error_rad"]) <= settling_band_rad for future in post[index:])
    ), None)
    states_finite = all(
        isfinite(r[key]) for r in rows
        for key in ("qpos_rad", "qvel_rad_s", "torque_nm")
    )
    return {
        "steady_state_mae_deg": degrees(sum(abs(r["error_rad"]) for r in steady) / len(steady)),
        "overshoot_deg": degrees(max(0.0, max(r["qpos_rad"] - target_rad for r in post))),
        "settling_time_s": None if settled_at is None else post[settled_at]["time_s"] - step_time_s,
        "rmse_after_step_deg": degrees(sqrt(sum(r["error_rad"] ** 2 for r in post) / len(post))),
        "max_abs_torque_nm": max(abs(r["torque_nm"]) for r in effort),
        "rms_torque_nm": sqrt(sum(r["torque_nm"] ** 2 for r in effort) / len(effort)),
        "saturation_fraction": sum(bool(r["saturated"]) for r in updates) / len(updates),
        "status": (
            "failed_nonfinite_or_over_360deg"
            if not states_finite or any(abs(r["qpos_rad"]) >= 2 * 3.141592653589793 for r in rows)
            else "finite_unsettled" if settled_at is None else "finite_settled"
        ),
    }
