import math

from mochi_sprout.experiments.day04 import run_scenario
from mochi_sprout.experiments.day04_diagnostics import (
    REFERENCE, audit_deliveries, traced_run,
)
from mochi_sprout.simulation.controlled_pendulum import make_controlled_model


def test_delivery_audit_and_failure_reproduce_without_controller_changes() -> None:
    scenario = REFERENCE[1]
    metrics, rows, trace = traced_run(scenario, 11)
    canonical_metrics, canonical_rows = run_scenario(scenario, 11)
    assert metrics == canonical_metrics and rows == canonical_rows
    audit = audit_deliveries(rows)
    assert (audit["generated"], audit["delivered"], audit["accepted"],
            audit["dropped_outdated"], audit["pending_at_last_decision"]) == (1000, 995, 425, 570, 5)
    assert metrics["status"] == "failed_nonfinite_or_over_360deg"
    assert max(abs(r["qpos_rad"]) for r in rows) > 2 * math.pi
    assert make_controlled_model().jnt_limited[0] == 0
    assert len(trace) == 1000


def test_filter_choice_preserves_noise_and_delivery_draws() -> None:
    _, unfiltered, _ = traced_run(REFERENCE[0], 23)
    _, filtered, _ = traced_run(REFERENCE[1], 23)
    a = [r for r in unfiltered if r["controller_updated"]]
    b = [r for r in filtered if r["controller_updated"]]
    assert all(x["scheduled_delay_s"] == y["scheduled_delay_s"]
               for x, y in zip(a, b, strict=True))
    assert all(math.isclose(x["raw_sensor_rad"] - x["qpos_rad"],
                            y["raw_sensor_rad"] - y["qpos_rad"], abs_tol=1e-13)
               for x, y in zip(a, b, strict=True))
