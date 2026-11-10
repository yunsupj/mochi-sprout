# Day 3 evidence handoff — draft, unpublished

Suggested English title: **How Robust Is PID Control? Testing Inertia, Sensor Noise, and Delay in MuJoCo**

Suggested Korean title: **PID 제어기는 얼마나 안정적일까? MuJoCo로 관성·센서 노이즈·지연 실험하기**

**Publication status:** PENDING. SmartWorkLab was not modified. **Code provenance:** Day 3 feature branch from merged Day 2 main `dc24cd5868e3caf548befe0f8e7c8b1415454437`; the feature commit and PR URL should be recorded in the human review handoff, avoiding a self-referential commit hash in this tracked document. The run's local date and dependency versions are in [summary.json](../../experiments/day03/summary.json) and `uv.lock`.

## Evidence to use

The exact configuration is in the [experiment record](../../experiments/day03/README.md). Its [JSON summary](../../experiments/day03/summary.json) has all nine scenarios, five individual seeds for each stochastic scenario, aggregate mean/population SD/range, and compiled model checks. [Representative controller-update samples](../../experiments/day03/representative_updates.csv) retain true, raw, and controller-observed angles plus timestamps and torque for seed 11. Full trajectories are reproducible by the command below.

| Scenario | Final-window MAE ° | Settling s after step | Main observation |
|---|---:|---:|---|
| Baseline | 0.313 | 3.630 | Day 2 PID reference reproduced |
| 1.5× inertia | 0.319 | 3.674 | Holding torque unchanged |
| 2× inertia | 0.320 | 3.680 | Post-step RMSE rose to 4.608° |
| 0.2° noise | 0.308 mean [0.282, 0.321] | 3.650 mean | Five seeds; no guarantee of improvement |
| 0.4° noise | 0.390 mean [0.307, 0.485] | 4.152 mean | Torque-change RMS 1.715 N·m |
| 10 ms delay | 0.315 | 3.648 | Observation age 10 ms after startup |
| 30 ms delay | 0.318 | 3.684 | Observation age 30 ms after startup |
| 50 ms delay | 0.321 | 3.718 | Observation age 50 ms after startup |
| Combined | 0.395 mean [0.298, 0.496] | 4.190 mean | Overshoot 1.477° mean, five seeds |

Graphs: [inertia angle](../../experiments/day03/inertia_angle.png), [true and observed noise angle](../../experiments/day03/noise_observation.png), [torque under noise](../../experiments/day03/noise_torque.png), [fixed delay angle](../../experiments/day03/delay_angle.png), [combined versus baseline](../../experiments/day03/combined_angle.png). These graphs are drawn from measured trajectories. Technical methods and interpretation are in [Day 3 notes](../phase0-day3.md).

Code references: [PID](../../src/mochi_sprout/control/pid.py), [observation pipeline](../../src/mochi_sprout/simulation/observation.py), [MuJoCo adapter](../../src/mochi_sprout/simulation/controlled_pendulum.py), [experiment runner](../../src/mochi_sprout/experiments/day03.py), [robustness metrics](../../src/mochi_sprout/evaluation/robustness_metrics.py), and [tests](../../tests/test_day03_robustness.py).

```sh
uv sync --locked --python 3.11
uv run --locked --python 3.11 pytest -q
uv run --locked --python 3.11 python -m mochi_sprout.experiments.day03 --output-dir experiments/day03
```

Important negative findings and limits: 0.2° noise did not worsen mean final-window MAE in these five seeds, though it greatly increased torque variation. Fixed delay reduced post-step RMSE here while increasing settling time. These are narrow, measured outcomes. No filter or jitter mitigation was tested. The model uses idealized isolated inertia changes and no physical sensor, motor dynamics, contact, or hardware validation. One combined case does not establish general interaction effects. No article should describe these results as robust hardware performance.
