# Day 2 technical evidence for Muse

**Proposed article title:** “Mochi Sprout Learns Feedback: Comparing P, PI, PD, and PID on a Pendulum”

**Publication status:** evidence handoff only; no SmartWorkLab article has been published.

## Learning objectives

Explain feedback torque control, P/I/D terms, anti-windup, derivative damping, actuator limits, and the difference between the 0.002 s physics step and 0.01 s controller update. Compare controllers through reproducible measurements without assuming PID always wins.

## Verified technical basis

**MEASURED:** The controlled MJCF retains Day 1's 0.5 m, 1 kg capsule, y-axis hinge, damping 0.01 N·m·s/rad, gravity −9.81 m/s², and RK4 integrator. MuJoCo reports hinge inertia 0.086392578125 kg·m² and +1.22625 N·m torque to hold +30°. Gear 1 maps positive `ctrl` to positive joint torque; `ctrlrange` is ±3 N·m. Both initial angle and velocity are zero. Target is 0° until 0.5 s, then +30° until 10 s. Controller updates every 0.01 s; physics advances every 0.002 s. Full configuration and versions are in [`summary.json`](../../experiments/day02/summary.json).

**HYPOTHESIS (before run):** P might have gravity-related offset; PI might reduce offset; PD might damp oscillation; PID might balance these effects. These were possibilities, not guaranteed rankings.

**MEASURED:** Final-window mean absolute angle error (last 2 s): baseline 30.000°, P 33.191°, PI 41.815°, PD 8.560°, PID 0.313°. Overshoot: P 40.396°, PI 53.249°, baseline/PD/PID 0°. Only PID satisfied the permanent ±2° settling criterion within 10 s, at 3.630 s after the step. P and PI saturated at 36.9% and 49.8% of controller updates; PD and PID each at 0.1%. The [experiment record](../../experiments/day02/README.md) defines every metric and reports RMSE and effort too.

**INTERPRETATION:** With these gains and this low-damping model, P and PI kept oscillating; integral action alone did not improve the observed final-window error. Derivative damping reduced oscillation. PID combined that damping with integral correction and performed best in this single comparison.

**NOT VERIFIED:** Physical robot accuracy, robustness to altered gain sets or model parameters, safety of a real actuator, general superiority of PID, and performance on locomotion tasks.

## Evidence and code paths

- Generic controller: [`src/mochi_sprout/control/pid.py`](../../src/mochi_sprout/control/pid.py).
- MuJoCo adapter: [`src/mochi_sprout/simulation/controlled_pendulum.py`](../../src/mochi_sprout/simulation/controlled_pendulum.py).
- Metrics: [`src/mochi_sprout/evaluation/control_metrics.py`](../../src/mochi_sprout/evaluation/control_metrics.py).
- Experiment configuration and runner: [`src/mochi_sprout/experiments/day02.py`](../../src/mochi_sprout/experiments/day02.py).
- Graph: [`experiments/day02/comparison.png`](../../experiments/day02/comparison.png).
- Full trajectory CSV: [`experiments/day02/trajectories.csv`](../../experiments/day02/trajectories.csv).
- Unrounded metrics and settings: [`experiments/day02/summary.json`](../../experiments/day02/summary.json).

**Source provenance:** the Day 2 branch starts from published Day 1 commit `b387c803a7ddf6192ace782515fff94377a7ff95`. The Day 2 pull request will provide a stable source reference; its URL can be added here after creation without embedding a self-referential SHA.

## Reproduce

```sh
uv sync --locked --python 3.11
uv run --locked --python 3.11 pytest -q
uv run --locked --python 3.11 python -m mochi_sprout.experiments.day02 --output-dir experiments/day02
```

The recording timestamp changes on rerun. Small cross-platform floating-point differences are possible.

**MEASURED verification:** `uv lock --check --python 3.11` succeeded and `uv run --locked --python 3.11 pytest -q` reported **16 passed in 0.57s**, including the Day 1 tests. A second run matched all numerical summary values, all 25,005 CSV rows, and the plot bytes on the development machine.

## Failed expectations and claims to avoid

PI did **not** reduce final-window error in this run; its oscillation and saturation were worse than P's. P, PI, and PD did not settle within the observation window. The dependency/setup checks succeeded; no simulation or test crashed. The model lacks hardware validation. Do not claim that PID is always superior, that the simulation is physically accurate for Mochi Sprout hardware, that the controller is ready for a real robot, or that this experiment demonstrates walking or reinforcement learning.
