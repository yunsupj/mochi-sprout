# Day 2: torque pendulum controller comparison

**Experiment ID:** `day02-pid-step-comparison`

**Run date:** November 5, 2026 (America/Los_Angeles)

**Base source revision:** Day 1 commit `b387c803a7ddf6192ace782515fff94377a7ff95`. The Day 2 PR identifies the exact new source revision without embedding a circular commit hash in the data.

## Configuration

The Day 1 capsule and hinge are reused: length 0.5 m, mass 1 kg, center of mass 0.25 m below the hinge, hinge damping 0.01 N·m·s/rad, gravity −9.81 m/s², RK4 integrator, physics timestep 0.002 s. The compiled hinge inertia is 0.086392578125 kg·m². Positive hinge angle moves the center of mass toward negative x. At +30°, the gravity holding torque is +1.22625 N·m. A gear-1 motor maps a positive `ctrl` value to positive joint torque, with `ctrlrange` ±3 N·m. This has enough authority to hold the target in this model.

Each condition begins at angle 0°, angular velocity 0 rad/s. The target is 0° until 0.5 s, then +30°. The run ends at 10 s. The controller updates every 0.01 s (every five physics steps), and its torque is held between updates. The no-control baseline applies zero torque. All conditions use the same model, initial state, torque limit, and numerical settings.

Gain selection preceded comparison. Using the compiled inertia, a desired natural frequency near 8 rad/s gives `Kp≈I·8²=5.53`, rounded to **6 N·m/rad** for every controller. Desired damping ratio near 0.7 gives `Kd≈2·0.7·sqrt(Kp·I)=1.01`, rounded to **1 N·m·s/rad** for PD and PID. Integral time `Kp/Ki=2 s` gives **Ki=3 N·m/(rad·s)** for PI and PID. These are one documented gain set, not a search for a winning controller. The exact settings are in [`summary.json`](summary.json).

## Metric definitions

All state metrics include samples at and after the 0.5 s step; the final 10 s sample is included. **Final-window MAE** is the mean absolute angle error from 8–10 s in degrees; for an unsettled response it is a window statistic, not proof of steady state. **Overshoot** is `max(0, highest angle after step − 30°)` in degrees. **Settling time** is elapsed time after 0.5 s to the first sample from which every later sample stays within ±2°; `null` means never settled. **RMSE** is the root mean squared angle error from 0.5–10 s in degrees. **Maximum absolute torque** and **RMS torque** use the 5,000 held commands over physics intervals, excluding the final endpoint, in N·m. **Saturation fraction** is the fraction of 1,000 controller updates whose unconstrained output exceeded ±3 N·m, including the initial 0.5 s. A failure is flagged if state becomes nonfinite or angle reaches ±360°; `finite_unsettled` means neither failure nor settling was observed within 10 s.

## Measured results

| Condition | Final-window MAE (°) | Overshoot (°) | Settling (s after step) | RMSE (°) | Max torque (N·m) | RMS torque (N·m) | Saturation |
|---|---:|---:|---:|---:|---:|---:|---:|
| Baseline | 30.000 | 0.000 | none | 30.000 | 0.000 | 0.000 | 0.0% |
| P | 33.191 | 40.396 | none | 28.699 | 3.000 | 2.254 | 36.9% |
| PI | 41.815 | 53.249 | none | 36.796 | 3.000 | 2.426 | 49.8% |
| PD | 8.560 | 0.000 | none | 9.091 | 3.000 | 0.881 | 0.1% |
| PID | 0.313 | 0.000 | 3.630 | 4.300 | 3.000 | 1.124 | 0.1% |

The baseline stayed at 0° because it began at the downward equilibrium without torque. P and PI oscillated, saturated often, and did not settle; PI's final-window error was worse than P's. PD damped the motion but retained a gravity-related angle offset. PID settled and had the lowest final-window error with these gains. This is one model and one gain set; it does not establish that PID is generally best.

## Artifacts and reproduction

- [`trajectories.csv`](trajectories.csv): all 5,001 physics-step samples for each condition, with time, target, angle, angular velocity, error, held torque, controller-update flag, and saturation flag.
- [`summary.json`](summary.json): exact settings, environment, compiled model checks, and unrounded metrics.
- [`comparison.png`](comparison.png): angle and commanded torque plotted from the same trajectories written to CSV.

From the repository root with Python 3.11 and uv:

```sh
uv sync --locked --python 3.11
uv run --locked --python 3.11 pytest -q
uv run --locked --python 3.11 python -m mochi_sprout.experiments.day02 --output-dir experiments/day02
```

The timestamp changes on rerun. Numerical trajectories were reproducible on the development machine; different platforms may differ slightly. The model has no measured hardware parameters, sensor noise, motor dynamics, backlash, or contact. The P and PI oscillations should not be extrapolated to physical hardware. See [Day 2 technical notes](../../docs/phase0-day2.md).

The final local run used Python 3.11.8, MuJoCo 3.17.0, NumPy 2.4.6, matplotlib 3.11.2, and pytest 9.2.0 on macOS ARM64. `uv lock --check --python 3.11` passed, and the complete suite (including all five Day 1 tests) reported **16 passed in 0.57s**. A second run produced identical numerical summary values, identical 25,005 CSV rows, and an identical plot file on the same machine; only the summary recording timestamp changed.
