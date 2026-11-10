# Day 3: PID robustness evidence

The human Day 3 learning gate passed before this implementation. This study keeps Day 2's PID and compares one factor at a time plus one combined stress case. All numbers below are **MEASURED simulation outputs**, not hardware measurements. The full unrounded per-seed metrics, aggregation, configurations, and compiled model checks are in [summary.json](summary.json).

## Configuration

The original 1 kg, 0.5 m capsule has a compiled hinge inertia of 0.086392578125 kg·m² and gravity holding torque of 1.22625 N·m at +30°. Gravity is −9.81 m/s², joint damping 0.01 N·m·s/rad, integration RK4 with 0.002 s physics steps. Controller decisions occur every 0.01 s. All runs start at 0° and 0 rad/s; target steps from 0° to +30° at 0.5 s; duration is 10 s; torque limit is ±3 N·m. The fixed PID gains are Kp=6, Ki=3, Kd=1. Seeds are 11, 23, 37, 53, 71. Deterministic cases use seed 11; noisy cases use all five.

Angle noise is independent zero-mean Gaussian per controller update, with standard deviation 0, 0.2°, or 0.4°. It enters only the controller observation. Fixed delays are 0, 10, 30, or 50 ms. The history selects the newest sample at or before decision time minus delay; before a full history exists it holds the t=0 sample. The measured age is thus less than the requested delay during startup. No real-time sleep is involved. The combined case uses 2× inertia, 0.4° noise, and 50 ms delay; it is one selected interaction case.

For inertia perturbations, the MuJoCo model's two transverse *central* body inertia components are increased equally. The tensor remains positive with valid triangle inequalities. The body mass, center of mass, gravity, damping, actuator gear, and torque range are unchanged. Compiled effective hinge inertias are 0.086392578125, 0.1295888671875, and 0.17278515625 kg·m² for 1×, 1.5×, and 2×. The +30° holding torque remains 1.22625 N·m for all three. This is an idealized sensitivity test: the visible capsule geometry is unchanged even though the inertia tensor represents a different mass distribution.

## Results

All errors use MuJoCo ground-truth angle. Final-window MAE covers 8–10 s. RMSE covers 0.5–10 s. Positive-target overshoot is maximum angle above +30°. Settling requires every subsequent physics sample to remain within ±2°; times are seconds after the step. RMS torque uses held commands over 5,000 physics intervals. Saturation is the fraction of 1,000 controller decisions with unclipped output beyond ±3 N·m. Torque change RMS is the difference between adjacent controller-update commands. Stochastic entries show five-seed mean and observed range; the full population SD and individual seeds are in JSON.

| Scenario | Final MAE ° mean [range] | RMSE ° mean | Overshoot ° mean | Settling s mean [range] | RMS torque change N·m | Saturation mean |
|---|---:|---:|---:|---:|---:|---:|
| Baseline | 0.313 [0.313, 0.313] | 4.300 | 0.000 | 3.630 | 0.099 | 0.1% |
| Inertia 1.5× | 0.319 [0.319, 0.319] | 4.468 | 0.000 | 3.674 | 0.098 | 0.2% |
| Inertia 2× | 0.320 [0.320, 0.320] | 4.608 | 0.000 | 3.680 | 0.097 | 0.2% |
| Noise 0.2° | 0.308 [0.282, 0.321] | 4.314 | 0.000 | 3.650 [3.586, 3.726] | 0.888 | 0.14% |
| Noise 0.4° | 0.390 [0.307, 0.485] | 4.396 | 0.027 | 4.152 [3.792, 4.364] | 1.715 | 3.9% |
| Delay 10 ms | 0.315 [0.315, 0.315] | 4.256 | 0.000 | 3.648 | 0.100 | 0.2% |
| Delay 30 ms | 0.318 [0.318, 0.318] | 4.184 | 0.000 | 3.684 | 0.102 | 0.4% |
| Delay 50 ms | 0.321 [0.321, 0.321] | 4.177 | 0.000 | 3.718 | 0.109 | 0.6% |
| Combined | 0.395 [0.298, 0.496] | 4.649 | 1.477 | 4.190 [3.838, 4.494] | 1.680 | 4.1% |

All five seeds in each noisy case settled within 10 s and stayed finite. The combined case is not evidence about every possible interaction. Its overshoot of 1.248–1.910° across seeds differs from the zero overshoot in the corresponding single-factor cases. The 50 ms delay observation age averages 49.85 ms across decisions because startup samples are younger. The 0.4° noise raw sensor error RMS is about 0.400°; its derivative-input error RMS is about 0.974 rad/s and the commanded torque variation is much higher than baseline. This shows derivative sensitivity to measurement noise in this unfiltered controller.

## Artifacts and reproduce

- [Angle response versus inertia](inertia_angle.png)
- [True and observed angle under 0.4° noise](noise_observation.png)
- [Torque under noise](noise_torque.png)
- [Angle response versus delay](delay_angle.png)
- [Baseline versus combined case](combined_angle.png)
- [Representative controller-update samples](representative_updates.csv): seed 11, all nine scenarios, including raw and delayed observations. Full metrics for all seeds are in JSON. Regenerate all full physics-step trajectories by running the module.

```sh
uv sync --locked --python 3.11
uv run --locked --python 3.11 pytest -q
uv run --locked --python 3.11 python -m mochi_sprout.experiments.day03 --output-dir experiments/day03
```

The run writes a new `recorded_at` timestamp. Numerical values were reproduced on this development machine; floating-point results can vary slightly across platforms. We did not test variable delay, filtering, different gain sets, nonlinear actuator dynamics, contact, hardware, or a range of initial states. The current overshoot definition is specific to a positive target step. Read the [technical analysis](../../docs/phase0-day3.md) before extrapolating.

Developers can call `run_scenario(Scenario("custom", inertia_factor=1.5, noise_sd_deg=0.3, delay_ms=20), seed=101)` from `mochi_sprout.experiments.day03`; delays must be multiples of the 10 ms update interval. The `ObservationPipeline` can also be passed to `run_episode` with another controller implementing the existing scalar interface. The summary generator's `SCENARIOS` and `SEEDS` constants define the committed matrix; changing them and rerunning writes a new, explicitly configured study.
