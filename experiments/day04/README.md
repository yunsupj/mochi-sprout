# Day 4: filtering and variable delivery delay

**MEASURED simulation evidence.** Day 4 human learning gate: PASSED. The Day 1–3 evidence is unchanged. The [summary](summary.json) contains unrounded configurations, every seed's metrics, means, population standard deviations, ranges, status counts, and the source base revision.

## Configuration and semantics

Day 3's 1 kg, 0.5 m pendulum and fixed PID are reused: gravity −9.81 m/s²; RK4 physics step 0.002 s; controller update 0.01 s; zero initial angle/velocity; +30° target at 0.5 s; 10 s duration; ±3 N·m torque; Kp=6, Ki=3, Kd=1. Noise is independent zero-mean Gaussian at each controller tick with 0.4° standard deviation. Seeds are **11, 23, 37, 53, 71** for stochastic cases and seed 11 for deterministic cases. Separate seeded generators select noise and delay, preserving fair noise draws across filter comparisons.

An EMA updates only when a new measurement is **accepted**: `y=alpha*x+(1-alpha)*previous_y`. Its first accepted value initializes it. Alpha 1 is identity. At the 10 ms update interval, smaller alpha smooths more and lags more; this is a discrete filter, so changing the update interval changes its effective time response. Raw current sensor, latest accepted raw sensor, filtered controller input, and true MuJoCo angle are separate columns.

Fixed delay is 50 ms. Jitter selects each generated sample's delivery delay uniformly from **10/30/50/70/90 ms**, nominal mean 50 ms. The measured five-seed mean scheduled delay was 50.048 ms. Each measurement carries generation and scheduled arrival timestamps. At each controller decision, all arrived messages are processed by arrival timestamp, then generation index. An older measurement timestamp is dropped instead of replacing a newer accepted sample. No sample is visible before arrival; there is no wall-clock sleep. Before first arrival, the controller uses a known zero-angle initial prior, explicitly flagged in the data. The pending samples at the 10 s endpoint are not counted as accepted or dropped. This policy makes accepted observation age (~31.9 ms for jitter) different from nominal scheduled delay.

## Results

Ground-truth physical tracking uses MuJoCo `qpos`. Final-window MAE is 8–10 s; post-step RMSE is 0.5–10 s; settling requires every later physics sample inside ±2°. Torque-change RMS is the RMS of adjacent 10 ms controller commands, in N·m. Raw and filtered error RMS compare the respective signal with true angle at decision time; filtered error therefore includes lag. Stochastic values are five-seed means, with observed range shown for MAE. `null` settling means no settle. Status counts retain failed and unsettled runs.

| Case | MAE ° mean [range] | RMSE ° mean | Settling s mean | Torque-change RMS N·m | Filtered error RMS ° | Status |
|---|---:|---:|---:|---:|---:|---|
| Clean baseline | 0.313 | 4.300 | 3.630 | 0.099 | 0.000 | 1 settled |
| Noise, alpha 1 | 0.390 [0.307–0.485] | 4.396 | 4.152 | 1.715 | 0.400 | 5 settled |
| Noise, alpha 0.8 | 0.319 [0.247–0.354] | 4.323 | 3.798 | 1.242 | 0.331 | 5 settled |
| Noise, alpha 0.2 | 0.296 [0.248–0.326] | 4.126 | 3.626 | 0.245 | 0.695 | 5 settled |
| Noise, alpha 0.1 | 0.289 [0.265–0.310] | 4.450 | 3.630 | 0.150 | 2.050 | 5 settled |
| Fixed 50 ms | 0.321 | 4.177 | 3.718 | 0.109 | 1.049 | 1 settled |
| Jitter, no noise | 0.350 [0.338–0.358] | 4.463 | 3.928 | 0.575 | 0.762 | 5 settled |
| Noise + jitter, alpha 1 | 0.389 [0.333–0.482] | 4.565 | 4.979 | 1.099 | 0.907 | 5 settled |
| Noise + jitter, alpha 0.2 | 241.722 [197.658–290.467] | 191.596 | none | 2.085 | 57.102 | 4 failed; 1 unsettled |

**Unexpected result:** Alpha 0.1 produced the lowest five-seed final-window MAE in the noise-only cases, but its post-step RMSE and current-time filtered error were worse than alpha 0.2 because of lag. Under jitter, alpha 0.2 interacted badly with stale/irregular accepted updates: four seeds crossed the ±360° failure threshold and the fifth never settled. Smoother torque alone is therefore insufficient evidence of improved physical control. The jitter policy dropped a mean 550.8 out-of-order messages per 1,000 generated updates; 444.2 were accepted and about five remained pending at the horizon. This reflects the stated discard policy, not packet loss in physical hardware.

## Artifacts and reproduction

- [Raw versus filtered angle](raw_filtered_angle.png)
- [Torque across alpha](filter_torque.png)
- [Physical tracking across alpha](filter_tracking.png)
- [Fixed delay versus jitter](delay_tracking.png)
- [Combined disturbance and failure](combined_tracking.png)
- [Representative controller updates](representative_updates.csv): seed 11 for all nine cases; generation, arrival, decision, raw, accepted, filtered, true-state, torque, and drop diagnostics.

```sh
uv sync --locked --python 3.11
uv lock --check --python 3.11
uv run --locked --python 3.11 pytest -q
uv run --locked --python 3.11 python -m mochi_sprout.experiments.day04 --output-dir experiments/day04
```

The JSON timestamp changes on rerun. `run_scenario(Scenario("custom", noise_sd_deg=0.3, delays_ms=(20, 40, 60), alpha=0.5), seed=101)` permits custom scenarios; delay values must be controller-interval multiples. `DeliveryPipeline` and `EMA` do not import MuJoCo. No state predictor was implemented because the core combined case exposed instability that deserves bounded diagnosis before another mitigation is added. No filtering or jitter performance is validated on hardware.
