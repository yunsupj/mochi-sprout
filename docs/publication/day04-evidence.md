# Day 4 publication evidence — draft, unpublished

Suggested EN title: **Reducing PID Sensor Noise: Filtering, Jitter, and State Estimation in MuJoCo**

Suggested KO title: **PID 제어의 센서 노이즈 줄이기: MuJoCo 필터링·지연 실험**

**Status:** SmartWorkLab not modified or published. The title mentions state estimation as a question for future work; no estimator was implemented or validated. The Day 4 feature branch starts at merged Day 3 main `3a7db4782c22612c77d93892bc88e80f9392b332`. The final feature commit and CI status belong in the review handoff to avoid a self-referential tracked SHA.

**MEASURED:** [Day 4 record](../../experiments/day04/README.md) has the complete scenario table and metric definitions. [JSON](../../experiments/day04/summary.json) retains all five stochastic seed results, status counts, and population SD/ranges. [Representative update CSV](../../experiments/day04/representative_updates.csv) includes raw, accepted, filtered, true angle, arrival and generation time, and torque. Figures: [sensor](../../experiments/day04/raw_filtered_angle.png), [torque](../../experiments/day04/filter_torque.png), [filter tracking](../../experiments/day04/filter_tracking.png), [delay](../../experiments/day04/delay_tracking.png), [combined failure](../../experiments/day04/combined_tracking.png).

**MEASURED:** Noise-only alpha 0.2 reduced mean adjacent-command RMS from 1.715 to 0.245 N·m and post-step RMSE from 4.396° to 4.126° over five seeds. Alpha 0.1 made torque still smoother (0.150 N·m) but worsened post-step RMSE (4.450°). Uniform 10/30/50/70/90 ms jitter had a measured mean scheduled delay of 50.048 ms. Noise and jitter with alpha 0.2 failed the ±360° criterion in four of five seeds; the fifth did not settle within 10 s.

**HYPOTHESIS:** Filtering may attenuate derivative noise, but stale/irregular samples plus filter memory may destabilize this tuned PID.

**INTERPRETATION:** This measured interaction makes command smoothness an inadequate success metric. The causal timestamp policy and 10 ms controller schedule are central to interpreting the result.

**NOT VERIFIED:** No estimator, Kalman filter, hardware sensor, locomotion, contact, broader gain/target study, or sim-to-real result. The virtual inertia and other Day 3 assumptions remain idealized. Article claims must retain the failure and all limitations. See [engineering notes](../phase0-day4.md) and the [code](../../src/mochi_sprout/experiments/day04.py).

**Verification:** The local full suite passed 35 tests and `uv lock --check --python 3.11` passed before PR creation. Two fresh experiment runs matched the committed JSON metrics, representative CSV, and all five figures, except for the recording timestamp. GitHub CI results belong in the PR handoff after the feature branch is pushed.

```sh
uv sync --locked --python 3.11
uv run --locked --python 3.11 pytest -q
uv run --locked --python 3.11 python -m mochi_sprout.experiments.day04 --output-dir experiments/day04
```
