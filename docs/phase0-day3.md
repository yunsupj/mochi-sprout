# Phase 0, Day 3 — controller robustness

**Learning gate:** PASSED by the project owner before implementation. **Study status:** measured simulated evidence, pending human engineering review. This work does not validate a humanoid joint or hardware.

## Architecture

The reusable [PID controller](../src/mochi_sprout/control/pid.py) remains unchanged and has no MuJoCo dependency. The [observation pipeline](../src/mochi_sprout/simulation/observation.py) converts angle noise from degrees to radians, draws from a dedicated seeded generator, and stores timestamped samples in a causal buffer. It is also simulator-independent. The [pendulum adapter](../src/mochi_sprout/simulation/controlled_pendulum.py) advances MuJoCo and passes either the true angle (Day 2) or the pipeline's observation (Day 3) to the controller. It records true `qpos`, raw sensor angle, controller observation, sample timestamp, and observation age separately. The [Day 3 runner](../src/mochi_sprout/experiments/day03.py) specifies scenarios and seeds. The [robustness evaluator](../src/mochi_sprout/evaluation/robustness_metrics.py) reuses Day 2 step-response metrics and adds observation, derivative-input, torque-change, and age diagnostics. Evaluators read trajectories only.

The zero-noise, zero-delay route reproduces Day 2's PID trajectory exactly on this machine. Day 1 and Day 2 code and evidence remain available. A controller that implements the existing `reset`, `update`, and `saturated` interface can use the same observation pipeline.

## Mechanics and experimental controls

**MEASURED:** The original compiled hinge inertia is 0.086392578125 kg·m². `PendulumModelConfig(inertia_factor=...)` increases the central inertia components transverse to the rod by the same amount. The mass matrix then shows 1.5× and 2× effective hinge inertia, while +30° gravity holding torque stays 1.22625 N·m. This isolates inertia in the equations while retaining a positive, triangle-valid central inertia tensor. The unchanged visual capsule no longer describes the full implied mass distribution, so the perturbation is idealized rather than a redesigned physical rod.

**HYPOTHESIS:** More inertia at unchanged gains may slow or alter the response; noise in measured angle may appear strongly in a derivative-on-measurement term; stale observations may degrade feedback. We did not assume a particular monotonic response or that any case would fail.

The one-factor-at-a-time matrix includes 1×, 1.5×, 2× inertia; 0°, 0.2°, 0.4° Gaussian sensor noise; and 0, 10, 30, 50 ms fixed sensor delay. Every case retains the Day 2 1 kg body, −9.81 m/s² gravity, RK4, 0.002 s physics step, 0.01 s controller step, ±3 N·m torque, 0° initial state, +30° step at 0.5 s, 10 s duration, and Kp=6, Ki=3, Kd=1. The combined case uses 2× inertia, 0.4° noise, and 50 ms delay. No gains were retuned.

Sensor noise is generated once per controller decision and never changes MuJoCo `qpos`. A delayed sample is selected from timestamps no later than decision time minus requested delay; t=0 is held during history warmup. The controller sees the delayed noisy value, while performance metrics always use true state. Delays must be integer multiples of 10 ms. Zero delay selects the current update sample. The derivative term remains derivative-on-measurement, so noise differences between updates enter it directly. The recorded derivative-input error RMS quantifies this effect before the PID gain is applied; torque-change RMS additionally reflects clipping and the other PID terms.

## Findings

**MEASURED:** The [full evidence table](../experiments/day03/README.md) and [unrounded results](../experiments/day03/summary.json) show baseline final-window MAE 0.313° and settling 3.630 s. Doubling inertia raised post-step RMSE from 4.300° to 4.608° and settling to 3.680 s, while the final-window MAE changed only to 0.320°. The 50 ms delay increased settling to 3.718 s but reduced post-step RMSE to 4.177°; that is a measured tradeoff for this one transient, not proof that delay improves control. At 0.4° noise, five-seed mean final-window MAE was 0.390° (range 0.307–0.485°), settling 4.152 s (range 3.792–4.364 s), and RMS adjacent-command change 1.715 N·m versus 0.099 N·m at baseline. The combined case had mean overshoot 1.477° (range 1.248–1.910°) and settling 4.190 s.

**INTERPRETATION:** Fixed PID gains tolerated the tested inertia and delay changes without a failed or unsettled run. Sensor noise caused much larger command fluctuation and occasional saturation, consistent with the derivative path amplifying sample-to-sample angle variation. The 0.2° noise case had a slightly lower mean final-window MAE than the no-noise baseline, but this five-seed, 10 s result does not establish that noise improves control. The combined case's overshoot is a signal to study interactions further, not enough to identify a general interaction law.

**NOT VERIFIED:** Robustness to variable delay or jitter, out-of-order delivery, filtering, other seeds beyond five, other initial states or targets, altered gains, timestep convergence, sensor bias, actuator dynamics, joint friction uncertainty, limb contacts, or hardware. Fixed delay has predictable age after warmup; jitter would produce varying age and could change which sampled states are used. There is no jitter experiment here, so no claim about its numerical effect. An optional future comparison could use fixed 30 ms against seeded 10/30/50 ms with timestamps and a rule to discard samples older than the last consumed sample. A small factorial study could examine whether inertia, noise, and delay interactions persist beyond the one combined point.

## Metric boundaries and future use

Day 2 positive-target overshoot semantics were preserved. `settling_time_s=null` means the response did not stay within ±2° by the end; it is never converted to zero. Aggregates report a settled count and compute settling statistics only over observed settled runs. Final-window MAE is a finite-window statistic, not a guaranteed steady-state error. The status rule detects nonfinite states or angles at/above ±360°, but other control failures could still be physically important. Noise statistics are empirical over five seeds, not deterministic guarantees. The plotted and recorded trajectories are simulator outputs.

Interchangeable humanoid limbs will eventually have different inertia, sensing, and timing. This small one-hinge study demonstrates interfaces and measurements that can be reused when those models exist. It does not imply that the current pendulum already models a complete limb, balance task, or physical robot.

## Reproduce

```sh
uv sync --locked --python 3.11
uv lock --check --python 3.11
uv run --locked --python 3.11 pytest -q
uv run --locked --python 3.11 python -m mochi_sprout.experiments.day03 --output-dir experiments/day03
```
