# Phase 0, Day 2 — modular feedback control

## Goal and architecture

The Day 2 PID fundamentals learning gate was passed by the project owner before this implementation. We added a reusable scalar [`PID`](../src/mochi_sprout/control/pid.py) with no MuJoCo import, a [MuJoCo pendulum adapter](../src/mochi_sprout/simulation/controlled_pendulum.py), a [step-response evaluator](../src/mochi_sprout/evaluation/control_metrics.py), and an [experiment runner](../src/mochi_sprout/experiments/day02.py). The controller knows only target, measurement, and update interval. The adapter knows how to set `data.ctrl[0]` and advance physics. The evaluator reads trajectories rather than changing the simulation. This allows another controller to implement the small `reset`, `update`, and `saturated` interface without modifying the physical model.

## Control concepts

**Feedback control** measures the current angle and updates a torque command toward a target. Error is `target − angle`. **P** output responds to present error; in this pendulum a nonzero angle error is needed for P to provide gravity-holding torque. **I** accumulates error over time and can remove that offset, but can also drive oscillation. **D** responds to the negative change in measured angle, providing velocity-like damping. Our implementation uses derivative-on-measurement, `−Kd·(measurement − previous_measurement)/dt`, so a sudden target step alone does not cause a derivative kick. It uses zero derivative on the first update.

The actuator is a direct-drive hinge motor with gear 1, so its `ctrl` value is joint torque in N·m. Its ±3 N·m range clamps the command. **Integral windup** occurs when an integral term keeps growing while the actuator cannot supply the requested output. Conditional anti-windup here skips an integral update if the requested output exceeds a limit and the error would drive it farther beyond that limit; integration resumes when error would unwind it. The output is still clamped at the limit. This protects controller state but does not guarantee a good response.

MuJoCo advances physics in 0.002 s steps, while the controller updates every 0.01 s. One command is held for five physics steps. At the 0.5 s step, all controllers see the same +30° target. The numerical physics step and controller update interval affect accuracy and response and are not interchangeable.

**Overshoot** means angle above the target after the step. **Settling** requires every later sampled angle to remain within ±2°; a response that briefly enters the band and leaves it does not count. **Final-window MAE** measures error over the last 2 s, but for an unsettled response it should not be interpreted as a true steady-state error. Exact definitions and measurements are in the [experiment record](../experiments/day02/README.md).

## Gain procedure and actual observations

The compiled hinge inertia was 0.086392578125 kg·m². We chose a frequency near 8 rad/s and rounded `I·ω²` to shared `Kp=6 N·m/rad`. A damping ratio near 0.7 led to `Kd=1 N·m·s/rad`; an integral time of 2 s led to `Ki=3 N·m/(rad·s)`. P, PI, PD, and PID use the same available gains and ±3 N·m output bounds. This is a documented starting set, not a claim of optimal tuning.

In the measured 10 s run, P and PI kept oscillating and were saturated for 36.9% and 49.8% of controller updates. PI's final-window MAE was 41.815°, worse than P's 33.191°. PD avoided overshoot but remained 8.560° away on average in the final 2 s. PID settled 3.630 s after the target step and had final-window MAE 0.313°. The no-control baseline stayed at 0° and therefore had a 30° error after the step. See the [plot](../experiments/day02/comparison.png) and [unrounded metrics](../experiments/day02/summary.json). These outcomes contradicted the simple expectation that PI alone would reduce error within this 10 s window; derivative damping mattered substantially with the low joint damping and selected gains. PID was best on these particular metrics and conditions, not universally.

## Limitations

The model is the idealized Day 1 rod and hinge plus a direct torque motor. The effective inertia and gravity holding torque are simulator-derived, not hardware measurements. There is no sensor noise, delay beyond the 0.01 s update interval, friction beyond joint damping, motor electrical dynamics, mechanical backlash, or contact. We ran one initial state, one target step, one gain set, and one torque limit. The P/PI responses remained finite and under one revolution, but never settled; a longer run or different gains could change the comparison. No physical-correctness claim follows from deterministic tests. The next engineering question is robustness across gains, delays, and model uncertainty; that work is outside this Day 2 scope.
