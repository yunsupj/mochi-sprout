# Phase 0, Day 1 — first physics simulation

## Objective

Build a minimal Python and MuJoCo environment, run a one-joint pendulum headlessly, and compare its motion under two gravity settings. This is a learning experiment, not a validated robot model.

## Method

The explicit MJCF in `src/mochi_sprout/simulation/pendulum.py` defines a 0.5 m, 1 kg capsule attached to a hinge about the y axis. Joint damping is 0.01. MuJoCo uses RK4 integration with a 0.002 s timestep. We set the initial angle to 0.5 rad and angular velocity to 0 rad/s, then simulate 2 s without an actuator. Experiment A uses vertical gravity −9.81 m/s²; B uses −3.0 m/s². Both use the same initial state, model, duration, and sampling schedule. The script records `data.time`, `data.qpos[0]`, and `data.qvel[0]` at t=0 and after each 50 steps.

The model object (`MjModel`) holds geometry and options; the data object (`MjData`) holds changing state. `mj_step` advances the simulation. The runner sets gravity on each model before creating its data object. The full sampled trajectories and metadata are in [`results.json`](../experiments/day01/results.json).

## Verified observations

Recorded on November 4, 2026, using Python 3.11.8, MuJoCo 3.17.0, and macOS 26.6.2 ARM64. At simulated t≈2.0 s:

| Gravity z (m/s²) | `qpos` (rad) | `qvel` (rad/s) |
|---:|---:|---:|
| −9.81 | −0.21383601484656056 | 2.069170516798967 |
| −3.0 | 0.3920126521060395 | 0.587366201059721 |

The maximum absolute angle difference across the 21 recorded samples was **0.9085448978228956 rad**. Both trajectories began at `qpos=0.5 rad`, `qvel=0 rad/s`. Stronger gravity produces a different restoring torque and therefore a different rate and phase of swing. These values are simulation output, not hardware measurements.

## Tests and limitations

`uv run --locked --python 3.11 pytest -q` reported **5 passed in 3.68s**. The checks cover model loading, increasing time, finite state, repeated-run consistency to 1e−12 absolute tolerance, and an angle difference exceeding 0.01 rad after changing gravity.

The rod is a simple rigid capsule with an ideal hinge. There is no motor, sensor, contact task, manufacturing tolerance, or measured hardware parameter. We have not compared against a real pendulum or tested timestep convergence. The deterministic test verifies repeatability on this environment, not bitwise identity across platforms. The JSON records every 0.1 s, although physics advances every 0.002 s. Floating-point time can differ slightly from exact decimal values.

## Reproduce

```sh
uv sync --locked --python 3.11
uv run --locked --python 3.11 pytest -q
uv run --locked --python 3.11 python -m mochi_sprout.simulation.pendulum --output experiments/day01/results.json
```

The project owner confirmed that the [Day 1 learning gate](learning-gates.md) was passed. The next learning topic is PID and feedback control; no Day 2 work is implemented here.
