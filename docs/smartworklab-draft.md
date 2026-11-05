# Draft for Muse — Building Mochi Sprout: My First Physics Simulation

**Status:** Unpublished draft based on the Day 1 experiment. No website changes were made.

## Learning objective

I wanted to establish a reproducible Python and MuJoCo workflow and see how gravity changes the motion of a simple pendulum before tackling a robot body or controller.

## Background theory

A pendulum with one hinge has one generalized coordinate: its angle. Angular velocity describes how quickly that angle changes. Gravity creates a turning effect about the hinge when the rod is tilted, while damping resists movement. A simulator advances this state in small timesteps; the results depend on the model and numerical method.

## Implementation and setup

I wrote an explicit MJCF model for a 0.5 m, 1 kg capsule attached to a y-axis hinge with damping 0.01. A Python runner loads it with the official MuJoCo bindings and steps it headlessly using RK4 at 0.002 s per step. Each run begins at 0.5 rad with zero angular velocity and lasts 2 s. Run A uses gravity −9.81 m/s²; run B uses −3.0 m/s². I recorded simulated time, hinge angle, and angular velocity at the start and every 0.1 s. No actuator or controller is present.

## Verified observations

The November 4, 2026 run used Python 3.11.8 and MuJoCo 3.17.0 on macOS ARM64. At simulated t≈2 s, run A ended at −0.213836 rad and 2.069171 rad/s; run B ended at 0.392013 rad and 0.587366 rad/s. The maximum angle difference among recorded samples was 0.908545 rad. All five pytest checks passed, covering model loading, time advancement, finite values, repeatability, and a measurable gravity effect. The full values are in `experiments/day01/results.json`.

## Failures and limitations

The first dependency attempt could not reach PyPI from the sandbox. Retrying with approved network access succeeded. uv's default cache was also inaccessible, so setup used a temporary cache. The pendulum is an idealized rigid body with an ideal hinge. I have no physical pendulum measurements to validate it against, and I have not checked timestep convergence. Recorded samples are 0.1 s apart, so the reported maximum is only the maximum among those samples.

## Reproduce

From the project root with Python 3.11 and uv:

```sh
uv sync --locked --python 3.11
uv run --locked --python 3.11 pytest -q
uv run --locked --python 3.11 python -m mochi_sprout.simulation.pendulum --output experiments/day01/results.json
```

## Next learning objectives

The project owner has confirmed the Day 1 learning gate as passed. The next learning topic is PID and feedback control; this draft does not include a controller or any Day 2 result.
