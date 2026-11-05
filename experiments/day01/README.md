# Day 1 experiment record

- **ID:** `day01-pendulum-gravity`
- **Date:** November 4, 2026, America/Los_Angeles
- **Source provenance:** the run preceded the initial commit. The stable experiment ID, MJCF, lockfile, and JSON values identify it. The initial source commit SHA is reported separately at the publication gate, avoiding a self-referential hash in tracked data.
- **Environment:** macOS 26.6.2 ARM64, Python 3.11.8, MuJoCo 3.17.0
- **Model:** one y-axis hinge; 0.5 m, 1 kg capsule; joint damping 0.01; RK4 integrator
- **Initial state:** `qpos=0.5 rad`, `qvel=0 rad/s` for both runs
- **Physics timestep:** 0.002 s; **duration:** 2 s (1,000 steps); **recording:** initial state plus every 50 steps
- **Gravity:** A = `(0, 0, -9.81)` m/s²; B = `(0, 0, -3.0)` m/s²
- **Control:** no actuator and no applied `ctrl`

## Actual results

[`results.json`](results.json) contains metadata, the MJCF, both 21-sample trajectories, and calculated observations. At the final sample, A had `qpos=-0.21383601484656056 rad`, `qvel=2.069170516798967 rad/s`; B had `qpos=0.3920126521060395 rad`, `qvel=0.587366201059721 rad/s`. The largest sampled angle difference was `0.9085448978228956 rad`. These are measured simulation outputs. The expectation that gravity would change the trajectory preceded the run; the numerical values did not.

## Commands and checks

The project was initialized with `uv init --lib --name mochi-sprout --python 3.11 --vcs none --no-workspace --no-pin-python .` and `git init`. Dependencies were added with `uv add 'mujoco==3.17.0'` and `uv add --dev pytest`, selecting Python 3.11 explicitly. The first install attempt failed on PyPI DNS in the development sandbox; an authorized retry succeeded. `uv lock` captured the resolved versions.

The original test invocation used `uv run --locked` with Python 3.11 and reported **5 passed in 3.68s**. The experiment invocation used `uv run --locked` with Python 3.11 and `python -m mochi_sprout.simulation.pendulum --output experiments/day01/results.json`. Portable exact reproduction commands appear in the root README. The original invocation used machine-specific interpreter and cache paths; they are intentionally excluded from this public record.

## Reproduce and limits

From the project root, run the three commands in the root README with Python 3.11 and uv. The lockfile preserves package versions; other hardware and MuJoCo builds may give small numerical differences. The idealized model has not been validated against physical hardware. Sampling every 0.1 s can miss extrema between samples. See the [engineering notes](../../docs/phase0-day1.md) for the method and limitations.
