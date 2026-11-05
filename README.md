# Mochi Sprout

Mochi Sprout is an open-source robotics learning project with a long-term goal of building a small, expressive bipedal companion robot. The planned character has a rounded cream body, screen-based eyes, and a leaf on its head. The engineering path is **study → experiment → build → evaluate → document**, with a learning gate between milestones.

## Current scope

Day 1 is a reproducible, headless [MuJoCo](https://mujoco.readthedocs.io/en/stable/python.html) pendulum experiment. An explicit MJCF model has one hinge. The runner records simulated time, joint angle (`qpos`), and angular velocity (`qvel`) under normal and reduced gravity, starting from the same state. The five tests check that the model loads, time advances, states stay finite, repeated runs agree, and gravity changes the trajectory. The full configuration and measured results are in the [experiment record](experiments/day01/README.md) and [JSON data](experiments/day01/results.json).

There is **no physical robot, locomotion policy, hardware integration, actuator, or learned controller** in this release. Simulated results have not been validated against hardware.

## Setup and reproduce

Install Python 3.11 and [uv](https://docs.astral.sh/uv/). Clone the repository, then run from its root:

```sh
uv sync --locked --python 3.11
uv run --locked --python 3.11 pytest -q
uv run --locked --python 3.11 python -m mochi_sprout.simulation.pendulum --output experiments/day01/results.json
```

The last command regenerates `results.json` with a new recording timestamp. Compare the trajectory values with the checked-in record; minor floating-point differences across platforms are possible. `uv.lock` records dependency versions. See [Day 1 notes](docs/phase0-day1.md) for the method and limits.

## Learning path and contributions

The long-term roadmap is to learn feedback control, extend physics experiments, evaluate robot dynamics and learning, and only then work toward sim-to-real validation. Future features are proposals, not present capabilities. The [Day 1 learning gate](docs/learning-gates.md) is recorded as passed; the next learning topic is PID and feedback control, with no Day 2 implementation here.

Contributors can help improve experiment clarity, tests, reproducibility, and quantitative evaluation. Start with [CONTRIBUTING.md](CONTRIBUTING.md). Small, evidence-backed changes are welcome.

## License and assets

Project code and documentation in this repository are licensed under [Apache-2.0](LICENSE). Dependencies retain their own licenses. Character artwork, logos, branding, and other creative assets may require separate licensing; this release includes no such assets.
