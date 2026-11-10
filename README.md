# Mochi Sprout

Mochi Sprout is an open-source robotics learning project with a long-term goal of building a small, expressive bipedal companion robot. The planned character has a rounded cream body, screen-based eyes, and a leaf on its head. The engineering path is **study → experiment → build → evaluate → document**, with a learning gate between milestones.

## Current scope

Day 1 is a reproducible, headless [MuJoCo](https://mujoco.readthedocs.io/en/stable/python.html) pendulum experiment. An explicit MJCF model has one hinge. The runner records simulated time, joint angle (`qpos`), and angular velocity (`qvel`) under normal and reduced gravity, starting from the same state. The five tests check that the model loads, time advances, states stay finite, repeated runs agree, and gravity changes the trajectory. The full configuration and measured results are in the [experiment record](experiments/day01/README.md) and [JSON data](experiments/day01/results.json).

Day 2 adds a torque actuator to the same pendulum and compares a no-control baseline with P, PI, PD, and PID feedback. The [comparison plot](experiments/day02/comparison.png), [metrics](experiments/day02/summary.json), and [full physics-step trajectories](experiments/day02/trajectories.csv) are generated from the experiment runner. See the [Day 2 record](experiments/day02/README.md) for configuration and metric definitions.

Day 3 keeps the Day 2 PID gains fixed and tests isolated hinge inertia, Gaussian angle noise, fixed sensor delay, and one combined case. The [Day 3 record](experiments/day03/README.md) links nine scenario summaries, five plots, and representative controller-update trajectories. The [technical notes](docs/phase0-day3.md) explain the disturbance boundaries and measured limitations.

There is **no physical robot, locomotion policy, hardware integration, or learned controller** in this release. Simulated results have not been validated against hardware.

## Setup and reproduce

Install Python 3.11 and [uv](https://docs.astral.sh/uv/). Clone the repository, then run from its root:

```sh
uv sync --locked --python 3.11
uv run --locked --python 3.11 pytest -q
uv run --locked --python 3.11 python -m mochi_sprout.simulation.pendulum --output experiments/day01/results.json
uv run --locked --python 3.11 python -m mochi_sprout.experiments.day02 --output-dir experiments/day02
uv run --locked --python 3.11 python -m mochi_sprout.experiments.day03 --output-dir experiments/day03
```

The runners regenerate their artifacts and recording timestamps. Compare numerical values with the checked-in records; minor floating-point differences across platforms are possible. `uv.lock` records dependency versions. See the [Day 1 notes](docs/phase0-day1.md), [Day 2 notes](docs/phase0-day2.md), and [Day 3 notes](docs/phase0-day3.md) for methods and limits.

## Reuse the controller

```python
from mochi_sprout.control import PID, PIDConfig

controller = PID(PIDConfig(kp=6, ki=3, kd=1, output_min=-3, output_max=3))
torque_nm = controller.update(target=0.5, measurement=0.0, dt=0.01)
controller.reset()  # before a new episode
```

The generic controller uses no MuJoCo import. Supply target and measurement in consistent units; the example uses radians and returns N·m. To try another controller, implement `reset()` and `update(target, measurement, dt)` plus a `saturated` flag, then pass it to [`run_episode`](src/mochi_sprout/simulation/controlled_pendulum.py). The physics model does not need changing.

For observation uncertainty, pass an [`ObservationPipeline`](src/mochi_sprout/simulation/observation.py) to `run_episode`. It generates angle noise and retrieves causal delayed samples independently of MuJoCo. Set `PendulumModelConfig(inertia_factor=1.5)` to change effective hinge inertia while preserving mass and center of mass. The Day 3 runner shows complete usage.

## Learning path and contributions

The long-term roadmap is to extend physics experiments, evaluate robot dynamics and learning, and only then work toward sim-to-real validation. Future features are proposals, not present capabilities. The [Day 1 learning gate](docs/learning-gates.md) and the human Day 2 and Day 3 learning gates have been recorded as passed.

Contributors can help improve experiment clarity, tests, reproducibility, and quantitative evaluation. Start with [CONTRIBUTING.md](CONTRIBUTING.md). Small, evidence-backed changes are welcome.

## License and assets

Project code and documentation in this repository are licensed under [Apache-2.0](LICENSE). Dependencies retain their own licenses. Character artwork, logos, branding, and other creative assets may require separate licensing; this release includes no such assets.
