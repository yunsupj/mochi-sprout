# Day 1 handoff

**Scope completed:** Python 3.11/uv project, explicit MJCF pendulum, two headless gravity runs, JSON trajectory record, five tests, and an unpublished SmartWorkLab article draft. The project owner confirmed the Day 1 learning gate as **PASSED**.

**Experiment:** `day01-pendulum-gravity` in [`results.json`](../experiments/day01/results.json). With identical initial state and 2 s duration, the final angles were −0.21383601484656056 rad at gravity −9.81 m/s² and 0.3920126521060395 rad at −3.0 m/s². The largest sampled angle difference was 0.9085448978228956 rad. These are simulated observations, not hardware measurements.

**Repository:** the intended public home is [yunsupj/mochi-sprout](https://github.com/yunsupj/mochi-sprout). The initial source commit is the revision that contains the MJCF, lockfile, tests, and recorded JSON together. Its SHA is reported separately at the publication approval gate and can be resolved locally with `git rev-parse HEAD`; it is deliberately not embedded here or in the JSON because that would change the commit being identified.

**Verification:** `uv lock --check --python 3.11` succeeded, and `uv run --locked --python 3.11 pytest -q` reported **5 passed in 0.18s** during finalization. A fresh experiment run reproduced both 21-sample trajectories and the calculated observations exactly on the same machine. Reproduce with the commands in the root README; the original Day 1 test run reported 5 passed in 3.68s.

**Limits:** one ideal hinge and rigid capsule; no actuator, sensors, contacts, robot body, or hardware validation. The result record samples every 0.1 s, so a peak between samples may be missed. Cross-platform results may differ slightly.

**Next learning topic:** PID and feedback control. No Day 2 implementation is included.
