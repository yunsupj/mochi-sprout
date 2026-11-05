# Learning gates

## Day 1: pendulum foundations — passed

A **generalized coordinate** is the smallest variable needed to describe a system's configuration. This one-hinge pendulum needs one angle. MuJoCo stores that angle in `data.qpos[0]`, measured here in radians. Its rate of change is angular velocity, stored in `data.qvel[0]` in radians per second. Acceleration is the rate of change of velocity; MuJoCo computes it from forces, constraints, and the current state as it steps the model.

Gravity pulls on the rod's mass. When the rod is tilted, the pull has a turning effect, or **torque**, about the hinge. A stronger gravity setting usually produces a stronger restoring torque at the same angle. Joint damping also resists motion. The experiment changes only the vertical gravity value; both runs start at the same angle and velocity.

The **timestep** is the simulated time advanced by one `mujoco.mj_step` call. Here it is 0.002 s. We integrate 1,000 steps to reach 2 s, recording the initial state and every 50 steps (0.1 s). A smaller timestep may change numerical error and computing cost; it is not itself proof of physical accuracy.

`qpos` and `qvel` describe state. `ctrl` holds actuator commands. This model has no actuator, so `data.ctrl` has length zero and no command is applied. A later motorized model could set an actuator command in `data.ctrl[...]` before `mj_step`; the actuator definition determines how that command creates force or torque.

**Open-loop** control chooses commands without using observed state to correct them. **Closed-loop** control reads state or sensors and adjusts future commands. Day 1 uses no controller or actuator; its motion is the free response to gravity and damping. This provides a baseline for understanding the dynamics before adding control.

### Five questions — answer before advancing

1. Why does this pendulum need only one generalized coordinate, and where is it stored in MuJoCo?
2. What does `qvel[0]` mean here, and how is acceleration related to it?
3. If both runs begin at the same angle, why can changing gravity alter their later `qpos` trajectories?
4. With a 0.002 s timestep, how many `mj_step` calls make 2 s? What does changing the timestep affect?
5. What would `ctrl` contain if we added a motor, and how would an open-loop command differ from closed-loop control?

**Status:** Passed, as confirmed by the project owner during Day 1 finalization. The questions are retained as a self-check for new contributors. The answers are not recorded here.
