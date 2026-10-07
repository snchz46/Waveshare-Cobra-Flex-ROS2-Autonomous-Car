# 03 · State estimation

> **In one sentence:** an extended Kalman filter (`robot_localization`) turns
> drifting odometry into the smooth `odom → base_footprint` transform every
> other part of the stack builds on — fed by the ZED's visual odometry on the
> car, and by wheel velocities and the IMU in Gazebo.

[← 02 System architecture](02_system_architecture.md) · [Concepts](README.md) · Next: [04 SLAM and mapping →](04_slam_and_mapping.md)

---

## Concept

### Odometry and drift

Odometry integrates motion: wheel encoders, an IMU or a camera (visual
odometry) estimate how far the robot moved since the last step, and the pose is
the sum of all those steps. Every step carries a small error, so the integrated
pose **drifts** without bound. A skid-steer robot drifts fastest in yaw,
because it can only turn by dragging its wheels sideways
([Kinematics §3.3](../../assets/Mathematical%20Model/Kinematics.md)).

### Bayes filter

State estimation keeps a belief over the state $x_t$ and updates it in two
steps, with control $u_t$ and measurement $z_t$:

$$
\overline{bel}(x_t) = \int p(x_t \mid u_t, x_{t-1})\, bel(x_{t-1})\, dx_{t-1}
\qquad
bel(x_t) = \eta\, p(z_t \mid x_t)\, \overline{bel}(x_t)
$$

The first is the **prediction** with the motion model, the second the
**correction** with the measurement model.

### Kalman filter and EKF

With Gaussian beliefs and linear models the Bayes filter becomes the Kalman
filter. The **extended** Kalman filter linearises nonlinear models
$f$ and $h$ around the current estimate (Jacobians $F$, $H$):

$$
\begin{aligned}
\text{predict:}\quad & \hat x^- = f(\hat x, u), && P^- = F P F^\top + Q \\
\text{update:}\quad & K = P^- H^\top (H P^- H^\top + R)^{-1}, && \hat x = \hat x^- + K\,(z - h(\hat x^-)),\quad P = (I - K H)\,P^-
\end{aligned}
$$

$Q$ is the process noise (how much the model is trusted), $R$ the measurement
noise (how much the sensor is trusted). The gain $K$ weighs them against each
other.

### `robot_localization`

The `ekf_node` of `robot_localization` estimates a 15-dimensional state:
position $(x, y, z)$, orientation (roll, pitch, yaw), their velocities and the
linear accelerations. Each input gets a 15-entry boolean `config` vector that
selects which of its components are fused. `two_d_mode` pins $z$, roll, pitch
and their rates to zero — right for a ground robot.

---

## In this repository

Two configurations, one per world:

| | Car: [`ekf_hw.yaml`](../../src/cobraflex/config/ekf_hw.yaml) | Gazebo: [`ekf_gazebo.yaml`](../../src/cobraflex/config/ekf_gazebo.yaml) |
| --- | --- | --- |
| Rate | 30 Hz | 30 Hz |
| Inputs | `/zed/zed_node/odom`: **x, y, yaw** (absolute visual-odometry pose) | `odom` (DiffDrive): **vx, vy, vyaw**; `imu`: roll, pitch, yaw, vyaw, ax |
| IMU | Block present but commented out | Fused |
| `publish_tf` | `true` — the EKF owns `odom → base_footprint` | `false` — the ground-truth `OdometryPublisher` owns it |
| `initial_estimate_covariance` | Unset, deliberately | 1.0 on every state |
| Output | `/odometry/filtered` + TF | `/odometry/filtered` only (no consumer; runs for parity) |

Both use `world_frame: odom`, `base_link_frame: base_footprint` and
`two_d_mode: true`. The hardware process noise $Q$ is diagonal: 0.05 on x and
y, 0.06 on yaw, 0.025 on vx and vy, 0.02 on vyaw.

**Why visual odometry on the car.** The chassis reports wheel distances, but
the driver republishes them as a `Twist` on `/cobraflex/wheel_speeds`, not as
`nav_msgs/Odometry`, and nothing publishes `/odom` on the car. The ZED's visual
odometry is the input that exists — and it does not suffer from skid-steer
wheel slip.

**Frames.** The ZED stamps its odometry with child frame `zed_camera_link` and
has no parameter to change it. `robot_localization` transforms the measurement
into `base_footprint` through the static TF that `robot_state_publisher`
broadcasts from the URDF mount joint — so `robot_state_publisher` must be
running (layer 1) before the EKF produces anything. The ZED's own TF broadcast
is switched off in
[`cobraflex_sensors.launch.xml`](../../src/cobraflex/launch/cobraflex_sensors.launch.xml)
(`publish_tf:=false`) so that the EKF stays the only owner of the edge.

**Who uses it.** SLAM Toolbox, AMCL and Nav2 read the TF; the safety cage reads
`/odometry/filtered` as its only speed source on the car
([08](08_safety_cage.md)). The table of who publishes what in each world is in
[Kinematics §6.3](../../assets/Mathematical%20Model/Kinematics.md).

---

## Pitfalls

- **NaN on the first update.** The ZED reports a pose covariance around
  1e-5. With a prior of order 1.0 (as in the Gazebo file) the hardware filter
  emits NaN on its first update and poisons `/tf`. Keep
  `initial_estimate_covariance` unset in `ekf_hw.yaml`; the comment in the
  file explains it.
- **Two owners of one edge.** If the ZED broadcast or a second EKF also
  publishes `odom → base_footprint`, RViz jumps every cycle.
- **Wrong base frame.** `base_link_frame` must be the URDF root,
  `base_footprint`; any other name gives one link two parents.
- **Sim hides drift.** In Gazebo the odometry is ground truth, so anything
  built on it (SLAM, Nav2) never sees odometric drift. The car does.

---

## Try it

```bash
# Car, with layers 1 and 2 running
ros2 topic hz /zed/zed_node/odom
ros2 topic echo /odometry/filtered --field pose.pose.position
ros2 run tf2_ros tf2_echo odom base_footprint

# Drift experiment: drive a 1 m square by teleop back to the start mark,
# record it, then compare the final pose with (0, 0, 0)
ros2 bag record /odometry/filtered /zed/zed_node/odom /tf /tf_static -o square_run
```

In Gazebo, compare `/odometry/filtered` with `/odom_truth` from the bridge to
see what the filter adds.

---

## Lecture links

- **SLAM 03** — Bayes filter (the two equations above).
- **SLAM 04** — motion and sensor models: why skid-steer odometry drifts in yaw.
- **SLAM 05** — Kalman filter and EKF: the predict/update cycle that
  `robot_localization` runs at 30 Hz.

## Further reading

- `robot_localization` documentation: <https://docs.ros.org/en/humble/p/robot_localization/>
- S. Thrun, W. Burgard, D. Fox, *Probabilistic Robotics*, MIT Press, 2005 — chapters 2 and 3.
