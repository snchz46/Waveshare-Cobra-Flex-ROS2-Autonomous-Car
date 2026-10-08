# 03 · State estimation

> **Summary:** an extended Kalman filter (`robot_localization`) converts
> drifting odometry into the continuous `odom → base_footprint` transform used
> by the rest of the stack. On the robot it fuses the ZED visual odometry; in
> Gazebo it fuses wheel velocities and the IMU.

[← 02 System architecture](02_system_architecture.md) · [Concepts](README.md) · Next: [04 SLAM and mapping →](04_slam_and_mapping.md)

---

## Theory

### Odometry and drift

Odometry integrates motion: wheel encoders, an IMU or a camera (visual
odometry) estimate the displacement since the previous step, and the pose is
the sum of all steps. Each step carries a small error, so the integrated pose
**drifts** without bound. A skid-steer robot drifts fastest in yaw, because it
turns by dragging its wheels sideways
([Kinematics §3.3](../../assets/Mathematical%20Model/Kinematics.md)).

### Bayes filter

State estimation maintains a belief over the state $x_t$ and updates it in two
steps, with control $u_t$ and measurement $z_t$:

$$
\overline{bel}(x_t) = \int p(x_t \mid u_t, x_{t-1})\, bel(x_{t-1})\, dx_{t-1}
\qquad
bel(x_t) = \eta\, p(z_t \mid x_t)\, \overline{bel}(x_t)
$$

The first equation is the **prediction** with the motion model; the second is
the **correction** with the measurement model.

### Kalman filter and EKF

With Gaussian beliefs and linear models, the Bayes filter becomes the Kalman
filter. The **extended** Kalman filter linearises the nonlinear models $f$ and
$h$ around the current estimate (Jacobians $F$, $H$):

$$
\begin{aligned}
\text{predict:}\quad & \hat x^- = f(\hat x, u), && P^- = F P F^\top + Q \\
\text{update:}\quad & K = P^- H^\top (H P^- H^\top + R)^{-1}, && \hat x = \hat x^- + K\,(z - h(\hat x^-)),\quad P = (I - K H)\,P^-
\end{aligned}
$$

$Q$ is the process noise (confidence in the model) and $R$ the measurement
noise (confidence in the sensor). The gain $K$ weighs one against the other.

### `robot_localization`

The `ekf_node` of `robot_localization` estimates a 15-dimensional state:
position $(x, y, z)$, orientation (roll, pitch, yaw), their velocities and the
linear accelerations. Each input has a 15-entry boolean `config` vector that
selects the fused components. `two_d_mode` fixes $z$, roll, pitch and their
rates to zero, as required for a ground robot.

---

## Implementation

Two configurations, one per environment:

| | Robot: [`ekf_hw.yaml`](../../src/cobraflex/config/ekf_hw.yaml) | Gazebo: [`ekf_gazebo.yaml`](../../src/cobraflex/config/ekf_gazebo.yaml) |
| --- | --- | --- |
| Rate | 30 Hz | 30 Hz |
| Inputs | `/zed/zed_node/odom`: **x, y, yaw** (absolute visual-odometry pose) | `odom` (DiffDrive): **vx, vy, vyaw**; `imu`: roll, pitch, yaw, vyaw, ax |
| IMU | Block present, commented out | Fused |
| `publish_tf` | `true`: the EKF publishes `odom → base_footprint` | `false`: the ground-truth `OdometryPublisher` publishes it |
| `initial_estimate_covariance` | Not set | 1.0 on every state |
| Output | `/odometry/filtered` and TF | `/odometry/filtered` only (no consumer; kept for parity with the robot) |

Both configurations use `world_frame: odom`, `base_link_frame: base_footprint`
and `two_d_mode: true`. The hardware process noise $Q$ is diagonal: 0.05 on x
and y, 0.06 on yaw, 0.025 on vx and vy, 0.02 on vyaw.

**Visual odometry on the robot.** The chassis reports wheel distances, but the
driver republishes them as a `Twist` on `/cobraflex/wheel_speeds` and not as
`nav_msgs/Odometry`; no node publishes `/odom` on the robot. The ZED visual
odometry is therefore the available input, and it is not affected by
skid-steer wheel slip.

**Frames.** The ZED stamps its odometry with the child frame
`zed_camera_link`, which cannot be changed by parameter. `robot_localization`
transforms the measurement into `base_footprint` through the static TF that
`robot_state_publisher` broadcasts from the URDF mount joint. The EKF
therefore produces output only once `robot_state_publisher` (layer 1) is
running. The ZED TF broadcast is disabled in
[`cobraflex_sensors.launch.xml`](../../src/cobraflex/launch/cobraflex_sensors.launch.xml)
(`publish_tf:=false`), so the EKF remains the only publisher of the edge.

**Consumers.** SLAM Toolbox, AMCL and Nav2 read the TF; the safety cage reads
`/odometry/filtered` as its only speed source on the robot
([08](08_safety_cage.md)). The publishers per environment are listed in
[Kinematics §6.3](../../assets/Mathematical%20Model/Kinematics.md).

---

## Common errors

- **NaN on the first update.** The ZED reports a pose covariance of about
  1e-5. With a prior of order 1.0 (as in the Gazebo file), the hardware filter
  emits NaN on its first update and corrupts `/tf`.
  `initial_estimate_covariance` remains unset in `ekf_hw.yaml`; the comment in
  the file documents the reason.
- **Multiple publishers of one edge.** If the ZED broadcast or a second EKF
  also publishes `odom → base_footprint`, the robot pose jumps in RViz.
- **Incorrect base frame.** `base_link_frame` must be the URDF root,
  `base_footprint`; any other frame gives one link two parents.
- **No drift in simulation.** In Gazebo the odometry is ground truth, so the
  modules built on it (SLAM, Nav2) never observe odometric drift. On the robot
  they do.

---

## Commands

```bash
# Robot, with layers 1 and 2 running
ros2 topic hz /zed/zed_node/odom
ros2 topic echo /odometry/filtered --field pose.pose.position
ros2 run tf2_ros tf2_echo odom base_footprint

# Drift experiment: drive a 1 m square by teleoperation back to the start mark,
# record it, and compare the final pose with (0, 0, 0)
ros2 bag record /odometry/filtered /zed/zed_node/odom /tf /tf_static -o square_run
```

In Gazebo, compare `/odometry/filtered` with `/odom_truth` from the bridge to
evaluate the contribution of the filter.

---

## Lecture references

- **SLAM 03**: Bayes filter (the two equations above).
- **SLAM 04**: motion and sensor models; yaw drift of skid-steer odometry.
- **SLAM 05**: Kalman filter and EKF; the predict/update cycle that
  `robot_localization` runs at 30 Hz.

## Further reading

- `robot_localization` documentation: <https://docs.ros.org/en/humble/p/robot_localization/>
- S. Thrun, W. Burgard, D. Fox, *Probabilistic Robotics*, MIT Press, 2005, chapters 2 and 3.
