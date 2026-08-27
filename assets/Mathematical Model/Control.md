# Robot Control System

## 1. Control Architecture

Every controller in this repository ends at the same interface: a
`geometry_msgs/Twist` on `/cmd_vel`, carrying `linear.x` and `angular.z` only.
What consumes that Twist depends on which stack is running.

**Simulation:**

```text
Nav2  /  lane_keeper_gazebo_node  /  RL policy
    |
    v  /cmd_vel (Twist)
gz_bridge  ->  gz-sim-diff-drive-system
    |
    v  inverse kinematics, applied directly to the wheel joints
Gazebo physics
    |
    v
/odom (dead reckoning)  +  /odom_truth and TF (ground truth)
```

**Hardware:**

```text
Nav2  /  lane_keeper_node  /  lidar_avoidance_node  /  RL policy
    |
    v  /cmd_vel (Twist)
cobraflex_ros_driver     -- clamp, deadman timeout, 20 Hz keep-alive
    |
    v  JSON frames over serial
ESP32-S3 firmware        -- its own inverse kinematics -> motor RPM
    |
    v
DDSM motor closed loop
```

**There is no PID layer of ours in either path.** In simulation the plugin
applies wheel velocities directly from the differential-drive inverse
kinematics; on hardware the only closed loop is the one inside the motor
controllers, running on the ESP32.

The RL stack inserts one more stage before `/cmd_vel`:

```text
/raw_action  ->  cage_ros_node  ->  /safe_action  ->  vehicle_control_node  ->  /cmd_vel
```

`cage_ros_node` (package `safety_cage`) is a runtime monitor that can override
the policy's command; `vehicle_control_node` is the relay that turns the
arbitrated action into the Twist.

---

## 2. Simulation: the Gazebo `DiffDrive` Plugin

### 2.1 Configuration

**File**: `src/cobraflex/urdf/robot.gazebo` (included by every
`urdf/my_robot_*.urdf`).

```xml
<plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
    <left_joint>front_left_wheel_joint</left_joint>
    <left_joint>rear_left_wheel_joint</left_joint>

    <right_joint>front_right_wheel_joint</right_joint>
    <right_joint>rear_right_wheel_joint</right_joint>

    <wheel_separation>0.154</wheel_separation>
    <wheel_radius>0.03725</wheel_radius>

    <max_linear_acceleration>2.5</max_linear_acceleration>
    <min_linear_acceleration>-2.5</min_linear_acceleration>

    <topic>cmd_vel</topic>

    <odom_topic>odom</odom_topic>
    <!-- Dead-reckoning TF is kept off the ROS /tf topic: the ground-truth
         OdometryPublisher is the sole owner of odom -> base_footprint in
         simulation. -->
    <tf_topic>tf_diffdrive</tf_topic>
    <frame_id>odom</frame_id>
    <child_frame_id>base_footprint</child_frame_id>
</plugin>

<plugin filename="gz-sim-odometry-publisher-system"
        name="gz::sim::systems::OdometryPublisher">
    <odom_frame>odom</odom_frame>
    <robot_base_frame>base_footprint</robot_base_frame>
    <odom_publish_frequency>50</odom_publish_frequency>
    <!-- Ground truth, on its own topic so it does not collide with the
         DiffDrive plugin's dead-reckoning /odom. -->
    <odom_topic>/odom_truth</odom_topic>
    <tf_topic>tf</tf_topic>
    <dimensions>2</dimensions>
</plugin>

<plugin filename="gz-sim-joint-state-publisher-system"
        name="gz::sim::systems::JointStatePublisher">
    <topic>joint_states</topic>
    <joint_name>front_left_wheel_joint</joint_name>
    <joint_name>front_right_wheel_joint</joint_name>
    <joint_name>rear_left_wheel_joint</joint_name>
    <joint_name>rear_right_wheel_joint</joint_name>
</plugin>
```

Two details in that block are load-bearing and easy to undo:

- **`tf_topic` is `tf_diffdrive`, not `tf`.** The `DiffDrive` plugin, the
  `OdometryPublisher` and the EKF all want to broadcast
  `odom -> base_footprint`. All three once reached ROS `/tf` at the same time
  and RViz alternated between them, so the robot model jumped every cycle.
  `ekf_gazebo.yaml` sets `publish_tf: false` for the same reason.
- **`odom_topic` differs between the two plugins.** Both used to publish
  `/odom`, so subscribers saw truth and dead-reckoning samples interleaved —
  and after a `set_pose` teleport the dead-reckoning half does not jump, which
  corrupted the RL pose on every episode reset. RL training reads `/odom_truth`;
  the Nav2 stack keeps the encoder `/odom`.

`robot.gazebo` also holds a commented-out Gazebo Fortress (`ignition-*`) copy of
these plugins with `max_linear_acceleration` 0.53 and `min_linear_acceleration`
−10. Those values are wrong and dead — 0.53 is the chassis's maximum *velocity*
in m/s pasted into an acceleration field. Do not copy from that block.

### 2.2 Plugin Operation

**Input**: `/cmd_vel` (`geometry_msgs/Twist`)

```text
linear.x  : desired linear velocity  [m/s]
angular.z : desired angular velocity [rad/s]
```

`linear.y` is ignored — the model has no lateral degree of freedom.

**Processing**:

1. Read the velocity command from `/cmd_vel`.
2. Apply inverse kinematics (see [Kinematics.md §5](./Kinematics.md)):
   - $\omega_L = \dfrac{v - \omega \cdot W/2}{r}$
   - $\omega_R = \dfrac{v + \omega \cdot W/2}{r}$
3. Rate-limit the linear velocity to ±2.5 m/s². No velocity ceiling and no
   angular acceleration limit are configured on the plugin.
4. Command the resulting angular velocity to all four wheel joints, in
   synchronised left/right pairs.
5. Integrate wheel motion into dead-reckoning odometry.

**Output**:

- `/odom` (`nav_msgs/Odometry`) — dead reckoning
- TF `odom -> base_footprint`, published on `tf_diffdrive`, **not** on `/tf`
- `/joint_states`, from the separate `JointStatePublisher` plugin

---

## 3. Hardware: `cobraflex_ros_driver`

The plugin's counterpart on the real robot. It does no kinematics of its own —
the inverse map runs in the ESP32 firmware (`rosCtrl` in
`Cobra_Driver/movtion_module.h`), through the firmware's own `TRACK_WIDTH` and
`WHEEL_D` constants, which do **not** match the URDF. See
[parameters.md §1.4](./parameters.md).

What the driver does add:

| Behaviour | Parameter | Default | Why |
|---|---|---|---|
| Velocity clamp | `max_linear`, `max_angular` | 0.53 m/s, 6.0 rad/s | Catches any publisher on `/cmd_vel` that ignores the platform limits |
| Keep-alive | — | every 50 ms | Re-sends the last velocity to defeat the firmware's own command timeout |
| Deadman | `cmd_timeout` | 0.5 s | Stops the robot when no `/cmd_vel` has arrived |

**The keep-alive is why the deadman matters.** Because the driver re-sends the
last command every 50 ms, the firmware's timeout never fires, so `cmd_timeout`
is the only thing that stops a physical robot whose commander has died.
`lidar_avoidance_node` carries `scan_timeout` for the same reason. Neither may
be removed, and any new controller publishing `/cmd_vel` has to stay consistent
with them.

---

## 4. Navigation Limits

Nav2 plans well inside the platform's capability. The full comparison and the
resulting wheel speeds are in [Kinematics.md §7](./Kinematics.md):

| Limit | Nav2 / DWB | Driver clamp |
|---|---|---|
| Linear velocity | −0.15 … 0.35 m/s | ±0.53 m/s |
| Angular velocity | ±2.0 rad/s | ±6.0 rad/s |
| Linear acceleration | ±2.5 m/s² | — |
| Angular acceleration | ±3.2 rad/s² | — |

Controller rate is 20 Hz (`controller_frequency`), matched by the
`velocity_smoother` at the same `smoothing_frequency`.

---

## 5. Cross-References

- [Kinematics.md](./Kinematics.md) — the equations the plugin and the firmware
  both implement, and where skid-steer departs from them
- [parameters.md](./parameters.md) — geometry, limits, and the firmware-constant
  disagreement
- Source: `src/cobraflex/urdf/robot.gazebo`,
  `src/cobraflex/config/nav2_params.yaml`,
  `src/cobraflex/cobraflex/cobraflex_ros_driver.py`

---

## 6. Credits and References

The structure of this documentation set is adapted from
**[Axioma_robot](https://github.com/MrDavidAlv/Axioma_robot)** by
[MrDavidAlv](https://github.com/MrDavidAlv) (BSD licence) — a ROS 2 Humble
skid-steer robot running SLAM Toolbox and Nav2, and the origin of the idea for
this project. See [README.md § Credits](./README.md).

**External documentation**:

- [Gazebo `DiffDrive` system](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DiffDrive.html)
- [Gazebo `OdometryPublisher` system](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1OdometryPublisher.html)
- [Nav2 DWB controller](https://docs.nav2.org/configuration/packages/configuring-dwb-controller.html)
