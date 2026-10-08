# Robot control system

## 1. Control architecture

All controllers in this repository publish to the same interface: a
`geometry_msgs/Twist` on `/cmd_vel` with `linear.x` and `angular.z` only. The
consumer of the Twist depends on the active stack.

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
ESP32-S3 firmware        -- inverse kinematics -> motor RPM
    |
    v
DDSM motor closed loop
```

**Neither path contains a PID layer of this repository.** In simulation the
plugin applies wheel velocities directly from the differential-drive inverse
kinematics; on hardware the only closed loop is the motor control running on
the ESP32.

The RL stack adds one stage before `/cmd_vel`:

```text
/raw_action  ->  cage_ros_node  ->  /safe_action  ->  vehicle_control_node  ->  /cmd_vel
```

`cage_ros_node` (package `safety_cage`) is a runtime monitor that can override
the policy command; `vehicle_control_node` converts the arbitrated action into
the Twist.

---

## 2. Simulation: Gazebo `DiffDrive` plugin

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
    <!-- Dead-reckoning TF on a separate topic: the ground-truth
         OdometryPublisher is the only publisher of odom -> base_footprint
         in simulation. The DiffDrive plugin, the OdometryPublisher and
         the EKF can all publish this transform; with more than one on
         ROS tf, the robot model jumps in RViz. This plugin drives the
         wheels and publishes the encoder odometry. -->
    <tf_topic>tf_diffdrive</tf_topic>
    <frame_id>odom</frame_id>
    <child_frame_id>base_footprint</child_frame_id>
</plugin>

<plugin filename="gz-sim-odometry-publisher-system"
        name="gz::sim::systems::OdometryPublisher">
    <odom_frame>odom</odom_frame>
    <robot_base_frame>base_footprint</robot_base_frame>
    <odom_publish_frequency>50</odom_publish_frequency>
    <!-- Ground truth on a separate topic, distinct from the
         dead-reckoning /odom of the DiffDrive plugin. With a shared
         /odom, subscribers receive interleaved ground-truth and
         dead-reckoning samples; after a set_pose teleport the
         dead-reckoning samples do not jump, which corrupts the RL pose
         at every episode reset. RL training reads /odom_truth; the F2
         stack uses the encoder /odom. -->
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

Two settings in this block are essential:

- **`tf_topic` is `tf_diffdrive`, not `tf`.** The `DiffDrive` plugin, the
  `OdometryPublisher` and the EKF can all broadcast `odom -> base_footprint`.
  With more than one of them on ROS `/tf`, RViz alternates between the
  transforms and the robot model jumps every cycle. `ekf_gazebo.yaml` sets
  `publish_tf: false` for the same reason.
- **`odom_topic` differs between the two plugins.** With a shared `/odom`,
  subscribers receive ground-truth and dead-reckoning samples interleaved.
  After a `set_pose` teleport the dead-reckoning samples do not jump, which
  corrupts the RL pose at every episode reset. RL training reads `/odom_truth`;
  the Nav2 stack uses the encoder `/odom`.

`robot.gazebo` also contains a commented-out Gazebo Fortress (`ignition-*`)
copy of these plugins with `max_linear_acceleration` 0.53 and
`min_linear_acceleration` −10. These values are incorrect and unused: 0.53 is
the maximum chassis *velocity* in m/s entered in an acceleration field. The
block is not a valid reference.

### 2.2 Plugin operation

**Input**: `/cmd_vel` (`geometry_msgs/Twist`)

```text
linear.x  : desired linear velocity  [m/s]
angular.z : desired angular velocity [rad/s]
```

`linear.y` is ignored; the model has no lateral degree of freedom.

**Processing**:

1. Read the velocity command from `/cmd_vel`.
2. Apply inverse kinematics (see [Kinematics.md §5](./Kinematics.md)):
   - $\omega_L = \dfrac{v - \omega \cdot W/2}{r}$
   - $\omega_R = \dfrac{v + \omega \cdot W/2}{r}$
3. Rate-limit the linear velocity to ±2.5 m/s². No velocity ceiling and no
   angular acceleration limit are configured in the plugin.
4. Command the resulting angular velocity to all four wheel joints in
   synchronised left/right pairs.
5. Integrate the wheel motion into dead-reckoning odometry.

**Output**:

- `/odom` (`nav_msgs/Odometry`): dead reckoning
- TF `odom -> base_footprint`, published on `tf_diffdrive`, not on `/tf`
- `/joint_states`, from the separate `JointStatePublisher` plugin

---

## 3. Hardware: `cobraflex_ros_driver`

Counterpart of the plugin on the physical robot. The driver performs no
kinematics: the inverse mapping runs in the ESP32 firmware (`rosCtrl` in
`Cobra_Driver/movtion_module.h`) with the firmware constants `TRACK_WIDTH` and
`WHEEL_D`, which differ from the URDF. See [parameters.md §1.4](./parameters.md).

Functions of the driver:

| Function | Parameter | Default | Purpose |
|---|---|---|---|
| Velocity clamp | `max_linear`, `max_angular` | 0.53 m/s, 6.0 rad/s | Limits any `/cmd_vel` publisher to the platform limits |
| Keep-alive | — | Every 50 ms | Re-sends the last velocity to override the firmware command timeout |
| Deadman | `cmd_timeout` | 0.5 s | Stops the robot when no `/cmd_vel` arrives |

**Relation between keep-alive and deadman.** Because the driver re-sends the
last command every 50 ms, the firmware timeout never triggers;
`cmd_timeout` is therefore the only mechanism that stops the physical robot
when its controller fails. `lidar_avoidance_node` has `scan_timeout` for the
same purpose. Neither timer may be removed, and every new controller that
publishes `/cmd_vel` must be consistent with them.

---

## 4. Navigation limits

Nav2 plans well within the platform capability. The full comparison and the
resulting wheel speeds are given in [Kinematics.md §7](./Kinematics.md):

| Limit | Nav2 / DWB | Driver clamp |
|---|---|---|
| Linear velocity | −0.15 … 0.35 m/s | ±0.53 m/s |
| Angular velocity | ±2.0 rad/s | ±6.0 rad/s |
| Linear acceleration | ±2.5 m/s² | — |
| Angular acceleration | ±3.2 rad/s² | — |

The controller rate is 20 Hz (`controller_frequency`), equal to the
`smoothing_frequency` of the `velocity_smoother`.

---

## 5. Cross-references

- [Kinematics.md](./Kinematics.md): equations implemented by the plugin and the
  firmware, and skid-steer deviations
- [parameters.md](./parameters.md): geometry, limits and the firmware-constant
  discrepancy
- Source files: `src/cobraflex/urdf/robot.gazebo`,
  `src/cobraflex/config/nav2_params.yaml`,
  `src/cobraflex/cobraflex/cobraflex_ros_driver.py`

---

## 6. Credits and references

The structure of this documentation is adapted from
**[Axioma_robot](https://github.com/MrDavidAlv/Axioma_robot)** by
[MrDavidAlv](https://github.com/MrDavidAlv) (BSD licence), a ROS 2 Humble
skid-steer robot with SLAM Toolbox and Nav2 and the origin of the idea for this
project. See [README.md § Credits](./README.md).

**External documentation**:

- [Gazebo `DiffDrive` system](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DiffDrive.html)
- [Gazebo `OdometryPublisher` system](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1OdometryPublisher.html)
- [Nav2 DWB controller](https://docs.nav2.org/configuration/packages/configuring-dwb-controller.html)
