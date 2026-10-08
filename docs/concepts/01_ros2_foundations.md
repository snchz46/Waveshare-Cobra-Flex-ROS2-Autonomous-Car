# 01 · ROS 2 foundations

> **Summary:** the robot is a set of ROS 2 nodes that exchange typed messages.
> Every controller, simulated or physical, classical or learned, publishes the
> same `geometry_msgs/Twist` on `/cmd_vel`.

[← Concepts](README.md) · Next: [02 System architecture →](02_system_architecture.md)

---

## Theory

| Element | Definition | Example in this stack |
| --- | --- | --- |
| **Node** | Process or component with a single responsibility | `cobraflex_ros_driver`, `ekf_filter_node`, `slam_toolbox` |
| **Topic** | Typed many-to-many publish/subscribe stream | `/scan` (`sensor_msgs/LaserScan`), `/cmd_vel` (`geometry_msgs/Twist`) |
| **Service** | Request/response call | `/slam_toolbox/serialize_map`, which stores the pose graph for later resumption |
| **Action** | Long-running goal with feedback and cancellation | Nav2 `NavigateToPose` |
| **Parameter** | Node configuration, usually loaded from YAML | `max_vel_x: 0.35` in `nav2_params.yaml` |
| **Launch file** | Starts and connects a set of nodes | `cobraflex_sensors.launch.xml` |

ROS 2 is built on **DDS**. Nodes discover each other by multicast within a
**domain** (`ROS_DOMAIN_ID`); nodes in different domains are not visible to
each other. Each publisher/subscriber pair negotiates a **QoS profile**:

- *Reliability*: `RELIABLE` retransmits lost samples; `BEST_EFFORT` discards
  them. Sensor streams use best effort (`qos_profile_sensor_data`), because a
  retransmitted scan arrives too late to be useful and delays the next one.
- *History/depth*: number of queued samples. Debug images use depth 1.
- *Durability*: `TRANSIENT_LOCAL` retains the last sample for late subscribers
  (used for `/map` and `/robot_description`).

A best-effort subscriber can receive from a reliable publisher; the reverse
combination is incompatible. An incompatible pair appears as a topic that
exists but delivers no messages.

**TF2** maintains a tree of coordinate frames. Each edge has exactly one
publisher: static edges come from the URDF through `robot_state_publisher`,
dynamic edges from estimators. The mobile-robot convention
([REP-105](https://www.ros.org/reps/rep-0105.html)) is
`map → odom → base_footprint`: `odom` is continuous but drifts; `map` is
globally consistent but discontinuous.

**Time.** On the robot every node uses wall time. In Gazebo, nodes run with
`use_sim_time:=true` and read `/clock`. Mixing both time sources produces TF
"extrapolation into the future" errors.

---

## Implementation

### Packages

| Package | Build type | Role |
| --- | --- | --- |
| `cobraflex` | ament_python | Driver, URDF, Gazebo worlds, SLAM, Nav2, classical controllers |
| `cobraflex_rl` | ament_python | RL lane-following agent, CV lane estimator, CSI camera node, evaluation tools |
| `safety_cage` | ament_python | ROS wrapper of the runtime safety monitor |
| `cobraflex_safety_msgs` | ament_cmake | `CageStatus.msg` |
| `cobraflex_teleop_gui` | ament_python | Qt window for manual driving |

`cobraflex` provides exactly four `ros2 run` executables:
`cobraflex_ros_driver`, `lidar_avoidance_node`, `lane_keeper_node` and
`lane_keeper_gazebo_node`.

### Launch layers on the robot

The hardware launch files are layered so that each layer can be restarted
independently ([`deploy_cobraflex.launch.py`](../../src/cobraflex_rl/launch/deploy_cobraflex.launch.py)):

| Layer | Launch file | Starts |
| --- | --- | --- |
| 1 · Bring-up | [`cobraflex_bringup.launch.xml`](../../src/cobraflex/launch/cobraflex_bringup.launch.xml) | `robot_state_publisher`, `joint_state_publisher`, serial driver |
| 2 · Sensors | [`cobraflex_sensors.launch.xml`](../../src/cobraflex/launch/cobraflex_sensors.launch.xml) | RPLIDAR (`sllidar_ros2`), ZED Mini (`zed_wrapper`), CSI lane camera, EKF |
| 3 · Controller | [`cobraflex_lane_keeper.launch.py`](../../src/cobraflex/launch/cobraflex_lane_keeper.launch.py), [`cobraflex_automatic.launch.xml`](../../src/cobraflex/launch/cobraflex_automatic.launch.xml), [`cobraflex_mapping.launch.py`](../../src/cobraflex/launch/cobraflex_mapping.launch.py), [`deploy_cobraflex.launch.py`](../../src/cobraflex_rl/launch/deploy_cobraflex.launch.py) | Classical lane keeper, LiDAR avoidance, SLAM, RL policy behind the safety cage |

In Gazebo, [`gazebo.launch.py`](../../src/cobraflex/launch/gazebo.launch.py)
replaces layers 1 and 2: it spawns the robot and starts the `ros_gz_bridge` and
an EKF equivalent to the hardware configuration.

### `/cmd_vel` interface

Every controller publishes only `linear.x` (m/s) and `angular.z` (rad/s).

| Consumer | Location | Function |
| --- | --- | --- |
| Gazebo `DiffDrive` plugin | `urdf/robot.gazebo` | Wheel kinematics, acceleration limits |
| `cobraflex_ros_driver` | [`cobraflex_ros_driver.py`](../../src/cobraflex/cobraflex/cobraflex_ros_driver.py) | Clamp to `max_linear` 0.53 m/s and `max_angular` 6.0 rad/s, re-send every 50 ms, stop after `cmd_timeout` 0.5 s without a new command |

The periodic re-send overrides the firmware timeout. Consequently,
`cmd_timeout` is the only mechanism that stops the physical robot when its
controller fails. Details: [Control](../../assets/Mathematical%20Model/Control.md) §3.

### TF frames

`base_footprint` is the root of the robot description. All consumers use it:
`ekf_*.yaml` `base_link_frame`, SLAM `base_frame`, Nav2 `robot_base_frame` and
the RViz fixed frame. The complete tree is shown in the
[root README](../../README.md#transform-tree).

| Edge | Publisher (Gazebo) | Publisher (robot) |
| --- | --- | --- |
| `map → odom` | SLAM Toolbox or AMCL | SLAM Toolbox or AMCL |
| `odom → base_footprint` | Ground-truth `OdometryPublisher` plugin | EKF ([03](03_state_estimation.md)) |
| `base_footprint → sensors` | `robot_state_publisher` (URDF) | `robot_state_publisher` (URDF) |

### Gazebo bridge

[`gz_bridge.yaml`](../../src/cobraflex/config/gz_bridge.yaml) maps Gazebo
topics to ROS: `clock`, `joint_states`, `odom`, `odom_truth`, `tf`, `imu`,
`scan`, the stereo camera topics and `camera/image_raw_lane` from Gazebo to
ROS, and `cmd_vel` from ROS to Gazebo.

---

## Common errors

- **Multiple controllers.** Two layer-3 controllers publishing on `/cmd_vel`
  produce conflicting commands. Run exactly one.
- **Multiple drivers.** Linux does not lock `/dev/ttyACM*`; a second
  `cobraflex_ros_driver` interleaves writes on the same port without warning.
- **Multiple TF publishers.** More than one publisher of
  `odom → base_footprint` makes the robot pose jump in RViz. Verify the owner
  of an edge before adding a node that broadcasts TF.
- **Simulation time.** A node started without `use_sim_time:=true` alongside
  Gazebo stamps messages with wall time, and TF lookups fail.
- **Shutdown.** Under `ros2 launch`, Ctrl-C arrives as
  `ExternalShutdownException`. Nodes catch it together with
  `KeyboardInterrupt` and guard `rclpy.shutdown()` with `if rclpy.ok()`.

---

## Commands

```bash
# Gazebo: robot, bridge, EKF and RViz
ros2 launch cobraflex gazebo.launch.py

# Second terminal: nodes, topics and connections
ros2 node list
ros2 topic list
ros2 topic info /cmd_vel --verbose      # publishers, subscribers, QoS
ros2 topic hz /scan
rqt_graph
ros2 run tf2_tools view_frames          # writes frames.pdf

# Keyboard driving
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

On the robot, start layers 1 and 2 as described in [Usage](../USAGE.md) §5 and
run the same inspection commands from the lab PC with the same
`ROS_DOMAIN_ID` ([09](09_multi_machine_networking.md)).

---

## Lecture references

- **ADAS (SE4ADS)**: ROS introduction and the demonstrations of the midterm
  presentations; chapter 3 for the mapping of nodes onto an ADS architecture
  ([02](02_system_architecture.md)).

## Further reading

- ROS 2 Humble concepts: <https://docs.ros.org/en/humble/Concepts.html>
- QoS settings: <https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html>
- REP-105, coordinate frames for mobile platforms: <https://www.ros.org/reps/rep-0105.html>
