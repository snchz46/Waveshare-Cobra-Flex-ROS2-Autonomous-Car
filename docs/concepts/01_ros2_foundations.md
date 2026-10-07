# 01 · ROS 2 foundations

> **In one sentence:** the robot is a set of ROS 2 nodes exchanging typed
> messages, and every controller — simulated or real, classical or learned —
> ends in the same `geometry_msgs/Twist` on `/cmd_vel`.

[← Concepts](README.md) · Next: [02 System architecture →](02_system_architecture.md)

---

## Concept

| Building block | What it is | Example in this stack |
| --- | --- | --- |
| **Node** | One process (or component) with one job | `cobraflex_ros_driver`, `ekf_filter_node`, `slam_toolbox` |
| **Topic** | Typed, many-to-many publish/subscribe stream | `/scan` (`sensor_msgs/LaserScan`), `/cmd_vel` (`geometry_msgs/Twist`) |
| **Service** | Request/response call | SLAM Toolbox's `/slam_toolbox/serialize_map`, which saves the pose graph to resume mapping later |
| **Action** | Long-running goal with feedback and cancel | Nav2 `NavigateToPose` |
| **Parameter** | Per-node configuration, usually loaded from YAML | `max_vel_x: 0.35` in `nav2_params.yaml` |
| **Launch file** | Starts and wires a set of nodes | `cobraflex_sensors.launch.xml` |

Underneath, ROS 2 uses **DDS**. Nodes find each other by multicast discovery
inside a **domain** (`ROS_DOMAIN_ID`); nodes in different domains never see each
other. Each publisher/subscriber pair agrees on a **QoS profile**:

- *Reliability* — `RELIABLE` resends lost samples; `BEST_EFFORT` drops them.
  Sensor streams use best effort (`qos_profile_sensor_data`): a late laser scan
  is worthless, and resending it only delays the next one.
- *History/depth* — how many samples are queued. Debug images use depth 1.
- *Durability* — `TRANSIENT_LOCAL` keeps the last sample for late joiners (used
  for `/map` and `/robot_description`).

A best-effort subscriber can read a reliable publisher, but not the other way
round — a mismatch shows up as a topic that exists but never delivers.

**TF2** keeps a tree of coordinate frames. Each edge has exactly **one**
publisher; static edges come from the URDF via `robot_state_publisher`, dynamic
ones from estimators. The mobile-robot convention ([REP-105](https://www.ros.org/reps/rep-0105.html)) is
`map → odom → base_footprint`: `odom` is smooth but drifts, `map` is globally
correct but jumps.

**Time.** On the car every node uses wall time. In Gazebo, nodes must run with
`use_sim_time:=true` and read `/clock`; mixing the two produces "extrapolation
into the future" TF errors.

---

## In this repository

### Packages

| Package | Build type | Role |
| --- | --- | --- |
| `cobraflex` | ament_python | Driver, URDF, Gazebo worlds, SLAM, Nav2, classical controllers |
| `cobraflex_rl` | ament_python | RL lane-following agent, CV lane estimator, CSI camera node, evaluation tools |
| `safety_cage` | ament_python | ROS wrapper of the runtime safety monitor |
| `cobraflex_safety_msgs` | ament_cmake | `CageStatus.msg` |
| `cobraflex_teleop_gui` | ament_python | Qt window for manual driving |

`cobraflex` has exactly four `ros2 run` executables: `cobraflex_ros_driver`,
`lidar_avoidance_node`, `lane_keeper_node` and `lane_keeper_gazebo_node`. A
document that names another one is stale.

### Launch layers on the car

The hardware launch files are layered so that each layer can be restarted
without the others ([`deploy_cobraflex.launch.py`](../../src/cobraflex_rl/launch/deploy_cobraflex.launch.py) docstring):

| Layer | Launch file | Starts |
| --- | --- | --- |
| 1 · Bring-up | [`cobraflex_bringup.launch.xml`](../../src/cobraflex/launch/cobraflex_bringup.launch.xml) | `robot_state_publisher`, `joint_state_publisher`, serial driver |
| 2 · Sensors | [`cobraflex_sensors.launch.xml`](../../src/cobraflex/launch/cobraflex_sensors.launch.xml) | RPLIDAR (`sllidar_ros2`), ZED Mini (`zed_wrapper`), CSI lane camera, EKF |
| 3 · One controller | [`cobraflex_lane_keeper.launch.py`](../../src/cobraflex/launch/cobraflex_lane_keeper.launch.py), [`cobraflex_automatic.launch.xml`](../../src/cobraflex/launch/cobraflex_automatic.launch.xml), [`cobraflex_mapping.launch.py`](../../src/cobraflex/launch/cobraflex_mapping.launch.py), [`deploy_cobraflex.launch.py`](../../src/cobraflex_rl/launch/deploy_cobraflex.launch.py) | Classical lane keeper, LiDAR avoidance, SLAM, RL policy behind the cage |

In Gazebo, [`gazebo.launch.py`](../../src/cobraflex/launch/gazebo.launch.py)
replaces layers 1 and 2: it spawns the robot, starts the `ros_gz_bridge` and an
EKF for parity with the car.

### The `/cmd_vel` contract

Every controller publishes `linear.x` (m/s) and `angular.z` (rad/s) only.

| Consumer | Where | What it adds |
| --- | --- | --- |
| Gazebo `DiffDrive` plugin | `urdf/robot.gazebo` | Wheel kinematics, acceleration limits |
| `cobraflex_ros_driver` | [`cobraflex_ros_driver.py`](../../src/cobraflex/cobraflex/cobraflex_ros_driver.py) | Clamp to `max_linear` 0.53 m/s and `max_angular` 6.0 rad/s, re-send every 50 ms, stop after `cmd_timeout` 0.5 s without a new command |

The re-send defeats the firmware's own timeout, so `cmd_timeout` is the only
thing that stops a physical robot whose commander died. Details:
[Control](../../assets/Mathematical%20Model/Control.md) §3.

### TF frames

`base_footprint` is the root of the robot description; every consumer must use
it (`ekf_*.yaml` `base_link_frame`, SLAM `base_frame`, Nav2
`robot_base_frame`, the RViz fixed frame). The full tree is drawn in the
[root README](../../README.md#transform-tree).

| Edge | Published by (Gazebo) | Published by (car) |
| --- | --- | --- |
| `map → odom` | SLAM Toolbox or AMCL | SLAM Toolbox or AMCL |
| `odom → base_footprint` | Ground-truth `OdometryPublisher` plugin | EKF ([03](03_state_estimation.md)) |
| `base_footprint → sensors` | `robot_state_publisher` (URDF) | `robot_state_publisher` (URDF) |

### Gazebo bridge

[`gz_bridge.yaml`](../../src/cobraflex/config/gz_bridge.yaml) maps Gazebo
topics to ROS: `clock`, `joint_states`, `odom`, `odom_truth`, `tf`, `imu`,
`scan`, the stereo camera topics and `camera/image_raw_lane` from Gazebo to ROS,
and `cmd_vel` from ROS to Gazebo.

---

## Pitfalls

- **Two commanders.** Two Layer-3 controllers both publishing `/cmd_vel` fight
  each other. Run exactly one.
- **Two drivers.** Linux does not lock `/dev/ttyACM*`; a second
  `cobraflex_ros_driver` silently interleaves writes on the same port.
- **Two TF publishers.** Three publishers once fought over
  `odom → base_footprint` and RViz jumped every cycle. Check the owner before
  adding a node that broadcasts TF.
- **Sim time.** A node launched without `use_sim_time:=true` next to Gazebo
  stamps messages with wall time and TF lookups fail.
- **Shutdown.** Under `ros2 launch`, Ctrl-C arrives as
  `ExternalShutdownException`. Nodes catch it together with
  `KeyboardInterrupt` and guard `rclpy.shutdown()` with `if rclpy.ok()`.

---

## Try it

```bash
# Gazebo: robot, bridge, EKF and RViz
ros2 launch cobraflex gazebo.launch.py

# In a second terminal: what is running and who talks to whom
ros2 node list
ros2 topic list
ros2 topic info /cmd_vel --verbose      # publishers, subscribers, QoS
ros2 topic hz /scan
rqt_graph
ros2 run tf2_tools view_frames          # writes frames.pdf

# Drive by keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

On the car, start layers 1 and 2 as in [Usage](../USAGE.md) §5 and run the same
inspection commands from the lab PC (same `ROS_DOMAIN_ID`, see
[09](09_multi_machine_networking.md)).

---

## Lecture links

- **ADAS (SE4ADS)** — the ROS introduction and demos from the midterm
  presentations; chapter 3 for how nodes map onto an ADS architecture
  ([02](02_system_architecture.md)).

## Further reading

- ROS 2 Humble concepts: <https://docs.ros.org/en/humble/Concepts.html>
- QoS settings: <https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html>
- REP-105, coordinate frames for mobile platforms: <https://www.ros.org/reps/rep-0105.html>
