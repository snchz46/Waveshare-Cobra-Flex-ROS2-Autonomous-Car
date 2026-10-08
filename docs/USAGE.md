# Usage

Operation of the Cobra Flex stack in simulation and on the physical robot.

Prerequisite: a built and sourced workspace ([INSTALLATION.md](INSTALLATION.md)).
Each command runs in its own terminal with
`source ~/ros2_ws/install/setup.bash`.

---

## Contents

- [1. SLAM and mapping](#1-slam-and-mapping)
- [2. Autonomous navigation](#2-autonomous-navigation)
- [3. Lane keeping](#3-lane-keeping)
- [4. Obstacle avoidance](#4-obstacle-avoidance)
- [5. Physical robot](#5-physical-robot)
- [6. World selection](#6-world-selection)
- [7. Inspection and debugging](#7-inspection-and-debugging)

---

## 1. SLAM and mapping

```bash
# Terminal 1: simulation (obstacles.world)
ros2 launch cobraflex gazebo.launch.py

# Terminal 2: SLAM Toolbox + RViz
ros2 launch cobraflex mapping.launch.py

# Terminal 3: manual driving for exploration
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

After the environment has been explored, the map is saved in the `maps/`
directory of the package:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/cobraflex/maps/cobraflex_map
colcon build --packages-select cobraflex --symlink-install
```

The rebuild is required: `navigation.launch.py` reads the map from the install
share, not from the source tree. Maps are excluded from git; see
[`src/cobraflex/maps/README.md`](../src/cobraflex/maps/README.md).

**Mapping parameters** are defined in
[`config/slam_toolbox_mapping.yaml`](../src/cobraflex/config/slam_toolbox_mapping.yaml)
(simulation) and `slam_toolbox_mapping_hw.yaml` (hardware). The two profiles
differ: the hardware profile adds keyframes five times more often and disables
loop closure. See [parameters.md §5.1](../assets/Mathematical%20Model/parameters.md).

---

## 2. Autonomous navigation

Prerequisite: a saved map.

```bash
# Terminal 1
ros2 launch cobraflex gazebo.launch.py

# Terminal 2: Nav2 + AMCL + RViz
ros2 launch cobraflex navigation.launch.py
```

In RViz:

1. **2D Pose Estimate**: initial pose of the robot for AMCL convergence.
2. **2D Goal Pose**: navigation goal.
3. Global and local costmaps, planned path and AMCL particle cloud are
   displayed.

Alternative map or parameter file:

```bash
ros2 launch cobraflex navigation.launch.py \
  map:=/path/to/map.yaml \
  params_file:=/path/to/params.yaml
```

Nav2 uses [`config/nav2_params.yaml`](../src/cobraflex/config/nav2_params.yaml),
configured for the robot footprint (0.228 × 0.180 m, circumscribed radius
0.145 m) and for `base_footprint`. The planning envelope is 0.35 m/s and
2.0 rad/s, within the platform clamp of 0.53 m/s and 6.0 rad/s.

---

## 3. Lane keeping

Lane keeping uses its own world and controller, independent of the SLAM/Nav2
stack.

```bash
ros2 launch cobraflex lane_keeper_gazebo.launch.py
```

The launch file starts a lane-following world and `lane_keeper_gazebo_node`: a
calibrated CV lane estimator with pure-pursuit steering. The estimator is
implemented in `cobraflex_rl.cv_lane_controller` and shared with the scored RL
evaluation, so both execute identical code.

On hardware the equivalent is `lane_keeper_node`, a classical histogram tracker
on the Jetson CSI camera.

---

## 4. Obstacle avoidance

Reactive controller without map or planner:

```bash
ros2 launch cobraflex gazebo.launch.py
ros2 run cobraflex lidar_avoidance_node
```

The node subscribes to `/scan` and publishes `/cmd_vel`. Its `scan_timeout`
stops the robot when no scans arrive; this timer must not be removed.

---

## 5. Physical robot

```bash
# 1. Description + serial driver
ros2 launch cobraflex cobraflex_bringup.launch.xml

# 2. Sensors: LiDAR + ZED + CSI camera + EKF
ros2 launch cobraflex cobraflex_sensors.launch.xml

# 3. SLAM with real scans
ros2 launch cobraflex cobraflex_mapping.launch.py
```

Differences from simulation:

**TF ownership.** On hardware the EKF publishes `odom -> base_footprint`
(`ekf_hw.yaml`, `publish_tf: true`). The TF broadcast of the ZED wrapper is
disabled with `publish_tf:=false` in the sensors launch file.

**Deadman timer.** `cobraflex_ros_driver` re-sends the last velocity every
50 ms, which overrides the firmware timeout. `cmd_timeout` (default 0.5 s) is
therefore the only mechanism that stops the physical robot when its controller
fails. Every new `/cmd_vel` publisher must be consistent with it.

> The firmware converts every twist with its own `TRACK_WIDTH` = 0.159 and
> `WHEEL_D` = 0.0739, whereas the URDF and Gazebo use 0.154 and 0.0745. This
> results in a known, unresolved yaw gain difference of about 3–4 % between
> simulation and hardware. See [parameters.md §1.4](../assets/Mathematical%20Model/parameters.md).

---

## 6. World selection

```bash
ros2 launch cobraflex gazebo_mesh.launch.py world:=oval_complex gui:=true
```

A bare token resolves to `worlds/lane_following_<token>.world`;
`world:=oval_complex` and `world:=lane_following_oval_complex` are equivalent.
A value that contains a path separator or ends in `.world` / `.sdf` is passed
through unchanged.

| World | Purpose |
| --- | --- |
| `obstacles.world` | Default for `gazebo.launch.py`; SLAM and Nav2 |
| `oval_simple` | Low-curvature circuit; lane-keeping baseline |
| `oval_complex` | Default lane-following circuit (`complex_b`) |
| `complex_b_flipH` / `flipV` | Mirrored circuit; steering-bias test |
| `complex_b_worn_25/50/75` | Worn markings at three levels: `worn_25` strongest, `worn_75` mildest |
| `complex_b_gaps` | Lane-line dropouts |
| `complex_b_particles` | Debris and stains on the road |
| `straight_road.world` | Controller step responses |
| `empty.world` | Ground plane only; URDF bring-up |

Complete list and texture-generation scripts:
[`src/cobraflex/worlds/README.md`](../src/cobraflex/worlds/README.md).

---

## 7. Inspection and debugging

```bash
# Running nodes and parameters
ros2 node list
ros2 topic list
ros2 node info /slam_toolbox
ros2 param list /controller_server

# Topic rates
ros2 topic hz /scan            # approx. 10 Hz
ros2 topic hz /odom            # approx. 50 Hz
ros2 topic echo /cmd_vel

# TF
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_tools view_frames        # writes frames.pdf

# Session recording for offline analysis
ros2 bag record -a -o navigation_data
```

**World validation before launch:**

```bash
gz sdf -k src/cobraflex/worlds/lane_following_oval_complex.world
```

**Linters** (`ament_copyright`, `ament_flake8`, `ament_pep257`):

```bash
colcon test --packages-select cobraflex
colcon test-result --verbose
```

---

## Further documentation

- [INSTALLATION.md](INSTALLATION.md): setup from a clean machine
- [Mathematical Model](../assets/Mathematical%20Model/README.md): kinematics, control, parameters
- [`src/cobraflex/config/`](../src/cobraflex/config/): all tunable parameters, with rationale in comments
