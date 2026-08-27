# Usage

Running the Cobra Flex stack — in simulation and on the physical robot.

Assumes a built and sourced workspace ([INSTALLATION.md](INSTALLATION.md)).
Every command below wants its own terminal, each with
`source ~/ros2_ws/install/setup.bash`.

---

## Contents

- [1. SLAM — building a map](#1-slam--building-a-map)
- [2. Autonomous navigation](#2-autonomous-navigation)
- [3. Lane keeping](#3-lane-keeping)
- [4. Obstacle avoidance](#4-obstacle-avoidance)
- [5. On the physical robot](#5-on-the-physical-robot)
- [6. Choosing a world](#6-choosing-a-world)
- [7. Inspection and debugging](#7-inspection-and-debugging)

---

## 1. SLAM — building a map

```bash
# Terminal 1 — simulation (obstacles.world)
ros2 launch cobraflex gazebo.launch.py

# Terminal 2 — SLAM Toolbox + RViz
ros2 launch cobraflex mapping.launch.py

# Terminal 3 — drive the robot around to explore
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Once the environment is fully explored, save the map **into the package's own
`maps/` directory**:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/cobraflex/maps/cobraflex_map
colcon build --packages-select cobraflex --symlink-install
```

The rebuild is not optional: `navigation.launch.py` reads the map from the
install share, not from the source tree. Maps are gitignored — see
[`src/cobraflex/maps/README.md`](../src/cobraflex/maps/README.md).

**Mapping parameters** live in
[`config/slam_toolbox_mapping.yaml`](../src/cobraflex/config/slam_toolbox_mapping.yaml)
(simulation) and `slam_toolbox_mapping_hw.yaml` (hardware). The two are not the
same profile: the hardware one takes keyframes five times more often and runs
with **loop closure off**. See
[parameters.md §5.1](../assets/Mathematical%20Model/parameters.md).

---

## 2. Autonomous navigation

Needs a previously saved map.

```bash
# Terminal 1
ros2 launch cobraflex gazebo.launch.py

# Terminal 2 — Nav2 + AMCL + RViz
ros2 launch cobraflex navigation.launch.py
```

In RViz:

1. **2D Pose Estimate** — set the robot's initial pose, so AMCL can converge
2. **2D Goal Pose** — send a navigation goal
3. Watch the global/local costmaps, the planned path and the AMCL particle cloud

Pointing it at a different map or parameter file:

```bash
ros2 launch cobraflex navigation.launch.py \
  map:=/path/to/map.yaml \
  params_file:=/path/to/params.yaml
```

Nav2 runs on [`config/nav2_params.yaml`](../src/cobraflex/config/nav2_params.yaml),
tuned to this robot's real footprint (0.228 × 0.180 m, circumscribed radius
0.145 m) and to `base_footprint`. The planning envelope is 0.35 m/s and
2.0 rad/s — well inside the platform's own 0.53 m/s / 6.0 rad/s clamp.

---

## 3. Lane keeping

A separate world and a separate controller from the SLAM/Nav2 stack.

```bash
ros2 launch cobraflex lane_keeper_gazebo.launch.py
```

This brings up its own lane-following world and runs `lane_keeper_gazebo_node`
— a calibrated CV lane estimator feeding pure-pursuit steering. The estimator
itself lives in `cobraflex_rl.cv_lane_controller`, deliberately shared so the
deployed controller and the scored RL evaluation run identical code.

On hardware the equivalent is `lane_keeper_node`, a classical histogram tracker
reading the Jetson CSI camera.

---

## 4. Obstacle avoidance

A standalone reactive controller — no map, no planner:

```bash
ros2 launch cobraflex gazebo.launch.py
ros2 run cobraflex lidar_avoidance_node
```

It consumes `/scan` and publishes `/cmd_vel` directly. It carries a
`scan_timeout` deadman: if scans stop arriving, it stops the robot. Do not
remove it.

---

## 5. On the physical robot

```bash
# 1. Description + serial driver
ros2 launch cobraflex cobraflex_bringup.launch.xml

# 2. Sensors — lidar + ZED + CSI camera + EKF
ros2 launch cobraflex cobraflex_sensors.launch.xml

# 3. SLAM against real scans
ros2 launch cobraflex cobraflex_mapping.launch.py
```

Two things differ from simulation and both matter:

**TF ownership.** On hardware the **EKF** owns `odom -> base_footprint`
(`ekf_hw.yaml`, `publish_tf: true`), not the simulator. The ZED wrapper's own
broadcast is disabled via `publish_tf:=false` in the sensors launch file.

**The deadman.** `cobraflex_ros_driver` re-sends the last velocity every 50 ms
to defeat the firmware's own timeout, which means `cmd_timeout` (default 0.5 s)
is the *only* thing that stops a physical robot whose commander has died. Keep
any new `/cmd_vel` publisher consistent with it.

> The firmware interprets every twist through its own `TRACK_WIDTH` = 0.159 and
> `WHEEL_D` = 0.0739 — **not** the 0.154 / 0.0745 the URDF and Gazebo use. That
> is a known, unresolved ~3–4 % yaw gain difference between sim and hardware.
> See [parameters.md §1.4](../assets/Mathematical%20Model/parameters.md).

---

## 6. Choosing a world

```bash
ros2 launch cobraflex gazebo_mesh.launch.py world:=oval_complex gui:=true
```

A bare token resolves to `worlds/lane_following_<token>.world`, so
`world:=oval_complex` and `world:=lane_following_oval_complex` are the same
thing. A value containing a path separator, or ending in `.world` / `.sdf`, is
passed through unchanged.

| World | For |
| --- | --- |
| `obstacles.world` | Default for `gazebo.launch.py` — SLAM and Nav2 demos |
| `oval_simple` | Gentler circuit, the easy lane-keeping baseline |
| `oval_complex` | Default lane-following circuit (`complex_b`) |
| `complex_b_flipH` / `flipV` | Same circuit mirrored — exposes a steering bias |
| `complex_b_worn_25/50/75` | Paint degraded 25/50/75 % — the intended degradation sweep |
| `complex_b_gaps` | Line dropouts — tests how the estimator holds through a gap |
| `straight_road.world` | Controller step responses |
| `empty.world` | Ground plane only, for URDF bring-up debugging |

Full list and the texture-generation scripts:
[`src/cobraflex/worlds/README.md`](../src/cobraflex/worlds/README.md).

---

## 7. Inspection and debugging

```bash
# What is running
ros2 node list
ros2 topic list
ros2 node info /slam_toolbox
ros2 param list /controller_server

# Is the data flowing at the right rate?
ros2 topic hz /scan            # expect ~10 Hz
ros2 topic hz /odom            # expect ~50 Hz
ros2 topic echo /cmd_vel

# TF
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_tools view_frames        # writes frames.pdf

# Record a session for offline analysis
ros2 bag record -a -o navigation_data
```

**Validating a world before launching it:**

```bash
gz sdf -k src/cobraflex/worlds/lane_following_oval_complex.world
```

**Running the linters** (`ament_copyright`, `ament_flake8`, `ament_pep257`):

```bash
colcon test --packages-select cobraflex
colcon test-result --verbose
```

---

## Related

- [INSTALLATION.md](INSTALLATION.md) — setup from a clean machine
- [Mathematical Model](../assets/Mathematical%20Model/README.md) — kinematics, control, parameters
- [`src/cobraflex/config/`](../src/cobraflex/config/) — every tunable, with rationale in comments
