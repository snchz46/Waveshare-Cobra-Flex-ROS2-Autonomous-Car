# Installation

Full setup for the Cobra Flex workspace, from a clean Ubuntu 22.04 machine to a
built and sourced ROS 2 workspace.

For the three-command version, see [Quick Start](../README.md#quick-start) in
the root README.

**Target platform**

| Component | Version |
| --- | --- |
| OS | Ubuntu 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| Gazebo | Harmonic (gz-sim 8) |
| Python | 3.10 (what 22.04 and Humble ship) |
| CMake | 3.16+ (only `cobraflex_safety_msgs`, for message generation) |

---

## 1. Install ROS 2 Humble

```bash
sudo apt update && sudo apt install locales curl
sudo locale-gen en_US en_US.UTF-8
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt upgrade
sudo apt install ros-humble-desktop
```

## 2. Install Gazebo Harmonic

```bash
sudo apt install gz-harmonic ros-humble-ros-gz
```

## 3. Install project dependencies

```bash
sudo apt install -y \
  python3-colcon-common-extensions python3-rosdep \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-robot-localization \
  ros-humble-rviz2 ros-humble-xacro \
  ros-humble-teleop-twist-keyboard ros-humble-joy \
  ros-humble-robot-state-publisher ros-humble-tf2-tools

sudo rosdep init && rosdep update
```

What each one is for:

| Package | Role |
| --- | --- |
| `ros-humble-ros-gz` | Gazebo Harmonic ↔ ROS 2 bridge |
| `ros-humble-navigation2`, `ros-humble-nav2-bringup` | Nav2 stack and its launch files |
| `ros-humble-slam-toolbox` | Graph SLAM, builds the occupancy grid |
| `ros-humble-robot-localization` | EKF — owns `odom -> base_footprint` **on hardware** |
| `ros-humble-rviz2` | Visualisation |
| `ros-humble-teleop-twist-keyboard` | Keyboard driving during mapping |
| `ros-humble-robot-state-publisher`, `ros-humble-xacro` | URDF expansion and TF publishing |
| `ros-humble-tf2-tools` | `view_frames`, `tf2_echo` — TF debugging |

## 4. Clone the repository

> **The repository is the workspace root.** It already contains `src/` with all
> four packages, so clone it *as* `~/ros2_ws` — **not** into `~/ros2_ws/src`, or
> the packages end up one level too deep and `colcon` will not find them.

```bash
git clone https://github.com/snchz46/Waveshare-Cobra-Flex-ROS2-Autonomous-Car.git ~/ros2_ws
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

## 5. Build

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` matters: without it, every edit to a Python node needs a
rebuild before it takes effect.

Iterating on a single package:

```bash
colcon build --packages-select cobraflex --symlink-install
```

Sourcing automatically in every new terminal:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## Hardware-only dependencies

The physical robot needs two packages that are **not** in the apt repositories.
Build them from source into the same workspace:

| Package | For | Source |
| --- | --- | --- |
| `sllidar_ros2` | RPLIDAR A2 | [Slamtec/sllidar_ros2](https://github.com/Slamtec/sllidar_ros2) |
| `zed_wrapper` | ZED Mini | [stereolabs/zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper) |

Simulation needs neither — skip this section if you are only running Gazebo.

The serial link to the ESP32-S3 also needs your user in the `dialout` group:

```bash
sudo usermod -aG dialout $USER   # log out and back in for this to take effect
```

---

## Verifying the install

```bash
source ~/ros2_ws/install/setup.bash

# 1. All four packages found?
ros2 pkg list | grep -E 'cobraflex|safety_cage'
#    cobraflex
#    cobraflex_rl
#    cobraflex_safety_msgs
#    safety_cage

# 2. Does the description expand and the world load?
ros2 launch cobraflex gazebo.launch.py

# 3. In a second terminal — is the TF tree complete?
ros2 run tf2_tools view_frames
```

A healthy tree has exactly one publisher for `odom -> base_footprint`. If RViz
shows the robot jumping every cycle, two nodes are fighting over that edge — see
[Mathematical Model § Odometry](../assets/Mathematical%20Model/Kinematics.md).

---

## Troubleshooting

| Symptom | Cause | Fix |
| --- | --- | --- |
| `colcon` finds no packages | Repo cloned into `~/ros2_ws/src` instead of *as* `~/ros2_ws` | Re-clone one level up |
| Python edits have no effect | Built without `--symlink-install` | Rebuild with the flag |
| Robot model jumps in RViz | More than one publisher of `odom -> base_footprint` | Check the DiffDrive plugin, the OdometryPublisher and the EKF |
| Nav2 refuses to start | No saved map | Run mapping first — see [USAGE](USAGE.md#1-slam--building-a-map) |
| A new asset directory never reaches `share/` | Not globbed into `setup.py` `data_files` | Add it there, then rebuild |
| Serial permission denied | User not in `dialout` | `sudo usermod -aG dialout $USER`, then re-login |

---

## Next steps

- [USAGE.md](USAGE.md) — running SLAM, navigation and the lane-keeping controllers
- [Mathematical Model](../assets/Mathematical%20Model/README.md) — kinematics, control and parameters
