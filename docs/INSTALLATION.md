# Installation

Setup of the Cobra Flex workspace, from a clean Ubuntu 22.04 machine to a built
and sourced ROS 2 workspace.

Short version: [Quick start](../README.md#quick-start) in the root README.

**Target platform**

| Component | Version |
| --- | --- |
| OS | Ubuntu 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| Gazebo | Harmonic (gz-sim 8) |
| Python | 3.10 (default of Ubuntu 22.04 and Humble) |
| CMake | 3.16+ (`cobraflex_safety_msgs` only, for message generation) |

---

## 1. ROS 2 Humble

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

## 2. Gazebo Harmonic

```bash
sudo apt install gz-harmonic ros-humble-ros-gz
```

## 3. Project dependencies

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

| Package | Role |
| --- | --- |
| `ros-humble-ros-gz` | Gazebo Harmonic ↔ ROS 2 bridge |
| `ros-humble-navigation2`, `ros-humble-nav2-bringup` | Nav2 stack and launch files |
| `ros-humble-slam-toolbox` | Graph SLAM, occupancy grid |
| `ros-humble-robot-localization` | EKF; publishes `odom -> base_footprint` on hardware |
| `ros-humble-rviz2` | Visualisation |
| `ros-humble-teleop-twist-keyboard` | Keyboard driving during mapping |
| `ros-humble-robot-state-publisher`, `ros-humble-xacro` | URDF expansion and TF publication |
| `ros-humble-tf2-tools` | `view_frames`, `tf2_echo` for TF diagnostics |

## 4. Repository

> **The repository is the workspace root.** It contains `src/` with all five
> packages and is cloned as `~/ros2_ws`, not into `~/ros2_ws/src`. Cloning into
> `src/` places the packages one level too deep, and `colcon` does not find
> them.

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

With `--symlink-install`, changes to Python nodes take effect without a
rebuild.

Single package:

```bash
colcon build --packages-select cobraflex --symlink-install
```

Automatic sourcing in every new terminal:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## Hardware dependencies

The physical robot requires two packages that are not available in the apt
repositories. They are built from source in the same workspace:

| Package | Device | Source |
| --- | --- | --- |
| `sllidar_ros2` | RPLIDAR A2 | [Slamtec/sllidar_ros2](https://github.com/Slamtec/sllidar_ros2) |
| `zed_wrapper` | ZED Mini | [stereolabs/zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper) |

These packages are not required for simulation.

Access to the serial link of the ESP32-S3 requires membership in the `dialout`
group:

```bash
sudo usermod -aG dialout $USER   # effective after the next login
```

---

## Verification

```bash
source ~/ros2_ws/install/setup.bash

# 1. Package list
ros2 pkg list | grep -E 'cobraflex|safety_cage'
#    cobraflex
#    cobraflex_rl
#    cobraflex_safety_msgs
#    cobraflex_teleop_gui
#    safety_cage

# 2. Robot description and world
ros2 launch cobraflex gazebo.launch.py

# 3. Second terminal: TF tree
ros2 run tf2_tools view_frames
```

A correct tree has exactly one publisher of `odom -> base_footprint`. A robot
model that jumps in RViz indicates a second publisher of that edge; see
[Mathematical Model § Odometry](../assets/Mathematical%20Model/Kinematics.md).

---

## Troubleshooting

| Symptom | Cause | Solution |
| --- | --- | --- |
| `colcon` finds no packages | Repository cloned into `~/ros2_ws/src` instead of as `~/ros2_ws` | Clone one level higher |
| Python changes have no effect | Built without `--symlink-install` | Rebuild with the flag |
| Robot model jumps in RViz | More than one publisher of `odom -> base_footprint` | Check the DiffDrive plugin, the OdometryPublisher and the EKF |
| Nav2 does not start | No saved map | Run the mapping first; see [Usage](USAGE.md#1-slam-and-mapping) |
| New asset directory missing in `share/` | Directory not listed in `setup.py` `data_files` | Add it there and rebuild |
| Serial permission denied | User not in `dialout` | `sudo usermod -aG dialout $USER`, then log in again |

---

## Further documentation

- [USAGE.md](USAGE.md): SLAM, navigation and lane-keeping controllers
- [Mathematical Model](../assets/Mathematical%20Model/README.md): kinematics, control and parameters
