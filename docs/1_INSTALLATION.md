# Installation

Setup of the two machines of the Cobra Flex, from a clean operating system to a
built and sourced ROS 2 workspace. The lab PC runs the simulation and the
operator tools; the Jetson Orin Nano on the robot runs the driver, the sensors
and the on-board applications. Both machines build this repository, on
different Ubuntu and ROS 2 releases.

Short version for the lab PC: [Quick start](../README.md#quick-start) in the root README.

**Machines**

| | Lab PC | Robot |
| --- | --- | --- |
| Hardware | x86-64 (amd64) | NVIDIA Jetson Orin Nano Developer Kit (arm64) |
| Operating system | Ubuntu 24.04 LTS | Ubuntu 22.04 LTS, base of JetPack 6 (L4T R36) |
| ROS 2 | Jazzy Jalisco | Humble Hawksbill |
| Gazebo | Harmonic (gz-sim 8), from the ROS repository | Not installed |
| Python | 3.12 | 3.10 |
| Applications | Gazebo, RViz, teleoperation GUI, launch manager, lab session programs | Driver, sensors and EKF; LiDAR avoidance, SLAM, Nav2 and lane keeping on the physical robot |
| Procedure | [Lab PC](#lab-pc-ubuntu-2404-and-ros-2-jazzy) | [Robot](#robot-jetson-orin-nano-and-ros-2-humble) |

Each machine runs the ROS 2 release of its Ubuntu version. JetPack 6, the
software stack of the Jetson, is based on Ubuntu 22.04, the platform of ROS 2
Humble. The lab PCs run Ubuntu 24.04, the platform of ROS 2 Jazzy. Jazzy is
also the release paired with Gazebo Harmonic in the ROS repository; Humble
reaches Harmonic only through the unofficial `ros-humble-ros-gzharmonic`
packages. Conditions for the communication between the two releases:
[Interoperability of Humble and Jazzy](#interoperability-of-humble-and-jazzy).

> **The repository is the workspace root**, on both machines. It contains
> `src/` with all six packages and is cloned as `~/ros2_ws`, not into
> `~/ros2_ws/src`. Cloning into `src/` places the packages one level too deep,
> and `colcon` does not find them.

---

## Lab PC: Ubuntu 24.04 and ROS 2 Jazzy

Starting point: Ubuntu 24.04 LTS on an x86-64 PC with internet access.

### ROS 2 Jazzy

```bash
sudo apt update && sudo apt install locales curl
sudo locale-gen en_US en_US.UTF-8
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt upgrade
sudo apt install ros-jazzy-desktop
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

The last line loads the ROS 2 environment in every new terminal.

### Gazebo Harmonic

Under ROS 2 Jazzy, Gazebo Harmonic is distributed in the ROS repository as
vendor packages (`gz_*_vendor`). `ros-jazzy-ros-gz` installs it together with
the ROS–Gazebo bridge; the OSRF repository (`packages.osrfoundation.org`) is not
required ([Gazebo documentation](https://gazebosim.org/docs/harmonic/ros_installation/)).

```bash
sudo apt install ros-jazzy-ros-gz
```

### Project dependencies

```bash
sudo apt install -y \
  python3-colcon-common-extensions python3-rosdep \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox ros-jazzy-robot-localization \
  ros-jazzy-rviz2 ros-jazzy-xacro \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-joy \
  ros-jazzy-robot-state-publisher ros-jazzy-tf2-tools

sudo rosdep init && rosdep update
```

| Package | Role |
| --- | --- |
| `ros-jazzy-ros-gz` | Gazebo Harmonic and the Gazebo ↔ ROS 2 bridge ([Gazebo Harmonic](#gazebo-harmonic)) |
| `ros-jazzy-navigation2`, `ros-jazzy-nav2-bringup` | Nav2 stack and launch files |
| `ros-jazzy-slam-toolbox` | Graph SLAM, occupancy grid |
| `ros-jazzy-robot-localization` | EKF; in simulation without TF output (`ekf_gazebo.yaml`) |
| `ros-jazzy-rviz2` | Visualisation |
| `ros-jazzy-teleop-twist-keyboard` | Keyboard driving during mapping |
| `ros-jazzy-robot-state-publisher`, `ros-jazzy-xacro` | URDF expansion and TF publication |
| `ros-jazzy-tf2-tools` | `view_frames`, `tf2_echo` for TF diagnostics |

### Repository

```bash
git clone https://github.com/snchz46/Waveshare-Cobra-Flex-ROS2-Autonomous-Car.git ~/ros2_ws
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys "sllidar_ros2 zed_wrapper"
```

rosdep requires the ROS 2 environment (`ROS_DISTRO`); without it, it installs
nothing and still exits without an error. The skipped keys are the hardware
packages, which are built from source on the robot only
([Hardware dependencies](#hardware-dependencies)). Under Jazzy, the keys
`ros_gz_sim` and `ros_gz_bridge` resolve to the packages of
[Gazebo Harmonic](#gazebo-harmonic) and are not skipped.

### Build

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
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

## Robot: Jetson Orin Nano and ROS 2 Humble

Starting point: a Jetson Orin Nano Developer Kit flashed with JetPack 6
according to the
[NVIDIA getting-started guide](https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit),
with internet access. The robot is operated without a display; the commands
run in an SSH terminal
([3_ROBOT_OPERATION.md §5](3_ROBOT_OPERATION.md#5-switching-on-and-connecting)).

Installed JetPack release:

```bash
cat /etc/nv_tegra_release   # "# R36 ..." for JetPack 6
```

### ROS 2 Humble

```bash
sudo apt update && sudo apt install locales curl
sudo locale-gen en_US en_US.UTF-8
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt upgrade
sudo apt install ros-humble-ros-base
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

`ros-humble-ros-base` contains no graphical tools; RViz runs on the lab PC.
rosdep installs the remaining dependencies of the workspace in the build step.

### Project dependencies

```bash
sudo apt install -y \
  python3-colcon-common-extensions python3-rosdep \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-robot-localization \
  ros-humble-robot-state-publisher ros-humble-xacro \
  ros-humble-tf2-tools

sudo rosdep init && rosdep update
```

With the physical robot, SLAM Toolbox and Nav2 run on the Jetson;
`robot_localization` provides the EKF that publishes `odom -> base_footprint`
(`ekf_hw.yaml`).

### Repository

```bash
git clone https://github.com/snchz46/Waveshare-Cobra-Flex-ROS2-Autonomous-Car.git ~/ros2_ws
```

### Hardware dependencies

The robot requires two packages that are not available in the apt
repositories. They are built from source in the same workspace:

| Package | Device | Source |
| --- | --- | --- |
| `sllidar_ros2` | RPLIDAR A2M8 | [Slamtec/sllidar_ros2](https://github.com/Slamtec/sllidar_ros2) |
| `zed_wrapper` | ZED Mini | [stereolabs/zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper) |

`zed_wrapper` builds only with the ZED SDK installed, in the release for the
L4T version of the Jetson: [ZED SDK downloads](https://www.stereolabs.com/developers/release).
The default branch of the wrapper targets the latest ZED SDK; versions for
older SDK releases are listed on its
[releases page](https://github.com/stereolabs/zed-ros2-wrapper/releases).

```bash
cd ~/ros2_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git
git clone https://github.com/stereolabs/zed-ros2-wrapper.git
```

Access to the serial links of the ESP32-S3 and of the LiDAR requires
membership in the `dialout` group:

```bash
sudo usermod -aG dialout $USER   # effective after the next login
```

### Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys "ros_gz_sim ros_gz_bridge"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

- rosdep also installs the dependencies of the two hardware packages, which
  are now part of `src/`.
- The keys `ros_gz_sim` and `ros_gz_bridge` are skipped: the robot runs no
  simulation, and under Humble these keys install the bridge of Gazebo
  Fortress.
- `CMAKE_BUILD_TYPE=Release` follows the build instructions of the ZED
  wrapper.

---

## Interoperability of Humble and Jazzy

### Communication between the lab PC and the robot

The lab PC and the robot form one ROS 2 graph across the two releases
([09 · Multi-machine networking](concepts/09_multi_machine_networking.md)).
Nodes of Humble and Jazzy exchange messages when both machines use the same
middleware and the interface has the same definition in both releases.

| Condition | Setting |
| --- | --- |
| Same RMW implementation | Fast DDS (`rmw_fastrtps_cpp`), the default of Humble and Jazzy: `RMW_IMPLEMENTATION` unset on both machines, or set to the same value |
| Same domain | Identical `ROS_DOMAIN_ID` on the robot and its PC ([3_ROBOT_OPERATION.md §4.1](3_ROBOT_OPERATION.md#41-environment-of-each-terminal)) |
| Same repository version | The same commit built on both machines; each machine generates the messages of `cobraflex_safety_msgs` from its own copy ([3_ROBOT_OPERATION.md §4.2](3_ROBOT_OPERATION.md#42-software-update)) |

The interfaces between the PC and the robot are topics with standard message
types whose definitions are the same in both releases, for example `/cmd_vel`
(`geometry_msgs/Twist`), `/scan`, the camera images, `/tf`, `/map`,
`/initialpose` and `/goal_pose`.

Several interfaces of Nav2 (`nav2_msgs`) differ between the releases; the
result of the action `NavigateToPose`, for example, is empty in Humble and
carries an error code and message in Jazzy. Programs that use the Nav2 actions,
such as `nav2_simple_commander` scripts, therefore run on the same machine as
Nav2. From the PC, goals are set with the RViz tools *2D Pose Estimate* and
*2D Goal Pose*, which publish `/initialpose` and `/goal_pose`.

### Release-dependent files and variables

| Item | Lab PC, Jazzy | Robot, Humble |
| --- | --- | --- |
| `ROS_DISTRO` | `jazzy` | `humble` |
| Nav2 parameters | `nav2_params_jazzy.yaml` | `nav2_params.yaml` |
| Gazebo–ROS bridge | `ros-jazzy-ros-gz` | Not installed |

- `navigation.launch.py` selects the Nav2 parameter file from `ROS_DISTRO`.
  The Jazzy file uses the plugin names of Jazzy, has no `plugin_lib_names` and
  adds the sections `collision_monitor` and `docking_server`. A change to one
  file is applied to the other.
- Simulation terminals on the PC set `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`
  ([3_ROBOT_OPERATION.md §8](3_ROBOT_OPERATION.md#8-simulation)).
  `ROS_LOCALHOST_ONLY=1`, the Humble form, has the same effect under Jazzy and
  logs a deprecation warning.

---

## Verification

### Lab PC

```bash
source ~/ros2_ws/install/setup.bash

# 1. Package list
ros2 pkg list | grep -E 'cobraflex|safety_cage'
#    cobraflex
#    cobraflex_launch_gui
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

### Robot

```bash
source ~/ros2_ws/install/setup.bash
ros2 pkg list | grep -E 'cobraflex|safety_cage|sllidar_ros2|zed_wrapper'
#    cobraflex
#    cobraflex_launch_gui
#    cobraflex_rl
#    cobraflex_safety_msgs
#    cobraflex_teleop_gui
#    safety_cage
#    sllidar_ros2
#    zed_wrapper
```

Bring-up of the platform and the sensors, and the connection to the lab PC:
[3_ROBOT_OPERATION.md §4 to §6](3_ROBOT_OPERATION.md#4-preparation).

---

## Troubleshooting

| Symptom | Cause | Solution |
| --- | --- | --- |
| `Unable to locate package ros-jazzy-…` or `ros-humble-…` | Release for another Ubuntu version: Jazzy packages exist for Ubuntu 24.04, Humble packages for 22.04 | Ubuntu 24.04 (`lsb_release -cs`: `noble`) on the lab PC, 22.04 (`jammy`) on the Jetson |
| `colcon` finds no packages | Repository cloned into `~/ros2_ws/src` instead of as `~/ros2_ws` | Clone one level higher |
| Python changes have no effect | Built without `--symlink-install` | Rebuild with the flag |
| `zed_wrapper` does not build | ZED SDK missing or not the release for the L4T version; on JetPack 6, JetPack development packages missing | ZED SDK as in [Hardware dependencies](#hardware-dependencies); `sudo apt install nvidia-jetpack nvidia-jetpack-dev` |
| Serial permission denied | User not in `dialout` | `sudo usermod -aG dialout $USER`, then log in again |
| Robot model jumps in RViz | More than one publisher of `odom -> base_footprint` | Check the DiffDrive plugin, the OdometryPublisher and the EKF |
| Nav2 does not start | No saved map | Run the mapping first; see [Usage](2_USAGE.md#1-slam-and-mapping) |
| On the lab PC, `bt_navigator` stops with `ID [ComputePathToPose] already registered`, or `planner_server` does not configure | Humble parameter file under Jazzy, passed with `params_file:=` | Default `params_file` of `navigation.launch.py`, or `nav2_params_jazzy.yaml` |
| Nav2 action called from the PC fails against Nav2 on the robot | `nav2_msgs` differs between Jazzy and Humble | Program on the robot; from the PC, *2D Goal Pose* in RViz |
| Warning `ROS_LOCALHOST_ONLY is deprecated` on the lab PC | Humble variable under Jazzy; still effective | `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` instead ([3_ROBOT_OPERATION.md §8](3_ROBOT_OPERATION.md#8-simulation)) |
| New asset directory missing in `share/` | Directory not listed in `setup.py` `data_files` | Add it there and rebuild |

---

## Further documentation

- [2_USAGE.md](2_USAGE.md): SLAM, navigation and lane-keeping controllers
- [3_ROBOT_OPERATION.md](3_ROBOT_OPERATION.md): operation of the physical robot
- [09 · Multi-machine networking](concepts/09_multi_machine_networking.md): DDS discovery, domain IDs, Wi-Fi
- [Mathematical Model](../assets/Mathematical%20Model/README.md): kinematics, control and parameters
