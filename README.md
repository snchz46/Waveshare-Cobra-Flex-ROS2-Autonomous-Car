# Waveshare Cobraflex 4WD Autonomous Mobile Robot

Welcome to the **Waveshare Cobra Flex ROS 2 Autonomous Car** project. This repository contains everything you need to simulate, build and operate a skid-steer mobile robot using the Waveshare Cobra Flex chassis, the RPLIDAR A2 LiDAR, a ZED Mini stereo camera and the Nav2 navigation stack on ROS 2 Humble.

| **CAD Design** | **SImulation** | **Physical Robot**
|:---------------------:|:-----------------:|:-----------------:|
| <img src="assets/photos/Car Prototype V3_1.png" width="400"/> | <img src="assets/photos/digital_twin.png" width="600"/> | <img src="assets/photos/Mockup V2 side 2.jpg" width="400"/> |

<div align="center" width="70%">

[![Python](https://img.shields.io/badge/Python-3.10+-yellow?logo=python)](#)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu)](#)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-22314E?logo=ros)](#)
[![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-orange)](#)
[![Nav2](https://img.shields.io/badge/Nav2-Humble-00599C)](#)
[![SLAM Toolbox](https://img.shields.io/badge/SLAM-Toolbox-green)](#)

</div>

---

## Table of Contents
  
- [Waveshare Cobraflex 4WD Autonomous Mobile Robot](#waveshare-cobraflex-4wd-autonomous-mobile-robot)
  - [Table of Contents](#table-of-contents)
  - [Description](#description)
  - [Key features](#key-features)
    - [Robot Features](#robot-features)
  - [Simulation Features](#simulation-features)
  - [Robot Gallery](#robot-gallery)
  - [Video Demonstrations](#video-demonstrations)
  - [System Architecture](#system-architecture)
    - [Transform Tree (TF)](#transform-tree-tf)
    - [SLAM System](#slam-system)
    - [Navigation System](#navigation-system)
  - [Mathematical Model](#mathematical-model)
    - [Key Parameters](#key-parameters)
  - [Requirements](#requirements)
    - [Software](#software)
    - [ROS2 Dependencies](#ros2-dependencies)
  - [Quick Start](#quick-start)
  - [Installation](#installation)
    - [1. Install ROS2 Humble](#1-install-ros2-humble)
    - [2. Install Gazebo Harmonic](#2-install-gazebo-harmonic)
    - [3. Install Project Dependencies](#3-install-project-dependencies)
    - [4. Clone the Repository](#4-clone-the-repository)
  - [Build](#build)
  - [Usage](#usage)
    - [SLAM (Mapping)](#slam-mapping)
    - [Autonomous Navigation](#autonomous-navigation)
    - [Useful Commands](#useful-commands)
  - [Project structure](#project-structure)
  - [Acknowledgements](#acknowledgements)

---

## Description

Industrial autonomous mobile robots need a tightly integrated software stack to perceive the environment, localise themselves, plan safe paths and actuate the robot. Off-the-shelf chassis such as the Waveshare Cobra Flex provide a robust mechanical platform but not a ready-made control and navigation solution. This repository bridges that gap by providing:

- A digital twin of the Cobra Flex in Gazebo Harmonic with URDF/Xacro models, sensors and realistic physics.
- A navigation stack built on Nav2, AMCL and SLAM Toolbox, parameterised for this chassis rather than left on library defaults.
- Python drivers to control the real robot over a serial link and publish battery and wheel feedback.
- Two lane-keeping controllers — a classical histogram tracker on the Jetson CSI camera and a calibrated CV estimator with pure-pursuit steering — plus an RL agent (`cobraflex_rl`) that shares the same estimator.
- A runtime safety monitor (`safety_cage`) that supervises whichever controller is driving.
- Project documentation covering the mathematical model, control architecture, simulation setup and media assets.

## Key features

- 4WD skid-steer base with ROS 2 integration
- Gazebo simulation and RViz visualisation
- Mapping with SLAM Toolbox
- Navigation with Nav2
- Serial bridge to the Cobra Flex microcontroller
- Example perception and calibration scripts

---

### Robot Features

| Parameter | Value |
|-----------|-------|
| Total mass | 3.5 kg |
| Dimensions (L x W x H) | 0.228 x 0.18 x 0.21 m |
| Wheel radius | 0.03725 m |
| Wheel separation | 0.154 m |
| Max torque | 20 N*m per wheel |
| LiDAR (RPLidar A2) | 360 deg, 0.15-8 m, 10 Hz |
| Camera (Zed mini) | 2K, 100FPS, 0.15-15 m, Depth Sensing |

---

## Simulation Features

<div align="center">

| Feature | Description |
|---------|-------------|
| **Real-time SLAM** | Simultaneous mapping and localization using SLAM Toolbox in asynchronous mode |
| **Autonomous Navigation** | Full Nav2 stack with global planner (NavFn/Dijkstra) and local controller (DWB) |
| **Obstacle Avoidance** | Real-time detection and evasion using the 360-degree RPLidar A2 LiDAR |
| **Lane Keeping** | Two controllers: classical histogram tracking on the Jetson CSI camera, and a calibrated CV estimator with pure-pursuit steering in simulation |
| **Keyboard Teleoperation** | Standard teleop_twist_keyboard support for manual control during mapping |
| **Full Visualization** | RViz2 with dynamic costmaps, planned trajectories, and AMCL particle clouds |
| **4WD Differential Robot** | Skid-steering kinematics with EKF-fused odometry (`robot_localization`) |
| **Gazebo Harmonic Simulation** | Modern Gazebo Sim with ros_gz bridge for all sensor and actuator interfaces |
| **Configurable Parameters** | Nav2, AMCL, SLAM, DWB and EKF parameters in [`src/cobraflex/config/`](src/cobraflex/config/), tuned to this chassis |
| **Open Source** | MIT license, free for academic, research, and commercial use |

</div>

---

## Robot Gallery

<div align="center">
<table>
  <tr>
    <td><img src="assets/photos/Car Prototype V3_1.png" width="400"/></td>
    <td><img src="assets/videos/Assembly Video Mockup V2.gif" width="400"/></td>
  </tr>
  <tr>
    <td><img src="assets/photos/digital_twin.png" width="400"/></td>
    <td><img src="assets/photos/digital_wtin_rviz.png" width="400"/></td>
  </tr>
  <tr>
    <td><img src="assets/photos/Mockup V2 Front.jpg" width="400"/></td>
    <td><img src="assets/photos/Mockup V2 Back.jpg" width="400"/></td>
  </tr>
  <tr>
    <td><img src="assets/photos/Mockup V2 side 2.jpg" width="400"/></td>
    <td><img src="assets/photos/Mockup V2 Side 1.jpg" width="400"/></td>
  </tr>
</table>
</div>

---

## Video Demonstrations

<div align="center">

*Complete walkthrough: real-time SLAM, map saving, and autonomous Nav2 navigation*

</div>

<div align="center">
  
| **SImulation** | **Mechanical Assembly** |
|:---------------------:|:-----------------:|
| <img src="assets/videos/simulation.gif" width="400"/> | <img src="assets/videos/Assembly Video Mockup V2.gif" width="400"/> |
| *Gazebo Simulation* | *CAD design in Autodesk Inventor* |

| **SLAM and Mapping** | **Autonomous Navigation** |
|:------------------------:|:-----------------:|
| <img src="assets/videos/mapping.gif" width="400"/> | <img src="assets/videos/navigation.gif" width="400"/> |
| *Real-time mapping with LiDAR* | *Navigation in a mapped environment* |

</div>

---

## System Architecture

### Transform Tree (TF)

<div align="center">
<img src="assets/photos/digital_twin_tf.png" width="800"/>
</div>

Spatial transform tree: `map -> odom -> base_footprint -> base_link -> body_link -> sensors`. The `odom_to_tf` node publishes the `odom -> base_footprint` transform from Gazebo odometry. AMCL publishes `map -> odom` to correct odometric drift during navigation.

### SLAM System

<div align="center">
<img src="assets/photos/mapping.png" width="800"/>
</div>

SLAM Toolbox runs in asynchronous mode, building graph-based 2D occupancy grid maps in real time. It processes LiDAR scans at 10 Hz and odometry at 50 Hz with pose-graph optimization and loop closure detection.

### Navigation System

<div align="center">
<img src="assets/photos/navigation.png" width="800"/>
</div>

The Nav2 stack integrates the NavFn global planner (Dijkstra), the DWB local controller (Dynamic Window Approach), dynamic costmaps with inflation and obstacle layers, and recovery behaviors (spin, backup, wait).

---

## Mathematical Model

<div align="center">
<img width="850" height="418" alt="image" src="https://github.com/user-attachments/assets/6da0f924-369f-494f-b9e7-908198959b37" />
</div>

[Complete differential 4WD skid-steering kinematic model.](https://github.com/snchz46/Waveshare-Cobra-Flex-ROS2-Autonomous-Car/blob/main/assets/Mathematical%20Model/README.md) The diagram shows the robot geometry, control equations, Nav2 integration, and dynamic specifications.

### Key Parameters

| Parameter | Value |
| --------- | ----- |
| Wheel radius | $r = 0.03725$ m |
| Wheel separation (track) | $W = 0.154$ m |
| Wheelbase | $L = 0.120$ m |
| Total mass | $m = 3.5$ kg |
| Max linear velocity | $0.35$ m/s planned by Nav2, $0.53$ m/s platform clamp |
| Max angular velocity | $2.0$ rad/s planned by Nav2, $6.0$ rad/s platform clamp |
| Max linear acceleration | $a_{max} = 2.5$ m/s² |
| Max angular acceleration | $\alpha_{max} = 3.2$ rad/s² |

**Differential kinematics:**

$$v = \frac{r(\omega_R + \omega_L)}{2}, \quad \omega = \frac{r(\omega_R - \omega_L)}{W}$$

---

## Requirements

### Software

- **Operating System**: Ubuntu 22.04 LTS
- **ROS2**: Humble Hawksbill
- **Gazebo**: Harmonic (gz-sim 8)
- **Python**: 3.10 (the version Ubuntu 22.04 and Humble ship)
- **CMake**: 3.16+ (only `cobraflex_safety_msgs`, for message generation)

### ROS2 Dependencies

```bash
ros-humble-ros-gz                  # Gazebo Harmonic integration
ros-humble-navigation2             # Full Nav2 stack
ros-humble-nav2-bringup            # Nav2 launch files
ros-humble-slam-toolbox            # SLAM mapping
ros-humble-robot-localization      # EKF, owns odom -> base_footprint on hardware
ros-humble-rviz2                   # Visualization
ros-humble-teleop-twist-keyboard   # Keyboard teleoperation
ros-humble-robot-state-publisher   # URDF TF publishing
ros-humble-xacro                   # URDF macro expansion
ros-humble-tf2-tools               # TF debugging utilities
```

The physical robot additionally needs two packages that are **not** in the apt
repositories and must be built from source into the same workspace:
[`sllidar_ros2`](https://github.com/Slamtec/sllidar_ros2) (RPLidar A2) and
[`zed_wrapper`](https://github.com/stereolabs/zed-ros2-wrapper) (ZED Mini).
Simulation does not need either.

---

## Quick Start

```bash
# 1. Install ROS2 Humble (Ubuntu 22.04)
sudo apt update && sudo apt install ros-humble-desktop

# 2. Install project dependencies
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-argcomplete \
                     ros-humble-ros-gz ros-humble-navigation2 ros-humble-nav2-bringup \
                     ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
                     ros-humble-slam-toolbox ros-humble-robot-localization \
                     ros-humble-teleop-twist-keyboard \
                     ros-humble-rviz2 ros-humble-xacro ros-humble-tf2-tools

# 3. Clone and build
# This repository already contains the src/ directory, so it IS the workspace
# root -- clone it as ~/ros2_ws, not into ~/ros2_ws/src.
git clone https://github.com/snchz46/Waveshare-Cobra-Flex-ROS2-Autonomous-Car.git ~/ros2_ws
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

# 4. Launch Robot (Gazebo)
ros2 launch cobraflex gazebo.launch.py

# 5. Launch SLAM (mapping)
ros2 launch cobraflex mapping.launch.py

# 6. Save the map once explored, into the package's own maps/ directory
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/cobraflex/maps/cobraflex_map
colcon build --packages-select cobraflex --symlink-install

# 7. Launch autonomous navigation against that map
ros2 launch cobraflex navigation.launch.py
```

See [Installation](#installation) and [Usage](#usage) for detailed instructions.

---

## Installation

### 1. Install ROS2 Humble

```bash
# Configure locale and repository
sudo apt update && sudo apt install locales curl
sudo locale-gen en_US en_US.UTF-8
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble Desktop
sudo apt update && sudo apt upgrade
sudo apt install ros-humble-desktop
```

### 2. Install Gazebo Harmonic

```bash
sudo apt install gz-harmonic ros-humble-ros-gz
```

### 3. Install Project Dependencies

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

### 4. Clone the Repository

The repository already contains the `src/` directory holding the four
packages, so it is the workspace root itself — clone it *as* `~/ros2_ws`
rather than into `~/ros2_ws/src`, or the packages end up one level too deep.

```bash
git clone https://github.com/snchz46/Waveshare-Cobra-Flex-ROS2-Autonomous-Car.git ~/ros2_ws
```

Resolve the declared dependencies before the first build:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

---

## Build

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

To build a single package while iterating:

```bash
colcon build --packages-select cobraflex --symlink-install
```

To automatically source the workspace on every new terminal:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## Usage

### SLAM (Mapping)

Launch the simulation with Gazebo:

```bash
ros2 launch cobraflex gazebo.launch.py
```

Launch SLAM Toolbox and RViz:

```bash
ros2 launch cobraflex mapping.launch.py
```

In a separate terminal, control the robot to explore the environment:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Save the map once the environment has been fully explored:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/cobraflex/maps/cobraflex_map
```

Rebuild so the map reaches the install share. See
[`src/cobraflex/maps/README.md`](src/cobraflex/maps/README.md) for details.

### Autonomous Navigation

Launch the simulation with Nav2 and RViz (requires a previously saved map):

```bash
ros2 launch cobraflex navigation.launch.py
```

Nav2 runs on [`config/nav2_params.yaml`](src/cobraflex/config/nav2_params.yaml),
tuned to this robot's real footprint (0.228 x 0.180 m, circumscribed radius
0.145 m) and to `base_footprint`. To point it elsewhere:

```bash
ros2 launch cobraflex navigation.launch.py map:=/path/to/map.yaml params_file:=/path/to/params.yaml
```

In RViz2:

1. Use **2D Pose Estimate** to set the robot initial pose
2. Use **2D Goal Pose** to send a navigation goal
3. Monitor the global/local costmaps, planned paths, and AMCL particle distribution

### Useful Commands

```bash
# Monitoring
ros2 node list                           # Active nodes
ros2 topic list                          # Active topics
ros2 topic hz /scan                      # LiDAR frequency
ros2 topic echo /cmd_vel                 # Velocity commands
ros2 run tf2_ros tf2_echo map base_link  # TF lookup
ros2 run tf2_tools view_frames           # TF tree diagram

# Debugging
ros2 node info /slam_toolbox
ros2 param list /controller_server
ros2 bag record -a -o navigation_data
```

---

## Project structure

The repository is itself a ROS 2 workspace root: it carries the `src/`
directory, and `colcon build` from the top level picks up all four packages.

```text
.
├── README.md
├── LICENSE                          # MIT, applies to the whole repository
├── assets/                          # Documentation media and CAD, not built
│   ├── 3d-models/                   # STL / STEP for chassis and sensor mounts
│   ├── Mathematical Model/          # Kinematics, control and parameter reference
│   ├── Gazebo Simulation/           # Simulation setup notes
│   ├── photos/
│   └── videos/
└── src/
    ├── cobraflex/                   # Main package: driver, description, sim, nav
    │   ├── cobraflex/               # Nodes
    │   │   ├── cobraflex_ros_driver.py      # /cmd_vel -> JSON over serial
    │   │   ├── lidar_avoidance_node.py      # /scan -> /cmd_vel avoidance
    │   │   ├── lane_keeper_node.py          # CSI camera lane keeping (hardware)
    │   │   └── lane_keeper_gazebo_node.py   # CV + pure-pursuit lane keeping (sim)
    │   ├── config/                  # EKF, SLAM Toolbox, Nav2, gz bridge, ZED
    │   ├── launch/                  # Bringup, Gazebo, mapping, navigation
    │   ├── maps/                    # Saved occupancy grids (gitignored)
    │   ├── materials/road_assets/   # Generated road textures for the sim worlds
    │   ├── meshes/                  # Visual STLs referenced by the URDFs
    │   ├── rviz/                    # RViz layouts
    │   ├── urdf/                    # Robot descriptions + Gazebo plugin block
    │   └── worlds/                  # SDF worlds
    ├── cobraflex_rl/                # RL lane-following agent + shared CV estimator
    ├── safety_cage/                 # Runtime safety monitor over the controllers
    └── cobraflex_safety_msgs/       # CageStatus.msg (CMake / message generation)
```

---

## Acknowledgements

This project started from **[Axioma_robot](https://github.com/MrDavidAlv/Axioma_robot)**
by [MrDavidAlv](https://github.com/MrDavidAlv) — *"Robot autónomo ROS2 Humble |
SLAM + Nav2 + Gazebo | Navegación autónoma para logística industrial"*, released
under the BSD licence.

Axioma_robot is a ROS 2 Humble autonomous robot built on a 4WD skid-steer
chassis with SLAM Toolbox and Nav2. It is the same class of platform and the
same software stack as this one, and it is where the idea for this project came
from. Its documentation — in particular the way the robot's mathematical model
is organised into kinematics, control and parameters — is the direct basis for
[`assets/Mathematical Model/`](./assets/Mathematical%20Model/README.md).

The two robots are **different chassis**, and none of the numbers carry over:
Axioma runs a 0.0381 m wheel radius, a 0.1679 m effective track and a 0.26 m/s
top speed, against 0.03725 m, 0.154 m and 0.35 m/s here. Every figure in this
repository's documentation has been re-derived from its own URDFs, configuration
files, firmware source and bench measurements.

Thanks to MrDavidAlv for publishing the work openly.

---
