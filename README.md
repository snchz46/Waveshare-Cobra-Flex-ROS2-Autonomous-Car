<div align="center">

# Waveshare Cobra Flex ROS 2 Autonomous Car

### 1:14 Scaled Autonomous Vehicle Platform with Jetson Orin Nano, ZED Stereo Vision, and RPLIDAR

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white)](#software-stack)
[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white)](#software-stack)
[![Status](https://img.shields.io/badge/Status-Work%20in%20Progress-F4B400)](#project-status)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

</div>

---

## 📌 Table of Contents
- [🚗 Project Overview](#-project-overview)
- [🎯 Objectives](#-objectives)
- [📷 Media Gallery (Placeholders)](#-media-gallery-placeholders)
- [🧰 Hardware Setup](#-hardware-setup)
- [💻 Software Stack](#-software-stack)
- [🗂️ Repository Structure](#️-repository-structure)
- [⚙️ Installation](#️-installation)
- [▶️ Usage](#️-usage)
- [🧪 Core ROS 2 Scripts](#-core-ros-2-scripts)
- [🛣️ Roadmap](#️-roadmap)
- [🤝 Contributing](#-contributing)
- [📄 License](#-license)

---

## 🚗 Project Overview
This repository documents the development of an autonomous 1:14-scale car based on the **Waveshare Cobra Flex** chassis.

The platform is designed to validate and iterate on:
- ROS 2 integration on embedded compute (Jetson Orin Nano)
- Sensor fusion between **ZED stereo depth** and **2D LiDAR**
- Perception-to-control pipelines for future autonomous navigation

The repo combines ROS 2 packages, experiment scripts, media artifacts, and mechanical references so the full process stays reproducible.

---

## 🎯 Objectives
- Build a reproducible baseline for the Cobra Flex + Jetson stack
- Align LiDAR and camera frames for consistent fused perception
- Compare ZED depth vs. LiDAR measurements for calibration confidence
- Prepare the platform for navigation, SLAM, and planning experiments

---

## 📷 Media Gallery (Placeholders)

> Replace the placeholders below with your final photos/videos/diagrams.

### Hero Media
- **Project Hero Image:** `<<PLACEHOLDER_HERO_IMAGE>>`
- **Project Demo GIF/Video:** `<<PLACEHOLDER_HERO_DEMO>>`

### Hardware Build
- **Chassis Front View:** `<<PLACEHOLDER_CHASSIS_FRONT>>`
- **Chassis Rear View:** `<<PLACEHOLDER_CHASSIS_REAR>>`
- **Side Profile:** `<<PLACEHOLDER_CHASSIS_SIDE>>`
- **Electronics Layout:** `<<PLACEHOLDER_ELECTRONICS_LAYOUT>>`

### Perception & Results
- **RViz Fusion Screenshot:** `<<PLACEHOLDER_RVIZ_FUSION>>`
- **LiDAR Projection Debug View:** `<<PLACEHOLDER_LIDAR_PROJECTION>>`
- **Distance Comparison Plot:** `<<PLACEHOLDER_DISTANCE_COMPARISON>>`

### Assembly
- **Assembly Time-lapse / Video:** `<<PLACEHOLDER_ASSEMBLY_VIDEO>>`
- **Exploded / Disassembled View:** `<<PLACEHOLDER_DISASSEMBLED_VIEW>>`

---

## 🧰 Hardware Setup

| Component | Specification | Reference |
|---|---|---|
| Host Computer | NVIDIA Jetson Orin Nano (8 GB) | [NVIDIA Jetson Orin Nano](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/) |
| Chassis | Waveshare Cobra Flex | [Waveshare Cobra Flex](https://www.waveshare.com/product/ai/robots/mobile-robots/cobra-flex.htm?sku=31326) |
| Stereo Camera | ZED Mini | [Stereolabs ZED Mini](https://store.stereolabs.com/products/zed-mini) |
| LiDAR | RPLIDAR A2M8 | [Slamtec RPLIDAR A2](https://www.slamtec.com/en/Lidar/A2) |
| Motor Driver Board | Dual TB6612FNG + PCA9685 (Cobra Flex driver board) | [Driver board info](https://www.waveshare.com/wiki/Cobra_Flex#Driver_Board) |
| Host Power | XT-27000DC-AO-PA power bank (19V) | [Power bank reference](https://www.amazon.de/XTPower-XT-27000DC-AO-PA-Uninterrupted-Adapter-Included/dp/B09S6F56T4/) |
| Chassis Power | 3S battery pack (~12V class) | `<<PLACEHOLDER_BATTERY_REFERENCE>>` |
| Mounting Hardware | Custom brackets + M2/M3 fasteners | `<<PLACEHOLDER_MOUNTING_NOTES>>` |

---

## 💻 Software Stack

- **OS:** Ubuntu 22.04
- **ROS 2 Distribution:** Humble
- **ZED SDK:** 5.1
- **ZED ROS 2 Wrapper:** [stereolabs/zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper)
- **RPLIDAR ROS 2 Driver:** [Slamtec/rplidar_ros (ros2)](https://github.com/Slamtec/rplidar_ros/tree/ros2)
- **Workspace Model:** Colcon + Python nodes (`rclpy`)

---

## 🗂️ Repository Structure

```text
.
├── assets/
│   ├── README.md
│   ├── 3d-models/
│   ├── photos/
│   └── videos/
├── docs/
│   ├── README.md
│   ├── experiments/
│   └── media-log.md
├── ros2_ws/
│   ├── README.md
│   └── src/
│       └── cobraflex/
├── scripts/
│   ├── lidar_to_zed_pointcloud.py
│   ├── lidar_to_zed_projection_debug.py
│   ├── lidar_zed_distance_comparison.py
│   └── windows_yolov8_cam_sub_ZED.py
├── LICENSE
└── README.md
```

---

## ⚙️ Installation

### 1) System Preparation
```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep
```

### 2) ROS 2 Environment
```bash
source /opt/ros/humble/setup.bash
```

### 3) Clone Repository
```bash
git clone https://github.com/snchz46/Waveshare-Cobra-Flex-ROS2-Autonomous-Car.git
cd Waveshare-Cobra-Flex-ROS2-Autonomous-Car
```

### 4) Build Workspace
```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 5) Permissions for Scripts (if needed)
```bash
chmod +x ../scripts/*.py
```

---

## ▶️ Usage

### Bringup (example)
```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch cobraflex_bringup cobraflex.launch.xml
```

### Run individual perception scripts (example)
```bash
ros2 run <your_package> lidar_to_zed_pointcloud.py
ros2 run <your_package> lidar_to_zed_projection_debug.py
ros2 run <your_package> lidar_zed_distance_comparison.py
```

> Replace `<your_package>` with your actual package name or integrate these scripts into your package entry points.

---

## 🧪 Core ROS 2 Scripts

| Script | Purpose | I/O Topics |
|---|---|---|
| `scripts/lidar_to_zed_pointcloud.py` | Converts `/scan` LaserScan data to `PointCloud2` in camera-aligned coordinates | Sub: `/scan`, `/zed/zed_node/left/camera_info` • Pub: `/lidar_in_camera_frame` |
| `scripts/lidar_to_zed_projection_debug.py` | Projects LiDAR points into ZED image for extrinsic sanity checks | Sub: `/scan`, `/zed/zed_node/left/image_rect_color`, `/zed/zed_node/left/camera_info` |
| `scripts/lidar_zed_distance_comparison.py` | Compares ZED depth readings against LiDAR ranges | Sub: `/scan`, `/zed/zed_node/depth/depth_registered` |

---

## 🛣️ Roadmap

- [ ] Replace placeholders with final media assets
- [ ] Finalize complete wiring diagram and power distribution map
- [ ] Add reproducible calibration procedure (camera ↔ LiDAR extrinsics)
- [ ] Publish benchmark logs for depth-vs-LiDAR comparison
- [ ] Integrate localization + navigation stack
- [ ] Add simulation parity checks with Gazebo scenes

---

## 🤝 Contributing

Contributions are welcome.

If you plan to contribute:
1. Fork the repository
2. Create a feature branch
3. Keep experiments documented in `docs/experiments/`
4. Submit a pull request with clear test/validation notes

---

## 📄 License

This project is distributed under the MIT License. See [LICENSE](LICENSE) for details.

---

## Project Status

**Work in progress** — architecture, calibration workflow, and autonomy stack are actively evolving.
