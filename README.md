# Jetson Orin Nano Autonomous Car with Waveshare Cobra Flex Chasis

> **Status:** Work in progress — this repository documents the ongoing integration of a ZED stereo camera and an RPLIDAR sensor on a Waveshare Cobra Flex platform powered by an NVIDIA Jetson Orin Nano.

## Project Overview
This project showcases a ROS 2 development and sensor fusion work while building an autonomous 1:14 car. The primary goal is to create a reproducible reference for deploying a Jetson Orin Nano with stereo vision (ZED) and 2D LiDAR (RPLIDAR) to perceive the environment, validate sensor agreement, and provide the foundation for autonomous navigation on the Waveshare Cobra Flex chassis.

The repository collects:
- **ROS 2 nodes** for translating LiDAR scans into the ZED camera frame, projecting them onto ZED images for debugging, and numerically comparing depth readings between the sensors.
- **Visualization assets** for fusing point clouds in RViz.
- **Documentation templates** to capture hardware configuration, ROS graph design, calibration procedures, and lessons learned as the project matures.

## Hardware Platform & Bill of Materials
Use the table below to capture every confirmed part in the build. Update the vendor links, part numbers, and firmware notes as you source components so other makers can reproduce the platform without guesswork.

| Component | Details | Vendor / Reference |
| --- | --- | --- |
| Compute | NVIDIA Jetson Orin Nano (8 GB) developer kit | [NVIDIA](https://developer.nvidia.com/embedded/jetson-orin-nano-devkit) |
| Chassis | Waveshare Cobra Flex | [Waveshare](https://www.waveshare.com/wiki/Cobra_Flex) |
| Stereo Camera | ZED Mini stereo module | [Stereolabs](https://store.stereolabs.com/products/zed-mini) |
| LiDAR | RPLIDAR A2M8 360° laser scanner | [Slamtec](https://www.slamtec.com/en/Lidar/A2) |
| IMU / Additional Sensors | ZED Mini integrated IMU | — |
| Motor Controller | Waveshare Cobra Flex driver (dual TB6612FNG + PCA9685) | [Waveshare board details](https://www.waveshare.com/wiki/Cobra_Flex#Driver_Board) |
| Power System | 2S 18650 lithium battery pack (nominal 7.4 V) with 5 V/5 A BEC | Example: [Adafruit UBEC 5V 3A+](https://www.adafruit.com/product/1385) |
| Fasteners & Mounts | Custom camera/LiDAR brackets, M2/M3 hardware | Document specific sources as mounts are finalized |

> _Add rows for cables, storage, networking, and calibration tools (checkerboards, levels, etc.) as the build is finalized._

## ROS 2 Environment
- **Distribution:** ROS2 Humble on Ubuntu 22.04
- **ZED SDK:** 5.1
- **ZED ROS2 Package:** [ZED ROS2 Wrapper](https://github.com/stereolabs/zed-ros2-wrapper)
- **RPLIDAR Driver:** [RPLIDAR ROS2 Package](https://github.com/Slamtec/rplidar_ros/tree/ros2)

Record any custom workspace overlays, `colcon` packages, or launch files in the [`docs/`](docs/) directory as you refine the system.

## Repository Structure
```
├── assets/
│   ├── photos/                       # Curated image gallery documenting the build
│   └── videos/                       # Demo footage and experiment clips
├── scripts/
│   ├── lidar_to_zed_pointcloud.py       # Publishes LiDAR data as a PointCloud2 in the ZED camera frame
│   ├── lidar_to_zed_projection_debug.py # Projects LiDAR points into the ZED image for visual alignment debugging
│   ├── lidar_zed_distance_comparison.py # Compares ZED depth output against LiDAR ranges for accuracy checks
│   ├── windows_yolov8_cam_sub_ZED.py    # Windows-side YOLOv8 subscriber for ZED video (experiment)
│   ├── Pointcloud_Fusion.rviz           # RViz configuration for visualizing fused sensor data
│   ├── Lidar_ZED_Distance.png           # Sample comparison plot
│   └── pointcloud fusion.png            # Example fused point cloud capture
├── docs/
│   ├── README.md                        # Documentation hub for setup, calibration, and roadmap details
│   ├── experiments/                     # Session-by-session experiment logs and templates
│   └── media-log.md                     # Index of photos/videos and hosted mirrors
├── LICENSE
└── README.md (this file)
```

## Media Showcase
Create visual proof points as you progress and link them directly from this repository:

- **Photos:** Store build imagery inside [`assets/photos/`](assets/photos/). Organize by milestone (`assets/photos/2024-setup/`) and add short context notes so others understand what each shot illustrates.
- **Videos:** Save drive tests and demos in [`assets/videos/`](assets/videos/). Include session summaries (timestamps, highlights, hosted mirrors) in each subfolder.
- **Media Log:** Update [`docs/media-log.md`](docs/media-log.md) with every new clip or gallery so readers can jump straight to relevant material.

Reference these assets from the sections above—for example, embed before/after shots in the hardware table or link calibration recordings alongside the ROS 2 setup instructions.

## Key ROS 2 Nodes
| Script | Purpose | Topics |
| --- | --- | --- |
| [`lidar_to_zed_pointcloud.py`](scripts/lidar_to_zed_pointcloud.py) | Converts `/scan` LaserScan data into `sensor_msgs/PointCloud2` aligned with the ZED camera frame. | Subscribes: `/scan`, `/zed/zed_node/left/camera_info`  ·  Publishes: `/lidar_in_camera_frame` |
| [`lidar_to_zed_projection_debug.py`](scripts/lidar_to_zed_projection_debug.py) | Projects LiDAR hits into the rectified ZED image for visual debugging and calibration feedback. | Subscribes: `/scan`, `/zed/zed_node/left/image_rect_color`, `/zed/zed_node/left/camera_info` |
| [`lidar_zed_distance_comparison.py`](scripts/lidar_zed_distance_comparison.py) | Compares ZED depth measurements to LiDAR ranges to quantify agreement and spot drift. | Subscribes: `/scan`, `/zed/zed_node/depth/depth_registered` |

Each node is built with `rclpy`, making it easy to drop into a ROS 2 workspace and extend with additional publishers, diagnostics, or transforms. Adjust the extrinsic calibration parameters inside each script to match your physical sensor layout.

## Getting Started
1. **Provision the Jetson Orin Nano** with the desired JetPack release and install the ROS 2 distribution noted above.
2. **Install vendor SDKs** (ZED, RPLIDAR) and confirm their ROS drivers publish the expected topics.
3. **Clone this repository** into your ROS 2 workspace and mark the scripts as executable (`chmod +x scripts/*.py`).
4. **Run the nodes** using `ros2 run` or `ros2 launch` with your preferred packaging convention. Example:
   ```bash
   ros2 run <your_package> lidar_to_zed_pointcloud.py
   ```
5. **Visualize results** with RViz using the provided configuration files, and log findings in the [`docs/`](docs/) folder.

## Documentation Roadmap
Use the [`docs/`](docs/) directory to capture:
- Detailed hardware assembly notes, wiring diagrams, and calibration steps.
- ROS graph diagrams showing how perception, control, and planning nodes interact.
- Experiment logs comparing perception algorithms or sensor configurations (see [`docs/experiments/`](docs/experiments/)).
- Future work ideas (e.g., SLAM integration, autonomous navigation stack, machine learning perception).

Pair each experiment with supporting media and vendor references so the workflow is fully reproducible.

Feel free to expand this README with project milestones, demo videos, and personal reflections as the car progresses. This repository is intended to become a comprehensive portfolio piece highlighting your ROS 2 engineering skills.
