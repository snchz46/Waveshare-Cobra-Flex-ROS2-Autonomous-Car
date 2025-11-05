# Jetson Orin Nano Autonomous Car Portfolio

> **Status:** Work in progress — this repository documents the ongoing integration of a ZED stereo camera and an RPLIDAR sensor on a Waveshare Cobra Flex platform powered by an NVIDIA Jetson Orin Nano.

## Project Overview
This project showcases my ROS 2 development and sensor fusion work while building an autonomous RC car. The primary goal is to create a reproducible reference for deploying a Jetson Orin Nano with stereo vision (ZED) and 2D LiDAR (RPLIDAR) to perceive the environment, validate sensor agreement, and provide the foundation for autonomous navigation on the Waveshare Cobra Flex chassis.

The repository collects:
- **ROS 2 nodes** for translating LiDAR scans into the ZED camera frame, projecting them onto ZED images for debugging, and numerically comparing depth readings between the sensors.
- **Visualization assets** for fusing point clouds in RViz.
- **Documentation templates** to capture hardware configuration, ROS graph design, calibration procedures, and lessons learned as the project matures.

## Hardware Platform
| Component | Details |
| --- | --- |
| Compute | NVIDIA Jetson Orin Nano (8 GB) |
| Chassis | Waveshare Cobra Flex |
| Stereo Camera | ZED ___ (model TBD) |
| LiDAR | RPLIDAR ___ (model TBD) |
| IMU / Additional Sensors | _Add details here_ |
| Motor Controller | _Add details here_ |
| Power System | _Add details here_ |

> _Fill in the blank rows above with the final bill of materials once confirmed._

## ROS 2 Environment
- **Distribution:** _ROS 2 ___ (e.g., Humble, Iron — specify exact release)_
- **Middleware:** _CycloneDDS / FastDDS — confirm configuration_
- **ZED SDK:** _Version TBD_
- **RPLIDAR SDK / Driver:** _Version TBD_

Record any custom workspace overlays, `colcon` packages, or launch files in the [`docs/`](docs/) directory as you refine the system.

## Repository Structure
```
├── scripts/
│   ├── lidar_to_zed_pointcloud.py       # Publishes LiDAR data as a PointCloud2 in the ZED camera frame
│   ├── lidar_to_zed_projection_debug.py # Projects LiDAR points into the ZED image for visual alignment debugging
│   ├── lidar_zed_distance_comparison.py # Compares ZED depth output against LiDAR ranges for accuracy checks
│   ├── windows_yolov8_cam_sub_ZED.py    # Windows-side YOLOv8 subscriber for ZED video (experiment)
│   ├── Pointcloud_Fusion.rviz           # RViz configuration for visualizing fused sensor data
│   ├── Lidar_ZED_Distance.png           # Sample comparison plot
│   └── pointcloud fusion.png            # Example fused point cloud capture
├── docs/
│   └── README.md                        # Template for in-depth documentation (calibration, setup, etc.)
├── LICENSE
└── README.md (this file)
```

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
- Experiment logs comparing perception algorithms or sensor configurations.
- Future work ideas (e.g., SLAM integration, autonomous navigation stack, machine learning perception).

Feel free to expand this README with project milestones, demo videos, and personal reflections as the car progresses. This repository is intended to become a comprehensive portfolio piece highlighting your ROS 2 engineering skills.
