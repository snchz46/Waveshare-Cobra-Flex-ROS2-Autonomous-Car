# Jetson Orin Nano Autonomous Car with Waveshare Cobra Flex Chassis

> **Status:** Work in progress

![Jetson Orin Nano Cobra Flex build on workbench](assets/photos/Testing%20build.jpg)

## Project Overview
This project showcases ongoing ROS 2 development and sensor fusion work while building an autonomous 1:14 car. The primary goal is to create a reproducible reference for deploying a Jetson Orin Nano with stereo vision (ZED) and 2D LiDAR (RPLIDAR) to perceive the environment, validate sensor agreement, and provide the foundation for autonomous navigation on the Waveshare Cobra Flex chassis.

The repository collects:
- **ROS 2 nodes** for translating LiDAR scans into the ZED camera frame, projecting them onto ZED images for debugging, and numerically comparing depth readings between the sensors.
- **Visualization assets** for fusing point clouds in RViz.
- **Media-rich documentation** with inline photo galleries to showcase hardware iterations, calibration setups, and experiment highlights.

## Quick Look
| Prototype highlights | Sensor fusion preview |
| --- | --- |
| ![Initial wiring pass with ZED and RPLIDAR mounted](assets/photos/Testing%20build%202.jpg) | ![LiDAR and ZED depth comparison plot](assets/photos/Lidar_ZED_Distance.png) |

> 💡 **Tip:** Every Markdown file now links directly to curated photos inside [`assets/photos/`](assets/photos/). Use relative paths (for example, `![caption](assets/photos/pointcloud%20fusion.png)`) so renders work across GitHub, MkDocs, or PDF exports without extra hosting steps.

## Hardware Platform & Bill of Materials
| Component | Details | Vendor / Reference |
| --- | --- | --- |
| Compute | NVIDIA Jetson Orin Nano (8 GB) developer kit | [NVIDIA](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/) |
| Chassis | Waveshare Cobra Flex | [Waveshare](https://www.waveshare.com/product/ai/robots/mobile-robots/cobra-flex.htm?sku=31326) |
| Stereo Camera | ZED Mini stereo module | [Stereolabs](https://store.stereolabs.com/products/zed-mini) |
| LiDAR | RPLIDAR A2M8 360° laser scanner | [Slamtec](https://www.slamtec.com/en/Lidar/A2) |
| IMU / Additional Sensors | ZED Mini integrated IMU | — |
| Motor Controller | Waveshare Cobra Flex driver (dual TB6612FNG + PCA9685) | [Waveshare board details](https://www.waveshare.com/wiki/Cobra_Flex#Driver_Board) |
| Power System | XT-27000DC-AO-PA power bank with uninterrupted DC adapter | [XT-27000DC-AO-PA Power Bank](https://www.amazon.de/XTPower-XT-27000DC-AO-PA-Uninterrupted-Adapter-Included/dp/B09S6F56T4/261-0714907-2939417?pd_rd_w=XYwSS&content-id=amzn1.sym.13dbab83-f61c-4000-b9ab-184f02ce8fa2&pf_rd_p=13dbab83-f61c-4000-b9ab-184f02ce8fa2&pf_rd_r=HJBKMDSMEDR9M3958XXJ&pd_rd_wg=J09o0&pd_rd_r=c8a61ded-40c8-433c-bb0c-52fed8ea14df&pd_rd_i=B09S6F56T4&th=1) |
| Fasteners & Mounts | Custom camera/LiDAR brackets, M2/M3 hardware | Document specific sources as mounts are finalized |

![Side-by-side comparison of early and current sensor mounting plates](assets/photos/Physical%20comparison.jpg)

> _To Do: add rows for cables, storage, and networking accessories as the build matures._

## ROS 2 Environment
- **Distribution:** ROS 2 Humble on Ubuntu 22.04
- **ZED SDK:** 5.1
- **ZED ROS 2 Package:** [ZED ROS 2 Wrapper](https://github.com/stereolabs/zed-ros2-wrapper)
- **RPLIDAR Driver:** [RPLIDAR ROS 2 Package](https://github.com/Slamtec/rplidar_ros/tree/ros2)

Record any custom workspace overlays, `colcon` packages, or launch files in the [`docs/`](docs/) directory as you refine the system. Supplement textual notes with build photos or RViz screenshots for each milestone.

## Repository Structure
```
├── assets/
│   ├── 3d-models/                    # Mounting hardware and chassis accessory CAD files
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

- **Photos:** Store build imagery inside [`assets/photos/`](assets/photos/). Organize by milestone (`assets/photos/2024-setup/`) and add a `README.md` or captions section for context. Embed your favorite shots directly into Markdown so updates stand out in Git diffs.
- **Videos:** Save drive tests and demos in [`assets/videos/`](assets/videos/). Include session summaries (timestamps, highlights, hosted mirrors) in each subfolder.
- **3D Models:** Collect custom brackets, plates, and printable sensor mounts in [`assets/3d-models/`](assets/3d-models/). Add a brief README alongside each design describing mounting points, print orientation, and any required fasteners so others can replicate your setup.
- **Media Log:** Update [`docs/media-log.md`](docs/media-log.md) with every new clip or gallery so readers can jump straight to relevant material.

Reference these assets from the sections above—for example, embed before/after shots in the hardware table or link calibration recordings alongside the ROS 2 setup instructions.

## Key ROS 2 Nodes
| Script | Purpose | Topics |
| --- | --- | --- |
| [`lidar_to_zed_pointcloud.py`](scripts/lidar_to_zed_pointcloud.py) | Converts `/scan` LaserScan data into `sensor_msgs/PointCloud2` aligned with the ZED camera frame. | Subscribes: `/scan`, `/zed/zed_node/left/camera_info` · Publishes: `/lidar_in_camera_frame` |
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
5. **Visualize results** with RViz using the provided configuration files, and log findings in the [`docs/`](docs/) folder. Add inline photos or screenshots whenever they clarify a step.

## Documentation Roadmap
Use the [`docs/`](docs/) directory to capture:
- Detailed hardware assembly notes, wiring diagrams, and calibration steps.
- ROS graph diagrams showing how perception, control, and planning nodes interact.
- Experiment logs comparing perception algorithms or sensor configurations (see [`docs/experiments/`](docs/experiments/)).
- Future work ideas (e.g., SLAM integration, autonomous navigation stack, machine learning perception).

Pair each experiment with supporting media and vendor references so the workflow is fully reproducible.

Feel free to expand this README with project milestones, demo videos, and personal reflections as the car progresses. This repository is intended to become a comprehensive portfolio piece highlighting your ROS 2 engineering skills.
