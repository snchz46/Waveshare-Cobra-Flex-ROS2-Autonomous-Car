# LiDAR Plugin Basics

Use this folder for LiDAR simulation notes and snippets.

## Core Configuration Items

- Sensor type: `lidar` / `gpu_lidar`
- Horizontal samples and angular limits
- Range (`min`, `max`) and resolution
- Update rate (Hz)
- Output topic for scan/point cloud bridge

## Validation Checks

- Scan covers expected field of view.
- Obstacles appear at reasonable distances.
- Topic rate matches configured update rate.
- Frame ID aligns with LiDAR link frame.
