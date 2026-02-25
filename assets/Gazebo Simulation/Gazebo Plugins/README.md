# Gazebo Plugins

This section organizes baseline Gazebo sensor plugin references by sensor type.

## Subfolders

- [LiDAR](./LiDAR/README.md): ray / gpu_lidar sensor configuration notes.
- [Camera](./Camera/README.md): camera topic, image rate, and frame setup notes.
- [IMU](./IMU/README.md): inertial data publishing and noise configuration notes.

## Integration Pattern

For each sensor plugin:

1. Attach the sensor to an existing URDF/SDF link.
2. Set update rate and topic names.
3. Confirm frame IDs in TF.
4. Validate message output while robot is static and moving.
