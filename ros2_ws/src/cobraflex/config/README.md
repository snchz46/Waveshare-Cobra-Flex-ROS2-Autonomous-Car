# Configuration Files

Parameter and bridge configuration lives in this directory.

Highlighted files:

- `gazebo_bridge.yaml`: Parameter file for the ROS 2 ↔ Gazebo bridge. Update the mappings here when simulating Cobra Flex or connecting topics across namespaces.

You can add new YAML files for:

- PID tuning and safety limits.
- Sensor calibration (LiDAR, camera, IMU).
- Default values for launch files.

Reference them from the launch files in `../launch/` to keep configuration centralized.
