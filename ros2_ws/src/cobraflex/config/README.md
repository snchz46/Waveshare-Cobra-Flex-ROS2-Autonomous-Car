# Configuration Files

Parameter and bridge configuration lives in this directory.

- `gazebo_bridge.yaml`: Example parameter file for ROS3 to Gazebo topic bridging. Tune the mapping rules here when simulating the Cobra Flex or forwarding topics between namespaces.

Add new YAML files for PID tuning, sensor calibration, or launch-time defaults, and reference them from the relevant launch files under `../launch/`.
