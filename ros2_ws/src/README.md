# Packages go here

Place each ROS 2 package in this `src/` directory. Keep package manifests (`package.xml`) and build files (`setup.py` or `CMakeLists.txt`) alongside any `launch/` and `config/` folders.

## Included packages

- **cobraflex**: Control nodes for the Cobra chassis that subscribe to LiDAR `scan` data and ZED camera topics to publish JSON-based chassis commands.
- **cobraflex_bringup**: Launch files that start the Cobra control nodes together with hardware-specific runners for RPLIDAR and the ZED camera.
