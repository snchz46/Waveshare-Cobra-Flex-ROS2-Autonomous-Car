# Cobraflex Bringup

This package provides launch files to start a full Cobra Flex test session. The default launch file does the following:

- Includes the Slamtec RPLIDAR launch to expose `/scan` from the A2M8 sensor.
- Starts the Stereolabs ZED wrapper so depth and color topics are available.
- Spawns the `cobraflex` node (currently a placeholder) so you can verify the workspace builds cleanly.

Usage (from `ros2_ws`):

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch cobraflex_bringup cobraflex.launch.xml
```

Notes:

- The `config/cobraflex_param.yaml` file is intentionally minimal; add parameters there as you flesh out the control node.
- Adjust the `<node exec="...">` target in the launch file if you add new entry points or rename the control executable.
- The included camera args in the launch file are commented out; uncomment and tweak them to match your ZED model and desired resolution.
