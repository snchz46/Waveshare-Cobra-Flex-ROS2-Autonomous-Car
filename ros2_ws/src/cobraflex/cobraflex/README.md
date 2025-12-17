# Python Nodes

ROS 2 Python nodes for the Cobra Flex chassis live here:

- `cobraflex_cmdvel_driver.py`: Translates `/cmd_vel` geometry messages into JSON commands for the on-board microcontroller over serial, with configurable port, baud rate, and scaling parameters.
- `cobraflex_ros_driver.py`: Helper to bridge chassis feedback and publish drive status topics alongside the command interface.
- `lidar_avoidance_pid_node.py`: Implements a LiDAR-based obstacle avoidance loop that listens to `/scan`, applies PID-style heading corrections, and publishes safe velocity commands.

Extend this package with additional nodes (for example, telemetry publishers or diagnostics) and register their entry points in `setup.py` so `ros2 run cobraflex <node>` resolves correctly.
