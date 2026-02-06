# Python Nodes

ROS 2 Python nodes for the Cobra Flex chassis live here. This folder is the Python package installed by `setup.py`.

Key files:

- `cobraflex_cmdvel_driver.py`: Translates `/cmd_vel` messages into JSON commands for the microcontroller over serial, with port, baudrate, and scaling parameters.
- `cobraflex_ros_driver.py`: Bridges chassis feedback and publishes status topics alongside the command interface.
- `lidar_avoidance_pid_node.py`: Obstacle avoidance node; listens to `/scan`, applies PID-style correction, and publishes safe velocities.
- `__init__.py`: Defines the Python package and enables internal imports.

To add new nodes (telemetry, diagnostics, etc.), create the file here and register the entry point in `setup.py` so `ros2 run cobraflex <node>` can find it.
