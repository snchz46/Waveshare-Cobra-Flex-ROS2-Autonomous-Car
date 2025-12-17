# Cobraflex ROS 2

ROS 2 integration for the Cobra Flex 4WD chassis. Includes a serial driver, LiDAR-based avoidance nodes, and URDF descriptions with the RPLIDAR A2M8 and ZED cameras (ZED Mini/ZED2).

## Quick install

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

Requirement: `pyserial` for the serial driver.

```bash
sudo apt install python3-serial
# or
python3 -m pip install pyserial
```

## Main nodes

- **`cobraflex_cmdvel_driver`** (`cobraflex_cmdvel_driver.py`): Converts `/cmd_vel` into JSON commands for the chassis microcontroller. Key parameters: `port`, `baud`, `max_speed_value`, `max_linear`, `max_angular`.
- **`lidar_avoidance_pid`** (`lidar_avoidance_pid_node.py`): Listens to `/scan` and publishes `/cmd_vel`, applying PID corrections to maintain a safe distance (`front_angle_deg`, `safe_distance`, `forward_speed`).
- **`cobraflex_ros_driver`**: Helper node to publish states and tie the serial driver into the ROS 2 stack.

### Run

```bash
ros2 run cobraflex cobraflex_cmdvel_driver
ros2 run cobraflex lidar_avoidance_pid
```

## Notable launch files

- `cobraflex_sensors.launch.xml`: Sensors only (RPLIDAR + ZED) for calibration and RViz tests.
- `cobraflex_driver.launch.xml`: Chassis driver for teleoperation.
- `cobraflex_bringup.launch.xml` / `.py`: Full stack (sensors + driver + avoidance).
- `cobraflex_manual.launch.xml`: Manual teleoperation profile.
- `cobraflex_automatic.launch.xml`: Autonomous navigation with avoidance.
- `cobraflex_description.launch.xml` / `cobraflex_zed_description.launch.py`: Publishes the URDF and TF tree.
- `mav1_gazebo.launch.xml`: MAV1 simulation in Gazebo.

## Package layout

- Python code in [`cobraflex/`](cobraflex/)
- Parameters and bridges in [`config/`](config/)
- URDF/Xacro in [`urdf/`](urdf/)
- RViz configuration in [`rviz/`](rviz/)
- ament tests in [`test/`](test/)
- Launch files in [`launch/`](launch/)

Keep these directories updated when you add nodes, sensors, or frame changes so the team can run them through the existing launch files.
