# Cobraflex ROS2

ROS 2 integration for the Cobra Flex 4WD chassis with:

- JSON driver over serial
- LIDAR-based obstacle avoidance node (PID + ramping)
- RPLIDAR A2M8
- ZED camera (ZED2 / ZED Mini)

## Package structure

Quick overview of the main folders and files:

```
ros2_ws/src/cobraflex
├── cobraflex/          # Python nodes and package utilities
├── config/             # Configuration parameters and YAML files
├── launch/             # Launch files (sensors, driver, bringup, simulation)
├── resource/           # ament_index marker for package installation
├── rviz/               # RViz profiles for quick visualization
├── test/               # Package style and health checks
├── urdf/               # Robot/sensor URDF/Xacro descriptions
├── worlds/             # Simulation worlds (Gazebo)
├── package.xml         # ROS 2 package metadata
├── setup.py            # Python node entry points (`ros2 run cobraflex <node>`)
├── setup.cfg           # Style/installation configuration
└── mav1.urdf           # Generated URDF (see `urdf/README.md`)
```

For specific details, see the README files in each subfolder (for example `launch/README.md`, `urdf/README.md`, `cobraflex/README.md`).

## Installation

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Guide

Make sure pyserial is installed:

```bash
sudo apt install python3-serial
# or
python3 -m pip install pyserial
```

Nodes
cobraflex_cmdvel_driver

Converts /cmd_vel into JSON commands for the Cobra Flex chassis.

```bash
ros2 run cobraflex cobraflex_cmdvel_driver
```

Parameters:

- port (string): serial port, e.g. /dev/ttyACM2
- baud (int): baudrate, default 115200
- max_speed_value (int): power scaling for L/R (e.g. 100)
- max_linear (float): expected max linear velocity [m/s]
- max_angular (float): expected max angular velocity [rad/s]

lidar_avoidance_pid

Node that listens to /scan and publishes /cmd_vel with obstacle avoidance.

```bash
ros2 run cobraflex lidar_avoidance_pid
```

Main parameters:

- front_angle_deg (float): front angle ± in degrees
- safe_distance (float): minimum safe distance [m]
- forward_speed (float): max linear speed [m/s]

Launch files
Sensors only

```bash
ros2 launch cobraflex cobraflex_sensors.launch.xml
```

Chassis + avoidance

```bash
ros2 launch cobraflex cobraflex_driver.launch.xml
```

Full bringup (sensors + driver + avoidance)

```bash
ros2 launch cobraflex cobraflex_bringup.launch.xml
```
