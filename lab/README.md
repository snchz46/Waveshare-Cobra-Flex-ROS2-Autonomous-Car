# Lab

Code for the three lab sessions of the module ADAS (SE4ADS) · SLAM ·
Motion Planning · Computer Vision. Each session follows a workbook in
Moodle; this directory contains the code completed in it.

| Session | Files | Task |
| --- | --- | --- |
| 1 · ROS 2 basics: nodes, topics, launch files and the digital twin | [`session1/`](session1/) | Create the package `cobraflex_lab`, complete twelve gaps (A to L) in two nodes and a launch file, and run them in Gazebo and on the robot |
| 2 · LiDAR, mapping and navigation (in preparation) | [`session2/emergency_brake_node.py`](session2/emergency_brake_node.py) | Complete eight gaps (A to H) of the emergency-braking node |
| 3 · Reinforcement learning (in preparation) | | |

## Session 1: the package cobraflex_lab

In Session 1 each pair creates its own ROS 2 package, `cobraflex_lab`, in
a second workspace, `~/lab_ws`, on top of this repository's workspace
`~/ros2_ws`. The four files of `session1/` are copied into it:

| File | Destination in `~/lab_ws/src/cobraflex_lab/` | Gaps |
| --- | --- | --- |
| [`circle_driver.py`](session1/circle_driver.py) | `cobraflex_lab/` | A to E |
| [`obstacle_monitor.py`](session1/obstacle_monitor.py) | `cobraflex_lab/` | F to J |
| [`session1.launch.py`](session1/session1.launch.py) | `launch/` | K and L |
| [`session1.yaml`](session1/session1.yaml) | `config/` | none |

```bash
mkdir -p ~/lab_ws/src && cd ~/lab_ws/src
ros2 pkg create --build-type ament_python cobraflex_lab \
    --dependencies rclpy geometry_msgs sensor_msgs std_msgs
# copy the four files, register the programs and data files in setup.py (workbook, Parts 2, 4 and 5)
cd ~/lab_ws && colcon build --symlink-install && source install/setup.bash

ros2 run cobraflex_lab circle_driver --ros-args -p use_sim_time:=true   # Gazebo
ros2 launch cobraflex_lab session1.launch.py use_sim_time:=false       # physical robot
```

An open gap stops the program with an error that names it, for example
`NameError: name '___A___' is not defined`. The package stays on the lab
PC for Sessions 2 and 3.

## Session 2 script

`session2/emergency_brake_node.py` is a standalone Python file, not a
package, and requires no build:

```bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/lab/session2
python3 emergency_brake_node.py --ros-args -p use_sim_time:=true   # Gazebo
python3 emergency_brake_node.py                                    # physical robot
```

Solutions are not included in this repository.
