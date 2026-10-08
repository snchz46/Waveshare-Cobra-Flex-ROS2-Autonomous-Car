# Lab

Code for the three lab sessions of the module ADAS (SE4ADS) · SLAM ·
Motion Planning · Computer Vision. Each session follows a workbook; this
directory contains the code completed in it.

| Session | File | Task |
| --- | --- | --- |
| 1 · ROS 2 basics and emergency braking | [`session1/emergency_brake_node.py`](session1/emergency_brake_node.py) | Complete eight gaps (A to H) and run the node in Gazebo and on the robot |

Sessions 2 and 3 use the existing packages: working copies of
`nav2_params.yaml` created during the lab, and the lane-keeping and
safety-cage launch files.

## Execution of a lab script

The scripts are standalone Python files, not ROS packages, and require no
build:

```bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/lab/session1
python3 emergency_brake_node.py --ros-args -p use_sim_time:=true   # Gazebo
python3 emergency_brake_node.py                                    # physical robot
```

An open gap stops the script with an error that names it, for example
`NameError: name '___A___' is not defined`.

Solutions are not included in this repository.
