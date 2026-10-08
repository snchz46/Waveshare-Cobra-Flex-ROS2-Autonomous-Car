# Lab

Code for the three lab sessions of the module ADAS (SE4ADS) · SLAM ·
Motion Planning · Computer Vision. Students work through one workbook per
session; this folder holds the code they complete in it.

| Session | File | What students do |
| --- | --- | --- |
| 1 · ROS 2 basics and emergency braking | [`session1/emergency_brake_node.py`](session1/emergency_brake_node.py) | Fill eight gaps (A to H), then run the node in Gazebo and on the car |

Sessions 2 and 3 use the packages as they are: copies of
`nav2_params.yaml` that students make in the lab, and the existing lane
keeping and safety cage launch files.

## Running a lab script

The scripts are plain Python files, not ROS packages, so nothing needs to
be built:

```bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/lab/session1
python3 emergency_brake_node.py --ros-args -p use_sim_time:=true   # Gazebo
python3 emergency_brake_node.py                                    # car
```

A gap that is still open stops the script with an error that names it,
for example `NameError: name '___A___' is not defined`.

The solutions are not part of this repository.
