# Robot Description

URDF and Xacro assets describing the Cobra Flex platform and its sensors:

- `mav1.urdf` / `mav1.urdf.xacro`: Full robot description for the MAV1 configuration, combining base, LiDAR, and camera mounts.
- `mav1_base.urdf.xacro`: Base chassis geometry and joints without payload-specific attachments.
- `rplidar_a2.urdf.xacro`: RPLIDAR A2M8 model and mount offset.
- `zed_m.urdf.xacro`: ZED Mini camera model and placement.

Regenerate the compiled `mav1.urdf` from the Xacro macros when you adjust frames or add sensors:

```bash
ros2 run xacro xacro mav1.urdf.xacro -o mav1.urdf
```

Keep transforms consistent with the launch files and update RViz configs in `../rviz/` after significant changes.
