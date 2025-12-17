# ROS 2 Workspace Skeleton

This folder provides a ROS 2 Humble workspace layout for adding your own packages. The current stack centers on the [`cobraflex`](src/cobraflex/) package, which includes:

- Python nodes for serial driving and LiDAR-based avoidance (`src/cobraflex/cobraflex/`)
- Launch files for sensing-only, manual, autonomous, and Gazebo profiles (`src/cobraflex/launch/`)
- URDF/Xacro descriptions of the MAV1 chassis, RPLIDAR A2M8, and ZED Mini (`src/cobraflex/urdf/`)
- RViz configs and ament tests to visualize and lint the package

## How to use
1. Ensure ROS 2 Humble is sourced (`source /opt/ros/humble/setup.bash`).
2. From this `ros2_ws` directory run:
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```
3. Launch the sample stack once drivers are installed:
   ```bash
   ros2 launch cobraflex_bringup cobraflex.launch.xml
   ```
4. Keep generated folders (`build/`, `install/`, `log/`) untracked.

## Notes
- Update the `cobraflex` entry points and parameters as you add real control logic.
- Large assets (bags, models) should be stored externally and referenced in your docs.
