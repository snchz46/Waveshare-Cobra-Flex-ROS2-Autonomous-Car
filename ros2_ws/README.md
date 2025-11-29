# ROS 2 Workspace Skeleton

This folder provides a ROS 2 Humble workspace layout for adding your own packages. Two starter packages live under [`src/`](src/):

- [`cobraflex`](src/cobraflex/) – placeholder control node that currently spins and logs a startup message.
- [`cobraflex_bringup`](src/cobraflex_bringup/) – launch and config scaffolding to start LiDAR, ZED, and the cobraflex node together.

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
