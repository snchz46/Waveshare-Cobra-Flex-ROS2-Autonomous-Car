# ROS 2 Workspace Skeleton

This folder provides a ROS 2 Humble workspace layout for adding your own packages. Add your packages under [`src/`](src/) with their `package.xml`, `setup.py`/`CMakeLists.txt`, and any `launch/` or `config/` assets.

## How to use
1. Drop or create packages inside [`src/`](src/).
2. From this `ros2_ws` directory run:
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```
3. Keep generated folders (`build/`, `install/`, `log/`) untracked.

## Notes
- Include a short README inside each package describing launch commands and dependencies.
- Large assets (bags, models) should be stored externally and referenced in your docs.
