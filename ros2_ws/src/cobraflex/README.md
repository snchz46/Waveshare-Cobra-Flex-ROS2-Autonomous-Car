# Cobraflex

Cobraflex houses the ROS 2 nodes that will eventually drive the Waveshare Cobra Flex chassis. The current implementation is a minimal placeholder node (`cobraflex_node`) that spins and logs a startup message so you can validate the package wiring before adding subscribers and publishers.

When you start adding control logic, consider wiring in:

- `scan` data from the LiDAR for obstacle awareness.
- ZED camera topics (depth/color) for richer perception.
- Parameters for chassis-specific tuning (PID gains, speed limits, steering limits).

Recommended next steps:

1. Expand `cobraflex/cobraflex_node.py` with your publishers/subscribers and timers.
2. Update `setup.py` and `entry_points` if you add additional nodes.
3. Commit a configuration file under `config/` (and load it from the bringup package) once parameters are defined.
