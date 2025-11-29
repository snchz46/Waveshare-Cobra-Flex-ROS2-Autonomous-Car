# Cobraflex

Cobraflex contains the ROS 2 nodes responsible for driving the Cobra chassis. These nodes listen to:

- `scan` data from the LiDAR to understand nearby obstacles.
- ZED camera topics for depth and visual context.

Based on the incoming perception data, the nodes publish JSON-formatted control commands that steer and propel the chassis. Use this package to iterate on motion logic without touching the launch orchestration.
