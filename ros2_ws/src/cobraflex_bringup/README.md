# Cobraflex Bringup

This package provides launch files that start the full Cobra chassis control stack. It orchestrates:

- The Cobraflex control nodes that publish JSON commands for the chassis.
- The RPLIDAR runner needed to produce `scan` topics for obstacle sensing.
- The ZED camera runner so the control nodes receive depth and visual data.

Use these launch files to bring up a complete control session with all required hardware drivers and nodes in one command.
