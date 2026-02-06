# Resource Index

This directory contains the `cobraflex` marker file used by `ament_index` to locate package resources at runtime. Do not remove it—`ros2` tooling relies on this path to resolve launch files, parameters, and installed assets.

If you add installable resources (for example, new launch files or YAML), make sure the package installs correctly with `setup.py` and that `ament_index` can find them from here.
