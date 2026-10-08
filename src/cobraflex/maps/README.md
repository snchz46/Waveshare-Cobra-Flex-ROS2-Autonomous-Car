# cobraflex/maps

Occupancy grids for Nav2. `navigation.launch.py` uses `cobraflex_map.yaml` in
this directory by default, so a map saved here requires no launch argument.

Maps are not tracked in git: they are site-specific, they are regenerated when
the environment changes, and the PGM of a large room is large and compresses
poorly. A map must be created before the first navigation run.

## Saving a map

With `mapping.launch.py` (simulation) or `cobraflex_mapping.launch.py`
(hardware) running and the environment explored:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/cobraflex/maps/cobraflex_map
```

This writes `cobraflex_map.pgm` and `cobraflex_map.yaml`. A rebuild installs
both files into the package share:

```bash
colcon build --packages-select cobraflex --symlink-install
```

## Map from another location

```bash
ros2 launch cobraflex navigation.launch.py map:=/absolute/path/to/other_map.yaml
```

## Resolution

`slam_toolbox_mapping.yaml` maps at `resolution: 0.01` (1 cm/cell), finer than
the 0.025 m of the Nav2 costmaps (`config/nav2_params.yaml`). The static layer
resamples the map, so the difference is intended. A 1 cm map of a large area
grows quickly; for areas larger than a room, the SLAM resolution is set to
0.02–0.05 m.
