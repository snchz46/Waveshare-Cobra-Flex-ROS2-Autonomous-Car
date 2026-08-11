# cobraflex/maps

Occupancy grids for Nav2. `navigation.launch.py` defaults to
`cobraflex_map.yaml` in this directory, so a map saved here is picked up with
no launch argument.

The maps themselves are not tracked in git: they are site-specific, they are
regenerated whenever the environment changes, and a PGM of a large room is
large and compresses badly. Build one before the first navigation run.

## Saving a map

With `mapping.launch.py` (simulation) or `cobraflex_mapping.launch.py`
(hardware) running and the environment fully explored:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/cobraflex/maps/cobraflex_map
```

That writes `cobraflex_map.pgm` + `cobraflex_map.yaml`. Rebuild so the pair
reaches the install share:

```bash
colcon build --packages-select cobraflex --symlink-install
```

## Using a map from elsewhere

```bash
ros2 launch cobraflex navigation.launch.py map:=/absolute/path/to/other_map.yaml
```

## Resolution

`slam_toolbox_mapping.yaml` maps at `resolution: 0.01` (1 cm/cell), finer than
the 0.025 m the Nav2 costmaps run at (`config/nav2_params.yaml`). The static
layer resamples, so this is not an error, but a 1 cm map of a large space gets
big fast — raise the SLAM resolution to 0.02–0.05 for anything beyond a room
if the PGM becomes unwieldy.
