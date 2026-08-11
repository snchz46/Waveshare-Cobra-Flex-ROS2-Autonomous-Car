# cobraflex/worlds

SDF worlds for Gazebo Harmonic. Every one is loadable with:

```bash
ros2 launch cobraflex gazebo_mesh.launch.py world:=<name> gui:=true
```

`gazebo_mesh.launch.py` resolves a bare token to
`worlds/lane_following_<token>.world`, so `world:=oval_complex` and
`world:=lane_following_oval_complex` are the same thing. A value containing a
path separator, or ending in `.world` / `.sdf`, is passed through unchanged —
which is how you reach the non–lane-following worlds below.

## Lane-following worlds

The track is a single textured plane; the appearance of the road *is* the
texture. That is what makes these useful for probing the camera controllers:
the geometry is identical across a family, so any difference in behaviour comes
from perception and not from the path.

| World | Texture | What it is for |
| --- | --- | --- |
| `lane_following_oval_simple` | `tracks/complex_a.png` | Gentler circuit, the easy baseline |
| `lane_following_oval_complex` | `tracks/complex_b.png` | Default in `gazebo_mesh.launch.py` and `lane_keeper_gazebo.launch.py` |
| `lane_following_complex_b_flipH` | `tracks/complex_b_flipH.png` | Same circuit mirrored horizontally — checks for a steering bias |
| `lane_following_complex_b_flipV` | `tracks/complex_b_flipV.png` | Mirrored vertically, same purpose |
| `lane_following_complex_b_worn_25` | `tracks/complex_b_worn_25.png` | Paint degraded 25% |
| `lane_following_complex_b_worn_50` | `tracks/complex_b_worn_50.png` | Paint degraded 50% |
| `lane_following_complex_b_worn_75` | `tracks/complex_b_worn_75.png` | Paint degraded 75% — the hardest of the family |
| `lane_following_complex_b_gaps` | `tracks/complex_b_gaps.png` | Line dropouts, tests how the estimator holds through a gap |
| `lane_following_oval` | `road_curves/` + `road_textures/` tiles | Tiled construction (5 models) rather than one plane, for geometry work |

The `worn_25/50/75` triple is the intended degradation sweep: run the same
controller across the three and the intervention rate against wear is the
result.

## Other worlds

| World | Contents |
| --- | --- |
| `obstacles.world` | Obstacle course, the default for `gazebo.launch.py`. Used for SLAM and Nav2 demos |
| `straight_road.world` | Straight two-segment road, for controller step responses |
| `road_carpet.world` | Single road plane |
| `empty.world` | Ground plane and physics only, for URDF bring-up debugging |
| `test_world.sdf` | Sandbox with 14 models |

## Textures

Road textures are generated, not hand-painted, by the scripts under
`materials/road_assets/`:

```bash
python3 materials/road_assets/road_tiles/make_road_tiles.py
python3 materials/road_assets/road_textures/make_road_textures.py
python3 materials/road_assets/road_curves/make_road_curves.py
python3 materials/road_assets/road_variants/make_road_variants.py
```

PNG names carry their real size in metres (`tile_zebra_0p52m_x_4m.png` is
0.52 m × 4 m), so a tile can be placed without guessing at UV scale.

Worlds reference textures relatively (`../materials/road_assets/...`), which
resolves against the world file's own location — in the source tree *and* in
the install share, since `setup.py` installs both under `share/cobraflex/`.
Keep it relative; an absolute path will work locally and break on every other
machine.

## Adding a world

1. Copy the closest existing `lane_following_*.world`.
2. Point its `<albedo_map>` at the new texture, relative as above.
3. Add the texture's directory to `data_files` in `setup.py` if it is a new one.
4. `colcon build --packages-select cobraflex --symlink-install`.
5. Check it parses before launching: `gz sdf -k worlds/<name>.world`.
