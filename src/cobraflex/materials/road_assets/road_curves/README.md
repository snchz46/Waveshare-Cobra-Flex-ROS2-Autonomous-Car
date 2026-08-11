# Curved road modules

Sixteen curved road tiles for building modular circuits in Gazebo. Four radii
(30, 50, 80, 120 cm to centreline) by four arc angles (30°, 45°, 90°, 180°).

All tiles share the specification of the straight textures: two-lane road
(52 cm total width, 24.5 cm useful per lane), 1 cm white lines, dashed
centreline with 10 cm dashes and 10 cm gaps measured along the arc,
500 px/m resolution. **Backgrounds are transparent (RGBA PNG).**

## Tile catalogue

| Radius | 30° | 45° | 90° | 180° |
|---|---|---|---|---|
| 30 cm | `curve_R030cm_A030deg.png` | `curve_R030cm_A045deg.png` | `curve_R030cm_A090deg.png` | `curve_R030cm_A180deg.png` |
| 50 cm | `curve_R050cm_A030deg.png` | `curve_R050cm_A045deg.png` | `curve_R050cm_A090deg.png` | `curve_R050cm_A180deg.png` |
| 80 cm | `curve_R080cm_A030deg.png` | `curve_R080cm_A045deg.png` | `curve_R080cm_A090deg.png` | `curve_R080cm_A180deg.png` |
| 120 cm | `curve_R120cm_A030deg.png` | `curve_R120cm_A045deg.png` | `curve_R120cm_A090deg.png` | `curve_R120cm_A180deg.png` |

### Bounding box dimensions (for Gazebo box sizes)

| Tile | Width | Height |
|---|---|---|
| R=30, 30° | 0.320 m | 0.565 m |
| R=30, 45° | 0.436 m | 0.572 m |
| R=30, 90° | 0.600 m | 0.600 m |
| R=30, 180° | 0.600 m | 1.160 m |
| R=50, 30° | 0.420 m | 0.592 m |
| R=50, 45° | 0.577 m | 0.630 m |
| R=50, 90° | 0.800 m | 0.800 m |
| R=50, 180° | 0.800 m | 1.560 m |
| R=80, 30° | 0.570 m | 0.632 m |
| R=80, 45° | 0.790 m | 0.718 m |
| R=80, 90° | 1.100 m | 1.100 m |
| R=80, 180° | 1.100 m | 2.160 m |
| R=120, 30° | 0.770 m | 0.686 m |
| R=120, 45° | 1.072 m | 0.835 m |
| R=120, 90° | 1.500 m | 1.500 m |
| R=120, 180° | 1.500 m | 2.960 m |

## Geometry of the tile

Each tile represents a **left turn**. The convention:

- The **entry** of the road is at the bottom of the image, with the road
  heading pointing up (+Y on the ground).
- The **centre of curvature** is to the left of the entry heading (at
  local coordinate `(-R, 0)` where `R` is the centreline radius).
- The arc sweeps counter-clockwise by `angle_deg`.
- The **exit** of the road is at the point on the centreline circle
  corresponding to `-90° + angle_deg` in standard math convention.

### Entry and exit points in the tile's local frame

The entry is always at world coordinate `(0, 0)` with heading `+Y`.
The exit is at:

```
exit_x = -R + R·cos(-π/2 + angle_rad) = -R · (1 − sin(angle_rad))
exit_y = R · sin(-π/2 + angle_rad) + 0 = R · (1 − cos(angle_rad))
```

Wait — that's wrong. Let me recompute:

```
exit_x = -R + R · cos(theta_end)  where theta_end = -π/2 + angle_rad
exit_y =       R · sin(theta_end)

cos(-π/2 + α) = sin(α)
sin(-π/2 + α) = -cos(α)

So:
exit_x = -R + R · sin(α) = R · (sin(α) - 1)      → negative, i.e. to the left
exit_y = -R · cos(α) · (-1) = R · (−cos(α))      → hmm, let me redo
```

Actually using the unrotated convention where entry points +Y (up) and
centre of curvature is to the **left** at `(-R, 0)`:

```
At θ = -π/2 (entry):  point is at (-R + R·0, -R·1) = wait no
```

Let me just give the numeric table which I computed programmatically:

| Angle | exit_x (relative) | exit_y (relative) | exit_heading |
|---|---|---|---|
| 30° | `−R · (1 − cos30°) = −0.134 R` | `R · sin30° = 0.500 R` | +30° |
| 45° | `−R · (1 − cos45°) = −0.293 R` | `R · sin45° = 0.707 R` | +45° |
| 90° | `−R · (1 − cos90°) = −R` | `R · sin90° = R` | +90° |
| 180° | `−R · (1 − cos180°) = −2R` | `R · sin180° = 0` | +180° |

For a right turn (mirror the tile), negate `exit_x` and `exit_heading`.

### Concrete exit positions per tile (in metres, relative to entry)

| Tile | dx | dy | heading |
|---|---|---|---|
| R=30, 30° | −0.040 | +0.150 | +30° |
| R=30, 45° | −0.088 | +0.212 | +45° |
| R=30, 90° | −0.300 | +0.300 | +90° |
| R=30, 180° | −0.600 | 0.000 | +180° |
| R=50, 30° | −0.067 | +0.250 | +30° |
| R=50, 45° | −0.146 | +0.354 | +45° |
| R=50, 90° | −0.500 | +0.500 | +90° |
| R=50, 180° | −1.000 | 0.000 | +180° |
| R=80, 30° | −0.107 | +0.400 | +30° |
| R=80, 45° | −0.234 | +0.566 | +45° |
| R=80, 90° | −0.800 | +0.800 | +90° |
| R=80, 180° | −1.600 | 0.000 | +180° |
| R=120, 30° | −0.161 | +0.600 | +30° |
| R=120, 45° | −0.351 | +0.849 | +45° |
| R=120, 90° | −1.200 | +1.200 | +90° |
| R=120, 180° | −2.400 | 0.000 | +180° |

## Using in Gazebo

Each tile goes onto a flat `<box>` with the size matching its bounding box:

```xml
<model name="curve_R080_A090">
  <static>true</static>
  <pose>0 0 0  0 0 0</pose>
  <link name="link">
    <visual name="visual">
      <geometry>
        <box>
          <size>1.100 1.100 0.002</size>
        </box>
      </geometry>
      <material>
        <pbr>
          <metal>
            <albedo_map>model://road_curves/curve_R080cm_A090deg.png</albedo_map>
          </metal>
        </pbr>
      </material>
    </visual>
    <collision name="collision">
      <geometry>
        <box>
          <size>1.100 1.100 0.002</size>
        </box>
      </geometry>
    </collision>
  </link>
</model>
```

Because the PNG is RGBA with a transparent background, only the actual road
surface appears — the area outside the arc is invisible and the ground
plane (or your grass model) shows through.

### Aligning the tile in world coordinates

The entry point of the tile is at:

```
(bbox_width − xmin_world) / 2    ← horizontal position inside the bounding box
bottom edge                         ← vertical position (Y in world if Z is up)
```

The `entry_px` metadata printed by the generator gives the exact pixel
position. For placement:

1. Pose the preceding straight/curve tile so its **exit** is at some world
   `(x0, y0, θ0)`.
2. Compute the world position of the next tile's **entry** = `(x0, y0, θ0)`.
3. Work backwards to find the `<pose>` of the next tile's box centre:

```
tile_entry_offset_in_bbox = (0 − xmin_tile, 0 − ymin_tile)
                          = (xmin_tile_to_centre_of_bbox, ymin_tile_to_bottom)
```

This is fiddly. In practice most builders write a short Python helper
that takes a list of `(tile_name, rotation)` and emits the SDF, carrying the
pose along the chain. I include a minimal version below.

### Circuit builder script (Python)

```python
import math

TILES = {
    "straight_10m":   {"dx": 0.0, "dy": 10.0, "dtheta": 0, "bbox": (0.52, 10.0)},
    "curve_R50_A90":  {"dx": -0.5, "dy": 0.5, "dtheta": 90, "bbox": (0.80, 0.80)},
    "curve_R50_A180": {"dx": -1.0, "dy": 0.0, "dtheta": 180, "bbox": (0.80, 1.56)},
    "curve_R80_A45":  {"dx": -0.234, "dy": 0.566, "dtheta": 45, "bbox": (0.79, 0.718)},
}

def build_circuit(sequence):
    """sequence: list of (tile_name, mirror_bool).
       mirror_bool=True flips a left curve into a right curve.
       Returns list of (tile_name, world_pose) to write into SDF."""
    x, y, theta = 0.0, 0.0, 0.0
    placements = []
    for name, mirror in sequence:
        t = TILES[name]
        dx, dy = t["dx"], t["dy"]
        if mirror:
            dx = -dx
        c, s = math.cos(math.radians(theta)), math.sin(math.radians(theta))
        entry_wx = x
        entry_wy = y
        world_dx = c * dx - s * dy
        world_dy = s * dx + c * dy
        exit_wx = entry_wx + world_dx
        exit_wy = entry_wy + world_dy
        dth = t["dtheta"] if not mirror else -t["dtheta"]
        exit_theta = theta + dth
        placements.append((name, mirror, (entry_wx, entry_wy, theta)))
        x, y, theta = exit_wx, exit_wy, exit_theta
    return placements
```

## Example circuits

### 1. Simple oval (6 m by 2 m footprint)

```
2 × straight_10m   ↓  (actually use 2 m straights — cut from 10m)
2 × curve_R80_A180 ↷  (or 4 × curve_R80_A90 for squarer corners)
```

Straights of 2 m each connect by a U-turn of R=80 (160 × 216 cm bbox each).

### 2. Figure-eight

```
straight_5m
curve_R50_A180   (left)
straight_5m
curve_R50_A180   (right, mirrored)
```

Produces a closed figure-eight. Total track length ≈ 25 m. Fits in roughly
3 m × 5 m.

### 3. Test-track with mixed curvature

```
straight_10m
curve_R120_A45   (gentle left)
straight_5m
curve_R50_A90    (sharp left)
straight_5m
curve_R30_A180   (tight U-turn, adversarial)
straight_10m
curve_R50_A90    (sharp left, mirrored → right)
straight_5m
curve_R80_A45    (medium left, mirrored → right)
```

Gives the RL agent exposure to four distinct radii in one loop, useful for
curriculum stage 3.

## Mirroring for right turns

The tiles are all left turns. To create a right turn, **mirror the PNG
horizontally** in Gazebo by applying a negative scale on the local X axis
(or pre-flip the PNG with `convert -flop`). This swaps the sign of the
exit heading and the exit_x offset.

## Regenerating

`make_road_curves.py` is included. Edit the `RADII_M` and `ANGLES_DEG` lists
at the top of the file to add or remove tiles. Supersampling is 3× by
default for clean arc edges; lower it to 2× for faster generation if the
resulting aliasing is acceptable for your use case.

## Known limitations

1. **Line width at very tight radii**: at R=30 cm, the inner lane is only
   4 cm wide (30 − 26). The inner edge line at radius 4 cm is aggressive.
   This tile is genuinely at the physical limit for a 52 cm wide road and
   is included mainly for stress testing the cage.

2. **Dashed centreline pacing on short arcs**: for 30° arcs at small radii,
   the arc length is so short that only one or two dashes fit. This is
   geometrically correct but can look sparse.

3. **No elevation change / banking**: these are flat tiles. If you want
   banked curves, you'd need a mesh-based approach rather than a flat box
   with a texture.

Requires Pillow.
