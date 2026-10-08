# Curved road modules

Sixteen curved road tiles for modular circuits in Gazebo: four centreline radii
(30, 50, 80, 120 cm) combined with four arc angles (30°, 45°, 90°, 180°).

All tiles follow the specification of the straight textures: two-lane road
(52 cm total width, 24.5 cm useful per lane), 1 cm white lines, dashed
centreline with 10 cm dashes and 10 cm gaps measured along the arc, 500 px/m
resolution. **Backgrounds are transparent (RGBA PNG).**

## Tile catalogue

| Radius | 30° | 45° | 90° | 180° |
|---|---|---|---|---|
| 30 cm | `curve_R030cm_A030deg.png` | `curve_R030cm_A045deg.png` | `curve_R030cm_A090deg.png` | `curve_R030cm_A180deg.png` |
| 50 cm | `curve_R050cm_A030deg.png` | `curve_R050cm_A045deg.png` | `curve_R050cm_A090deg.png` | `curve_R050cm_A180deg.png` |
| 80 cm | `curve_R080cm_A030deg.png` | `curve_R080cm_A045deg.png` | `curve_R080cm_A090deg.png` | `curve_R080cm_A180deg.png` |
| 120 cm | `curve_R120cm_A030deg.png` | `curve_R120cm_A045deg.png` | `curve_R120cm_A090deg.png` | `curve_R120cm_A180deg.png` |

### Bounding boxes (Gazebo box sizes)

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

## Tile geometry

Each tile represents a **left turn**:

- The road **entry** is at the bottom of the image, with the road heading
  upwards (+Y on the ground).
- The **centre of curvature** lies to the left of the entry, at the local
  coordinate `(-R, 0)`, where `R` is the centreline radius.
- The arc sweeps counter-clockwise by the angle α.

### Exit point relative to the entry

With the entry at `(0, 0)` and heading `+Y`, the exit lies on the circle of
radius `R` around `(-R, 0)`, rotated by α:

```
exit_x = -R + R·cos(α) = -R · (1 − cos α)
exit_y =      R·sin(α)
exit_heading = +α
```

| Angle | exit_x | exit_y | exit_heading |
|---|---|---|---|
| 30° | `−0.134 R` | `0.500 R` | +30° |
| 45° | `−0.293 R` | `0.707 R` | +45° |
| 90° | `−R` | `R` | +90° |
| 180° | `−2R` | `0` | +180° |

For a right turn (mirrored tile), `exit_x` and `exit_heading` change sign.

### Exit positions per tile (metres, relative to the entry)

| Tile | dx | dy | Heading |
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

## Use in Gazebo

Each tile is applied to a flat `<box>` with the size of its bounding box:

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

The transparent RGBA background shows only the road surface; the ground plane
or grass model remains visible outside the arc.

### Placement in world coordinates

The entry point lies on the bottom edge of the bounding box; its exact pixel
position is printed by the generator as `entry_px`. Placement procedure:

1. Place the preceding tile so that its **exit** is at the world pose
   `(x0, y0, θ0)`.
2. Set the **entry** of the next tile to `(x0, y0, θ0)`.
3. Derive the `<pose>` of the box centre of the next tile from the entry
   offset within its bounding box.

A short script that converts a list of `(tile_name, mirror)` into SDF poses,
carrying the pose along the chain, simplifies this procedure:

```python
import math

TILES = {
    "straight_10m":   {"dx": 0.0, "dy": 10.0, "dtheta": 0, "bbox": (0.52, 10.0)},
    "curve_R50_A90":  {"dx": -0.5, "dy": 0.5, "dtheta": 90, "bbox": (0.80, 0.80)},
    "curve_R50_A180": {"dx": -1.0, "dy": 0.0, "dtheta": 180, "bbox": (0.80, 1.56)},
    "curve_R80_A45":  {"dx": -0.234, "dy": 0.566, "dtheta": 45, "bbox": (0.79, 0.718)},
}

def build_circuit(sequence):
    """Return (tile_name, mirror, entry_pose) for each tile of the sequence.

    sequence: list of (tile_name, mirror); mirror=True converts a left curve
    into a right curve.
    """
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

### 1. Oval (6 m × 2 m footprint)

```
2 × straight, 2 m each (cut from the 10 m texture)
2 × curve_R80_A180 (or 4 × curve_R80_A90 for squarer corners)
```

The 2 m straights are connected by R=80 U-turns (bounding box 1.10 × 2.16 m
each).

### 2. Figure eight

```
straight_5m
curve_R50_A180   (left)
straight_5m
curve_R50_A180   (right, mirrored)
```

Closed figure eight, total track length about 25 m, footprint about
3 m × 5 m.

### 3. Test track with mixed curvature

```
straight_10m
curve_R120_A45   (gentle left)
straight_5m
curve_R50_A90    (sharp left)
straight_5m
curve_R30_A180   (tight U-turn, adverse)
straight_10m
curve_R50_A90    (sharp left, mirrored → right)
straight_5m
curve_R80_A45    (medium left, mirrored → right)
```

Four different radii in one loop; suitable for curriculum stage 3.

## Right turns

All tiles are left turns. A right turn is obtained by **mirroring the PNG
horizontally**, either in Gazebo with a negative scale on the local X axis or
beforehand with `convert -flop`. Mirroring changes the sign of the exit
heading and of `exit_x`.

## Regeneration

Script: `make_road_curves.py`. The lists `RADII_M` and `ANGLES_DEG` at the top
of the file define the generated tiles. Supersampling is 3× by default for
smooth arc edges; 2× generates faster at the cost of visible aliasing.

## Limitations

1. **Line width at small radii**: at R=30 cm the inner lane is only 4 cm wide
   (30 − 26), and the inner edge line lies at a radius of 4 cm. This tile is at
   the geometric limit for a 52 cm road and is intended mainly for stress
   tests of the safety cage.
2. **Dash spacing on short arcs**: on 30° arcs at small radii, only one or two
   dashes fit on the arc. This is geometrically correct but appears sparse.
3. **No elevation or banking**: the tiles are flat. Banked curves require a
   mesh-based model instead of a textured box.

Requires Pillow.
