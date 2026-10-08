# Road textures for Gazebo

Three PNG textures of a flat road carpet for Gazebo simulations of a
scale-model lane-keeping vehicle.

## Files

| File | Total width | Useful lane(s) | Length | Pixels |
|---|---|---|---|---|
| `road_single_lane_0p265m_x_10m.png` | 26.5 cm | 24.5 cm | 10 m | 132 × 5000 |
| `road_two_lane_same_direction_0p52m_x_10m.png` | 52 cm | 2 × 24.5 cm | 10 m | 260 × 5000 |
| `road_two_lane_opposite_direction_0p52m_x_10m.png` | 52 cm | 2 × 24.5 cm | 10 m | 260 × 5000 |

## Specifications

- **Asphalt**: pure black, RGB(0, 0, 0).
- **Lines**: pure white, RGB(255, 255, 255), 1 cm wide.
- **Continuous edge lines**: both sides of every road.
- **Dashed centreline** (two-lane roads only): 10 cm dash × 1 cm, 10 cm gap,
  1:1 duty cycle.
- **Resolution**: 500 pixels per metre (1 px = 2 mm).

### Width calculation

Single lane: `24.5 (lane) + 1 + 1 (edges) = 26.5 cm`
Two lanes: `24.5 + 24.5 (lanes) + 1 + 1 + 1 (two edges + centreline) = 52 cm`

## Use in Gazebo

The road runs along the **Y axis of the image** (132 or 260 pixels wide,
5000 pixels high). For a 1:1 pixel-to-ground mapping, each texture is applied
to a box with the physical dimensions of the road:

```xml
<model name="road_segment">
  <static>true</static>
  <link name="link">
    <visual name="visual">
      <geometry>
        <box>
          <size>0.52 10.0 0.002</size>
        </box>
      </geometry>
      <material>
        <diffuse>1 1 1 1</diffuse>
        <specular>0.05 0.05 0.05 1</specular>
        <pbr>
          <metal>
            <albedo_map>model://road_textures/road_two_lane_same_direction_0p52m_x_10m.png</albedo_map>
          </metal>
        </pbr>
      </material>
    </visual>
    <collision name="collision">
      <geometry>
        <box>
          <size>0.52 10.0 0.002</size>
        </box>
      </geometry>
    </collision>
  </link>
</model>
```

Single-lane version: `<size>0.265 10.0 0.002</size>`.

### Orientation

If the vehicle heading is along +X and the texture appears rotated by 90°,
either rotate the visual with `<pose>0 0 0 0 0 1.5708</pose>` (90° about Z) or
swap the box dimensions to `<size>10.0 0.52 0.002</size>`.

### Tiling

The textures begin and end with a gap, so two 10 m segments placed end to end
produce a continuous centreline. A 20 m straight requires two segments, a 30 m
straight three.

## Regeneration

Script: `make_road_textures.py`. Constants at the top of the file:

- `LANE_USEFUL_M`: useful lane width
- `LINE_WIDTH_M`: width of all lines
- `DASH_MARK_M`, `DASH_GAP_M`: dashed centreline geometry
- `PX_PER_M`: resolution (higher values give sharper edges and larger files)

Execution: `python3 make_road_textures.py`. Requires Pillow.
