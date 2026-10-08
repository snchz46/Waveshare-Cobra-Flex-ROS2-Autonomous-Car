# Road texture variants, lot 2 (modular, 3–5 m)

Eight short adverse road segments for combination with the nine textures of
lot 1 (`road_variants/`) in domain-randomised training. Each variant targets a
specific capability of the system.

All textures follow the specification of lot 1: pure black asphalt, 1 cm white
lines, 24.5 cm useful lane, 52 cm total two-lane width, 500 px/m.

## Catalogue

### Group 1: vision

Targets the lane estimator with white content that a simple edge detector can
mistake for lane lines.

| File | Length | Content |
|---|---|---|
| `g1_01_zebra_crossing.png` | 4 m | Transverse white stripes and stop line; requires orientation-aware line detection |
| `g1_02_lane_arrows.png` | 5 m | Painted direction arrows (straight, right) in the right lane; large white distractors within the lane |

### Group 2: policy

Targets motion planning with geometry that requires a physical response, not
only visual interpretation.

| File | Length | Content |
|---|---|---|
| `g2_01_sharp_curve.png` | 5 m | Progressive 15 cm lateral shift over 4 m; tests steering rate and anticipation |
| `g2_02_lane_narrowing.png` | 5 m | Lane narrowing from 52 cm to 36 cm and back, with cones on the left |

### Group 3: safety cage

Targets cases in which the lane lines are absent or ambiguous.

| File | Length | Content |
|---|---|---|
| `g3_01_edge_line_gap.png` | 4 m | 1.2 m gap in the right edge line (worn pavement); requires estimator feed-forward or cage fallback |
| `g3_02_double_solid_centre.png` | 5 m | Double solid white centreline (no-passing zone); both lines form one boundary |

### Group 4: domain shift

Targets colour and texture priors with elements absent from clean-asphalt
training.

| File | Length | Content |
|---|---|---|
| `g4_01_fallen_leaves.png` | 4 m | Scattered brown leaves on and near the road |
| `g4_02_drain_cover.png` | 3 m | Metal drain grate with dark slots in the right lane |

## Use in training

### Modular composition

The tiles are inserted between lot 1 tiles in a randomised sequence, for
example:

```
[two_same_01_clean, 10 m]  →
  [g1_01_zebra_crossing, 4 m]  →
  [two_same_01_clean, 10 m]  →
  [g2_01_sharp_curve, 5 m]  →
  [two_same_02_patched, 10 m]
```

Total length 39 m, with two adverse events within otherwise nominal driving,
consistent with real adverse events: short and surrounded by nominal
conditions.

### Sampling

Per episode, one or two lot 2 tiles are inserted at random positions in a lot 1
sequence. Sampling weights per curriculum stage:

| Stage | Lot 1 only | +1 lot 2 tile | +2 lot 2 tiles |
|---|---|---|---|
| 1 (ep 0–3000) | 1.0 | 0.0 | 0.0 |
| 2 (ep 3000–6000) | 0.5 | 0.5 | 0.0 |
| 3 (ep 6000–10000) | 0.2 | 0.6 | 0.2 |
| 4 (ep 10000+) | 0.1 | 0.4 | 0.5 |

Within the lot 2 branch, groups are sampled uniformly. To focus on a specific
weakness (for example, excessive cage interventions in curves), the sampling
is biased towards the corresponding group.

### Expected effects per group

**Group 1 (vision):** more frequent false line detections. The transverse
patterns verify that the line detector rejects them.

**Group 2 (policy):** a transient increase of the lateral offset. The sharp
curve is designed so that a policy trained only on gentle curves (lot 1,
stage 2) fails; this marks the point for promotion to the next curriculum
stage.

**Group 3 (cage):** an increase of the cage intervention rate around the
edge-line gap. The cage behaviour during the gap is logged; the correct
reaction is to hold the last valid lateral estimate, not to declare a lane
departure. The double solid centreline is a benign variant that produces
**no** cage interventions with a well-trained policy, as the road is physically
unchanged; it serves as a regression test.

**Group 4 (domain shift):** a measurable performance drop at the first
encounter. The recovery time (number of episodes until the mean offset returns
to baseline after the introduction of leaves) is a proxy for visual
robustness.

## Gazebo integration

Same SDF pattern as lot 1. Sizes per tile:

```xml
<box><size>0.52 4.0 0.002</size></box>  <!-- g1_01_zebra_crossing -->
<box><size>0.52 5.0 0.002</size></box>  <!-- g1_02_lane_arrows -->
<box><size>0.57 5.0 0.002</size></box>  <!-- g2_01_sharp_curve (wider box) -->
<box><size>0.52 5.0 0.002</size></box>  <!-- g2_02_lane_narrowing -->
<box><size>0.52 4.0 0.002</size></box>  <!-- g3_01_edge_line_gap -->
<box><size>0.52 5.0 0.002</size></box>  <!-- g3_02_double_solid_centre -->
<box><size>0.52 4.0 0.002</size></box>  <!-- g4_01_fallen_leaves -->
<box><size>0.52 3.0 0.002</size></box>  <!-- g4_02_drain_cover -->
```

`g2_01_sharp_curve` is wider (57 cm) because the road shifts laterally within
the tile. The centreline of the following tile is aligned with the lateral
position of the curve exit, not with the tile centre, and its pose is rotated
by the heading change (about 17° at the end of the curve).

## Regeneration

Script: `make_road_variants_lot2.py`. Each variant is self-contained and can be
adjusted independently. The drawing primitives (`draw_arrow`, `draw_leaf` and
others) can be reused for further variants.

Requires Pillow.
