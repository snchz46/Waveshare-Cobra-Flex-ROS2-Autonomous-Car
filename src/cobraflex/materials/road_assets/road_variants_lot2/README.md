# Road texture variants — Lot 2 (modular, 3-5 m)

Eight short adversarial road segments designed to combine with the 9 textures
of Lot 1 during domain-randomized training. Each variant stresses a specific
capability of the system.

All textures share the specification of Lot 1: pure black asphalt, 1 cm
white lines, 24.5 cm useful lane, 52 cm two-lane total width, 500 px/m.

## Catalogue

### Group 1 — Vision stressors

Target the lane estimator. Introduce confusing white content that a naïve
edge detector can mistake for lane lines.

| File | Length | What it does |
|---|---|---|
| `g1_01_zebra_crossing.png` | 4 m | Transverse white stripes + stop line. Forces orientation-aware line detection |
| `g1_02_lane_arrows.png` | 5 m | Painted directional arrows (straight, right) inside the right lane. Large white distractors in free space |

### Group 2 — Policy stressors

Target the policy's ability to plan motion. Introduce geometry the vehicle
must physically respond to, not just visually interpret.

| File | Length | What it does |
|---|---|---|
| `g2_01_sharp_curve.png` | 5 m | Progressive 15 cm lateral shift over 4 m. Tests steering rate and anticipation |
| `g2_02_lane_narrowing.png` | 5 m | Lane tapers from 52 cm to 36 cm and back, with cones on the left |

### Group 3 — Safety cage stressors

Target edge cases where the cage's invariant check is stressed. Break the
assumption that lane lines are always present and unambiguous.

| File | Length | What it does |
|---|---|---|
| `g3_01_edge_line_gap.png` | 4 m | 1.2 m gap in the right edge line (worn pavement). Estimator must feed-forward or cage must fallback |
| `g3_02_double_solid_centre.png` | 5 m | Double solid white centreline (no-passing zone). Cage must treat both as one barrier |

### Group 4 — Domain shift

Target colour and texture priors. Introduce elements the policy has never
seen if trained only on clean asphalt.

| File | Length | What it does |
|---|---|---|
| `g4_01_fallen_leaves.png` | 4 m | Scattered brown autumn leaves on and near the road |
| `g4_02_drain_cover.png` | 3 m | Metal drain grate with dark slots in the right lane |

## How to use during training

### Modular composition

These tiles are designed to be placed between regular Lot 1 tiles in a
randomised sequence. A typical training episode might stitch together:

```
[two_same_01_clean, 10 m]  →
  [g1_01_zebra_crossing, 4 m]  →
  [two_same_01_clean, 10 m]  →
  [g2_01_sharp_curve, 5 m]  →
  [two_same_02_patched, 10 m]
```

Total 39 m, with two adversarial events embedded in otherwise clean driving.
This matches how real-world adversarial events appear: locally brief,
surrounded by nominal conditions.

### Sampling strategy

Per-episode, sample one or two Lot 2 tiles and insert at random positions in
a Lot 1 backbone. Weight the sampling by curriculum stage:

| Stage | Lot 1 only | +1 Lot 2 tile | +2 Lot 2 tiles |
|---|---|---|---|
| 1 (ep 0–3000) | 1.0 | 0.0 | 0.0 |
| 2 (ep 3000–6000) | 0.5 | 0.5 | 0.0 |
| 3 (ep 6000–10000) | 0.2 | 0.6 | 0.2 |
| 4 (ep 10000+) | 0.1 | 0.4 | 0.5 |

Within the "pick a Lot 2 tile" branch, sample uniformly across groups for
diversity. Alternatively, if you want to focus the curriculum on a specific
weakness (e.g. the cage is over-intervening during curves), bias toward that
group.

### What to expect per group

**Group 1** (vision): expect the cage's `g3` obstacle-clearance guard to fire
more often, because a zebra crossing registers as "surface texture" not
"obstacle" but naïve classifiers may flag it. Use this to verify the
classifier head rejects transverse patterns.

**Group 2** (policy): expect lateral offset to rise transiently. The sharp
curve is designed so that a policy trained only on gentle curves (Lot 1
stage 2) will fail — this is the point where curriculum stage promotion
should happen.

**Group 3** (cage): expect the cage intervention rate to spike around the
edge-line gap. Log the cage behaviour during the gap — the correct reaction
is to freeze the last valid lateral estimate, not to declare the vehicle
out of lane. The double solid centreline is a benign variant that should
produce **zero** cage interventions in a well-trained policy (nothing
physically different about the road), so it acts as a regression test.

**Group 4** (domain shift): expect a measurable drop in policy performance
the first time the agent sees these. Track recovery time as a metric —
"how many episodes until mean offset returns to baseline after introducing
leaves" is a good proxy for visual robustness.

## Gazebo integration

Same SDF pattern as Lot 1. Sizes per tile:

```xml
<box><size>0.52 4.0 0.002</size></box>  <!-- g1_01_zebra_crossing -->
<box><size>0.52 5.0 0.002</size></box>  <!-- g1_02_lane_arrows -->
<box><size>0.57 5.0 0.002</size></box>  <!-- g2_01_sharp_curve (note wider box) -->
<box><size>0.52 5.0 0.002</size></box>  <!-- g2_02_lane_narrowing -->
<box><size>0.52 4.0 0.002</size></box>  <!-- g3_01_edge_line_gap -->
<box><size>0.52 5.0 0.002</size></box>  <!-- g3_02_double_solid_centre -->
<box><size>0.52 4.0 0.002</size></box>  <!-- g4_01_fallen_leaves -->
<box><size>0.52 3.0 0.002</size></box>  <!-- g4_02_drain_cover -->
```

Note that `g2_01_sharp_curve` is slightly wider (57 cm) because the road
drifts laterally within the tile. When concatenating after it, the next tile's
centreline should align with the right-side position of the curve's exit,
not the tile centre. In practice you rotate the next tile's pose to match
the heading shift (≈17° at the end of the curve).

## Regenerating

`make_road_variants_lot2.py` is included. Each variant is self-contained and
can be tweaked independently. The drawing primitives (`draw_arrow`,
`draw_leaf`, etc.) are reusable for further variants.

Requires Pillow.
