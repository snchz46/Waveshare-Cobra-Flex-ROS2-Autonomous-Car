# Road texture variants for domain randomisation

Nine PNG textures for training a lane-keeping RL agent under adverse
conditions. Each of the three base road types has a clean baseline and two
adverse variants.

Common specifications:

- Asphalt base colour: pure black RGB(0, 0, 0)
- Lines: pure white RGB(255, 255, 255), 1 cm wide
- Useful lane width: 24.5 cm
- Dashed centreline: 10 cm dash × 1 cm, 10 cm gap (1:1 duty cycle)
- Resolution: 500 px/m (1 px = 2 mm)

## Catalogue

### Single lane (26.5 cm total width)

| File | Length | Description |
|---|---|---|
| `single_01_clean.png` | 10 m | Clean baseline, asphalt with fine grain |
| `single_02_worn.png` | 10 m | Worn paint, light dirt, occasional potholes and oil stains |
| `single_03_dirt_road.png` | 10 m | Brown gravel/dirt surface, faint intermittent edge markers (blurred) |

### Two lanes, same direction (52 cm total width)

| File | Length | Description |
|---|---|---|
| `two_same_01_clean.png` | 10 m | Clean baseline |
| `two_same_02_patched.png` | 10 m | Repair patches, faded paint, potholes, oil stains |
| `two_same_03_tee_junction.png` | 5 m | T-junction with side-road entrance on the right |

### Two lanes, opposite directions (52 cm total width)

| File | Length | Description |
|---|---|---|
| `two_opp_01_clean.png` | 10 m | Clean baseline |
| `two_opp_02_wet.png` | 10 m | Wet road with bluish sheen bands, blurred paint, oil stains |
| `two_opp_03_side_entrance.png` | 4 m | Side-road entrance on the left |

## Design rationale

### Perturbations

Domain randomisation trains the policy on a distribution of conditions wide
enough to include the real environment. For a lane-keeping agent with vision
as its primary sensor, the most relevant perturbations are those that degrade
the **edge and centre-line signal**, on which the lane estimator relies. The
variants degrade this signal progressively:

- **Worn paint** reduces line contrast and produces irregular edges.
- **Repair patches** add high-contrast distractors that simple classifiers
  can mistake for lines.
- **Potholes and stains** add dark regions that lane detectors can confuse
  with a line edge.
- **Wet road** softens line boundaries (Gaussian blur) and adds specular
  sheen bands that change the local brightness outside the dry-road training
  distribution.
- **Dirt road** replaces the colour basis of the scene (no black asphalt, no
  sharp lines) and requires road-edge detection instead of paint detection.
- **T-junction and side entrance** interrupt the edge line. The policy must
  remain in its lane while a line is temporarily absent.

### Use in training

A randomisation loop selects one texture per episode:

```python
TEXTURES = {
    'single_clean':     'single_01_clean.png',
    'single_worn':      'single_02_worn.png',
    'single_dirt':      'single_03_dirt_road.png',
    'two_same_clean':   'two_same_01_clean.png',
    'two_same_patched': 'two_same_02_patched.png',
    'two_same_tee':     'two_same_03_tee_junction.png',
    'two_opp_clean':    'two_opp_01_clean.png',
    'two_opp_wet':      'two_opp_02_wet.png',
    'two_opp_side':     'two_opp_03_side_entrance.png',
}

def reset_episode(rng):
    texture_name = rng.choice(list(TEXTURES.keys()))
    spawn_road_segment(TEXTURES[texture_name])
```

For a curriculum, the sampling favours clean textures at the beginning and
shifts towards adverse variants as training progresses. Example schedule:

| Stage | Clean | Worn / wet | Dirt / junction |
|---|---|---|---|
| 1 (ep 0–2000) | 1.0 | 0.0 | 0.0 |
| 2 (ep 2000–5000) | 0.6 | 0.4 | 0.0 |
| 3 (ep 5000–8000) | 0.3 | 0.5 | 0.2 |
| 4 (ep 8000+) | 0.2 | 0.4 | 0.4 |

### Junction variants and the safety cage

The T-junction and side-entrance textures test the fallback behaviour of the
safety cage. When the lane estimator loses the right edge over 20 cm, the
lateral-offset estimate of the cage becomes unreliable. Two behaviours are
applicable:

1. **Hold the lane estimate** during the gap, using the last valid offset as
   feed-forward. The cage then operates on a predicted state instead of a
   measured one.
2. **Controlled stop** or reduced speed until the edge reappears. This is the
   more conservative option and corresponds to the F3 lane-estimator-loss
   mitigation of the residual risk table.

In both cases, these variants verify the fallback logic of the cage during
training and not only in unit tests.

## Gazebo integration

Each texture is used as the `albedo_map` of a flat box with the physical
dimensions of the road:

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
        <pbr>
          <metal>
            <albedo_map>model://road_textures/two_same_02_patched.png</albedo_map>
          </metal>
        </pbr>
      </material>
    </visual>
  </link>
</model>
```

The file name is changed per episode (model reload or texture-swap plugin).
For the 5 m and 4 m junction variants, the sizes are
`<size>0.52 5.0 0.002</size>` and `<size>0.52 4.0 0.002</size>`.

### Junction tiles

The junction textures show the main road with a side-road opening. The side
road itself is a separate perpendicular road segment placed next to the
opening, so that its length and orientation are independent.

## Regeneration and extension

Script: `make_road_variants.py`. Each variant is a sequence of function calls
on a base canvas. A new variant is created by copying the most similar block
and adjusting its parameters (`wear_level`, `n_potholes`, `n_stains`, blur
radius). Seeds are fixed per variant, so the output is reproducible.

Requires Pillow.
