# Road texture variants for domain randomization

Nine PNG textures for training a lane-keeping RL agent against realistic
adversarial conditions. Each of the three base road types comes with a clean
baseline plus two adversarial variants.

All textures share the same specifications:
- Asphalt base colour: pure black RGB(0, 0, 0)
- Lines: pure white RGB(255, 255, 255), 1 cm wide
- Useful lane width: 24.5 cm
- Dashed centreline: 10 cm dash × 1 cm, 10 cm gap (1:1 duty)
- Resolution: 500 px/m (1 px = 2 mm)

## Catalogue

### Single lane (26.5 cm total width)

| File | Length | Description |
|---|---|---|
| `single_01_clean.png` | 10 m | Clean baseline, asphalt with subtle grain |
| `single_02_worn.png` | 10 m | Worn paint, light dirt sprinkle, occasional potholes and oil stains |
| `single_03_dirt_road.png` | 10 m | Brown gravel/dirt surface, faint intermittent edge markers (blurred) |

### Two lanes same direction (52 cm total width)

| File | Length | Description |
|---|---|---|
| `two_same_01_clean.png` | 10 m | Clean baseline |
| `two_same_02_patched.png` | 10 m | Repair patches, faded paint, potholes, oil stains |
| `two_same_03_tee_junction.png` | 5 m | T-junction with side road entrance on the right |

### Two lanes opposite direction (52 cm total width)

| File | Length | Description |
|---|---|---|
| `two_opp_01_clean.png` | 10 m | Clean baseline |
| `two_opp_02_wet.png` | 10 m | Wet road with bluish sheen bands, blurred paint, oil stains |
| `two_opp_03_side_entrance.png` | 4 m | Side road entrance on the left |

## Design rationale

### Why these variants

Domain randomization trains the policy on a distribution of conditions wide
enough that the real world is inside it. For a lane-keeping agent whose
primary sensor is vision, the most damaging perturbations are the ones that
degrade the **edge and centre line signal** — because those are what the
lane estimator relies on. The variants progressively attack that signal:

- **Worn paint** reduces line contrast and introduces ragged edges.
- **Repair patches** add high-contrast distractors that can be mistaken
  for lines by naïve classifiers.
- **Potholes and stains** add dark blobs that lane detectors may confuse
  with a line edge.
- **Wet road** softens line boundaries (gaussian blur) and adds specular
  sheen bands that change local brightness in ways a dry-trained policy
  has never seen.
- **Dirt road** replaces the entire colour basis of the scene — no black
  asphalt, no crisp lines — forcing the policy to rely on road-edge
  detection rather than paint detection.
- **T-junction and side entrance** break the invariant that the edge line
  is always continuous. The policy must learn to stay in its lane even
  when a line disappears temporarily.

### How to use in training

A simple randomization loop picks one texture per episode:

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

For a curriculum, bias the sampling toward clean textures early and shift
weight toward adversarial variants as training progresses. A reasonable
schedule:

| Stage | Clean | Worn / wet | Dirt / junction |
|---|---|---|---|
| 1 (ep 0–2000) | 1.0 | 0.0 | 0.0 |
| 2 (ep 2000–5000) | 0.6 | 0.4 | 0.0 |
| 3 (ep 5000–8000) | 0.3 | 0.5 | 0.2 |
| 4 (ep 8000+) | 0.2 | 0.4 | 0.4 |

### Junction variants and the safety cage

The T-junction and side-entrance textures are where the safety cage earns
its keep. When the lane estimator loses the right edge for 20 cm, the cage's
lateral-offset prediction becomes unreliable. Two sensible behaviours:

1. **Freeze the lane estimate** during the gap, using the last valid offset
   as a feed-forward. The cage then continues to operate on a predicted
   state rather than a measured one.
2. **Fall back to a controlled stop** or reduced speed until the edge
   reappears. This is the more conservative option and is what the
   F3-lane-estimator-loss mitigation in your residual risk table describes.

Either way, these variants let you verify the cage's fallback logic actually
kicks in during training, not just in post-hoc unit tests.

## Gazebo integration

Use each texture as the `albedo_map` of a flat box sized to match the physical
dimensions:

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

Swap the filename per episode (reload the model or use a texture-swap plugin).
For the 5 m and 4 m junction variants, use `<size>0.52 5.0 0.002</size>` and
`<size>0.52 4.0 0.002</size>` respectively.

### Joining junction tiles

The junction textures represent the main road with a side-road opening.
They do not contain the side road itself — that should be a separate
perpendicular road segment placed adjacent to the opening, so the side road
can be of any length and orientation.

## Regenerating or extending

`make_road_variants.py` is included. Each variant is a sequence of function
calls over a base canvas. To create a new variant, copy the most similar
block and tweak the parameters (`wear_level`, `n_potholes`, `n_stains`, blur
radius). Seeds are fixed per variant so output is reproducible.

Requires Pillow.
