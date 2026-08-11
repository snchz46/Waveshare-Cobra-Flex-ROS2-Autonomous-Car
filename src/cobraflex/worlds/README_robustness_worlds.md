# Robustness world family (track 'E', real perception)

Nine compact ovals that share **one geometry** and differ only in road
appearance, to probe the camera policy + the cage's CV lane-estimator against
realistic road degradation. Built by `scripts/compose_lane_circuit.py --build-all`
from the length-parametric tile factory
`materials/road_assets/road_tiles/make_road_tiles.py` (no UV compression; tile
PNGs are named with their real size in metres, e.g. `tile_zebra_0p52m_x_4m.png`).

- **Geometry:** R = 0.80 m U-turns + two 4 m straights (stadium oval), footprint
  ≈ 6.0 × 2.2 m, perimeter 13.02 m. Identical across all nine → one shared
  centreline `src/cobraflex_rl/config/oval_themed_L4_centerline.yaml`.
- **Validation:** every world passes `gz sdf -k`; top-down previews in
  `experiments/sim/world_previews/preview_<name>.png`; headless load checked.
  Visual GUI pass is on the operator (`world:=<name> gui:=true`).

| World (`lane_following_…`) | Stressor | Family |
| --- | --- | --- |
| `oval_clean` | baseline / control | — |
| `oval_faded` | heavily faded, low-contrast markings | degraded paint |
| `oval_no_centreline` | dashed centreline absent | missing lines |
| `oval_no_edges` | edge lines absent (CV keys on edges) | missing lines |
| `oval_edge_gap` | 1.2 m gap in one edge line | missing lines |
| `oval_no_lines` | no paint at all (total dropout) | missing lines |
| `oval_zebra` | zebra crossing + stop line | painted distractor |
| `oval_double_solid` | double solid centre (benign regression) | painted distractor |
| `oval_arrows` | painted lane arrows | painted distractor |

`wet`/`worn` are **deliberately not** in this family: they already exist as the
frozen 1.5 m scenario worlds `lane_following_oval_wet/worn.world` (wired into
SC-PERT-09/10, `adverse_profiles.yaml`, and closed-campaign evidence incl.
`world_variant_mask_check.json`). Degraded paint here is covered by `faded`.

## Loading a map by name (no file edits)

`gazebo_mesh.launch.py` now resolves a short token via `world:=`:

```bash
ros2 launch cobraflex gazebo_mesh.launch.py world:=oval_zebra gui:=true
# resolves to worlds/lane_following_oval_zebra.world
ros2 launch cobraflex_rl eval_scenario_batch.launch.py world:=oval_no_lines ...
```

A full path or a `.world`/`.sdf` token is passed through unchanged, so existing
callers and the campaign orchestrator are unaffected.

## Regenerating

```bash
python3 scripts/compose_lane_circuit.py --build-all          # all 9 + centreline + previews
python3 scripts/compose_lane_circuit.py --circuit oval_zebra # just one
python3 src/cobraflex/materials/road_assets/road_tiles/make_road_tiles.py --length 4.0  # tiles only
```

## Proposed SR / scenario mapping (next step — not yet wired)

Each world is meant to back a scenario (`scenarios/…`) that verifies an SR. Most
map onto existing SRs; the **missing-lines** family exposes a gap.

| World | Verifies (proposed) | Notes |
| --- | --- | --- |
| `oval_clean` | (control) | parity check vs SC-NOM-01 under camera |
| `oval_faded` | SR-012 (lane-keep, degraded vision), SR-014 (estimator plausibility) | low-contrast appearance shift |
| `oval_zebra` | SR-014 (reject transverse white as a lane line) | distractor rejection |
| `oval_arrows` | SR-014 | in-lane white distractors |
| `oval_double_solid` | SR-012 (benign regression: expect ~0 interventions) | geometry unchanged |
| `oval_edge_gap` | SR-013 (degrade safely on perception loss) | partial edge dropout |
| `oval_no_edges` / `oval_no_centreline` / `oval_no_lines` | **no clean SR fit** → see candidate below | total/partial line dropout |

**Candidate new SR (missing-line robustness).** No current SR covers a road
*without usable lane markings* — a real condition (unpainted/erased roads).
Proposed: *"On loss of lane markings (no detectable centre/edge lines), the
system shall not leave the drivable surface; it shall hold the last valid lane
estimate or execute a controlled stop (SR-013 behaviour) rather than steer on
spurious features."* This links to H-? (perception-availability hazard family)
and is verified by `oval_no_lines` / `oval_no_edges` / `oval_no_centreline`.
Creating it touches the hazard register, `docs/03`, the SR CSV and traceability,
so it is left as a proposal pending your go-ahead.
