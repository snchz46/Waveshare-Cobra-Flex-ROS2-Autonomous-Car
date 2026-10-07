# 08 · Safety cage

> **In one sentence:** a simple, rule-based monitor sits between any controller
> and the wheels, checks each command against the lane state from an
> independent perception path, corrects or stops it, and logs every
> intervention on `/cage_status`.

[← 07 Reinforcement learning](07_reinforcement_learning.md) · [Concepts](README.md) · Next: [09 Multi-machine networking →](09_multi_machine_networking.md)

---

## Concept

### Why a cage

A learned controller is hard to verify: nobody can list every image a CNN will
see. A **runtime monitor** (also called a safety envelope, or the *simplex*
pattern) keeps the high-performance controller but lets a much simpler
component — one that *can* be reviewed and tested exhaustively — override it
whenever the command would leave a safe region. When nothing is safe any more,
it brings the vehicle to a **minimal-risk condition**: here, a controlled stop.

### Where the rules come from

The rules are not invented ad hoc; they follow the safety engineering chain:

1. **ODD** — the conditions the function is designed for. The lane-following
   ODD (`ODD-1`) assumes a 0.245 m lane and at most 0.5 m/s.
2. **Hazard analysis** — what can go wrong (leaving the lane, entering a curve
   too fast, acting on a wrong lane estimate). STPA phrases these as unsafe
   control actions; each gets a safety constraint.
3. **Safety requirements** — constraints turned into checkable requirements
   (`SR-xxx`), each traced to hazards (`H-xx`).
4. **Monitor rules** — one rule per requirement, with thresholds in a versioned
   configuration file.

SOTIF adds **triggering conditions**: situations where nothing is broken but
the function's performance is not enough — worn paint, glare, a gap in the
line. The perception supervisor below exists for them.

### Independence

The monitor must not share the failure modes of what it monitors. Here the cage
never reads the policy's CNN and never reads simulator ground truth; it reads
its own deterministic CV lane estimator ([06](06_lane_perception_and_control.md)).

---

## In this repository

### Topic contract

[`cage_ros_node.py`](../../src/safety_cage/safety_cage/cage_ros_node.py):

| Direction | Topic | Type | Content |
| --- | --- | --- | --- |
| In | `/raw_action` | `geometry_msgs/Twist` | Controller command: `angular.z` steering in $[-1, 1]$, `linear.x` throttle |
| In | `/state_obs` | `std_msgs/Float64MultiArray` | `[lateral_offset_m, heading_error_rad, speed_mps, curvature_ahead_inv_m, distance_left_m, distance_right_m, state_valid]` |
| In | `/perception_invalid` | `std_msgs/Bool` | From the perception supervisor |
| In | `/external_stop`, `/cage_reset` | `Bool`, `Empty` | Manual stop; reset of the emergency latch |
| Out | `/safe_action` | `geometry_msgs/Twist` | Corrected command, same convention |
| Out | `/cage_status` | `cobraflex_safety_msgs/CageStatus` | What fired and why |
| Out | `/emergency` | `std_msgs/Bool` | Latched emergency, repeated every cycle |

Each `/raw_action` triggers one cage cycle. If no `/state_obs` has ever
arrived, the cage outputs the neutral safe stop; if it arrived but is stale,
C-05 fires.

### The six rules

Evaluated in this order every cycle:

| Order | Rule | Guards against |
| --- | --- | --- |
| 1 | **C-06** rate limit | Steering and throttle jumps between cycles |
| 2 | **C-04** speed | Too fast for the road: 0.5 m/s on straights (the ODD limit), 0.25 m/s in curves |
| 3 | **C-02** heading | Heading error beyond its limit |
| 4 | **C-03** time to lane crossing | Reaching the lane edge too soon at the current speed and heading |
| 5 | **C-01** lane offset | Lateral offset beyond its limit |
| 6 | **C-05** emergency | Controlled stop and latch, on triggers such as missing or stale state, invalid perception or a high-energy situation |

The rule logic (`cage.cage_node.SafetyCageNode`) and its thresholds
(`cage/cage.yaml`) belong to the thesis repository listed under *Related work*
in the [root README](../../README.md); this repository carries the ROS
wrapper, the message and the perception supervisor. `cage_ros_node` imports the
`cage` package, so install it once from the thesis repository root
(`pip install -e .`) before running the cage.

### Perception supervisor

[`cage_perception.py`](../../src/cobraflex_rl/cobraflex_rl/cage_perception.py)
composes three host-testable pieces every cycle:

| Piece | Requirement | Flags |
| --- | --- | --- |
| [`CvLaneEstimator`](../../src/cobraflex_rl/cobraflex_rl/cv_lane_estimator.py) | — | Produces the lane state |
| [`PerceptionHealthMonitor`](../../src/cobraflex_rl/cobraflex_rl/perception_health.py) | SR-013 (H-11) | Lost perception: stale or dropped frame, low confidence, missing features |
| [`LanePlausibilityCheck`](../../src/cobraflex_rl/cobraflex_rl/lane_plausibility.py) | SR-014 (H-12) | Suspect estimate: geometry outside the ODD, implausible jump |

Its output is the cage state plus `perception_invalid`, which feeds C-05's
perception trigger (inert unless enabled in `cage.yaml`).

### `/cage_status`

[`CageStatus.msg`](../../src/cobraflex_safety_msgs/msg/CageStatus.msg) carries
per cycle: `intervention_active`, `emergency_mode`, `rules_triggered` (for
example `["C-01", "C-06"]`), the command before and after the cage, the
`cage.yaml` version, an oscillation monitor (rates per rule, persistence flag)
and `cycles_since_last_state`. `cage_logger_node` writes it to CSV.

### Actuation and modes

[`vehicle_control_node.py`](../../src/cobraflex_rl/cobraflex_rl/vehicle_control_node.py)
turns `/safe_action` into `/cmd_vel`; while `/emergency` is latched it forces
`linear.x` to zero. In training the cage runs in-process in either
**enforcement** (the safe action drives) or **monitoring** mode (the raw action
drives, the cage only logs what it would have done).

### Deadman timers — the layer below

Independent of the cage, three timers stop the robot when their input goes
silent: the driver's `cmd_timeout` (0.5 s), the LiDAR avoidance node's
`scan_timeout` (0.5 s) and the Gazebo lane keeper's `watchdog_timeout_sec`
(1.5 s). Never remove them.

---

## Pitfalls

- **No `cage` package, no cage.** The import fails until the thesis
  repository's `cage` package is installed.
- **No EKF, silent rules.** On the car `/odometry/filtered` is the cage's only
  speed source. Without it the time to lane crossing is always infinite, C-04
  never sees excess speed and C-05's high-energy trigger cannot arm.
- **Throttle range.** The cage expects throttle in $[0, 1]$; a policy's
  symmetric $[-1, 1]$ output must be mapped first ([07](07_reinforcement_learning.md)).

---

## Try it

```bash
# Gazebo demo: lane perception + PD baseline + cage + vehicle control
ros2 launch safety_cage lane_following.launch.py

# Watch the cage work
ros2 topic echo /cage_status --field rules_triggered
ros2 topic echo /emergency
ros2 topic pub --once /cage_reset std_msgs/msg/Empty "{}"
```

Exercise: run the worn-paint and gap worlds and record which rules fire, and
when `perception_invalid` turns true.

---

## Lecture links

- **ADAS (SE4ADS) chapter 2** — operational design domain: write the ODD of
  your lane keeper.
- **ADAS (SE4ADS) chapter 5** — functional safety (ISO 26262), SOTIF
  (ISO 21448), FTA and STAMP/STPA: the chain from hazards to the six rules.

## Further reading

- L. Sha, "Using simplicity to control complexity", *IEEE Software*, 2001 — the simplex architecture.
- N. Leveson, J. Thomas, *STPA Handbook*, 2018: <https://psas.scripts.mit.edu/home/materials/>
- ISO 21448:2022, *Road vehicles — Safety of the intended functionality*.
