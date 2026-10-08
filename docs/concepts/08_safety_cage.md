# 08 · Safety cage

> **Summary:** a rule-based monitor between the controller and the wheels
> checks each command against the lane state from an independent perception
> path, corrects or stops it, and reports every intervention on `/cage_status`.

[← 07 Reinforcement learning](07_reinforcement_learning.md) · [Concepts](README.md) · Next: [09 Multi-machine networking →](09_multi_machine_networking.md)

---

## Theory

### Runtime monitoring

A learned controller is difficult to verify, since the set of images a CNN
will process cannot be enumerated. A **runtime monitor** (also called safety
envelope, or *simplex* pattern) retains the high-performance controller and
adds a simpler component, which can be reviewed and tested exhaustively. This
component overrides the controller when a command would leave the safe region.
When no safe command remains, it brings the vehicle into a **minimal-risk
condition**, here a controlled stop.

### Derivation of the rules

The rules follow the safety engineering chain:

1. **ODD**: conditions for which the function is designed. The lane-following
   ODD (`ODD-1`) assumes a 0.245 m lane and a maximum of 0.5 m/s.
2. **Hazard analysis**: potential failures (lane departure, curve entry at
   excessive speed, action on an incorrect lane estimate). STPA formulates
   them as unsafe control actions, each with a safety constraint.
3. **Safety requirements**: constraints formulated as verifiable requirements
   (`SR-xxx`), each traced to hazards (`H-xx`).
4. **Monitor rules**: one rule per requirement, with thresholds in a versioned
   configuration file.

SOTIF adds **triggering conditions**: situations without any fault in which the
performance of the function is insufficient, such as worn paint, glare or a gap
in the line. The perception supervisor described below addresses them.

### Independence

The monitor must not share the failure modes of the monitored component. The
cage reads neither the CNN of the policy nor the simulator ground truth; it
uses its own deterministic CV lane estimator
([06](06_lane_perception_and_control.md)).

---

## Implementation

### Topic interface

[`cage_ros_node.py`](../../src/safety_cage/safety_cage/cage_ros_node.py):

| Direction | Topic | Type | Content |
| --- | --- | --- | --- |
| In | `/raw_action` | `geometry_msgs/Twist` | Controller command: `angular.z` steering in $[-1, 1]$, `linear.x` throttle |
| In | `/state_obs` | `std_msgs/Float64MultiArray` | `[lateral_offset_m, heading_error_rad, speed_mps, curvature_ahead_inv_m, distance_left_m, distance_right_m, state_valid]` |
| In | `/perception_invalid` | `std_msgs/Bool` | Output of the perception supervisor |
| In | `/external_stop`, `/cage_reset` | `Bool`, `Empty` | Manual stop; reset of the emergency latch |
| Out | `/safe_action` | `geometry_msgs/Twist` | Corrected command, same convention |
| Out | `/cage_status` | `cobraflex_safety_msgs/CageStatus` | Triggered rules and their cause |
| Out | `/emergency` | `std_msgs/Bool` | Latched emergency, repeated every cycle |

Each `/raw_action` message triggers one cage cycle. If no `/state_obs` has been
received, the cage outputs the neutral safe stop; if the last `/state_obs` is
stale, C-05 is triggered.

### Rules

Evaluated in the following order every cycle:

| Order | Rule | Protection |
| --- | --- | --- |
| 1 | **C-06** rate limit | Steering and throttle jumps between cycles |
| 2 | **C-04** speed | Excessive speed: 0.5 m/s on straights (ODD limit), 0.25 m/s in curves |
| 3 | **C-02** heading | Heading error beyond its limit |
| 4 | **C-03** time to lane crossing | Lane-edge crossing too early at the current speed and heading |
| 5 | **C-01** lane offset | Lateral offset beyond its limit |
| 6 | **C-05** emergency | Controlled stop and latch on triggers such as missing or stale state, invalid perception or a high-energy situation |

The rule logic (`cage.cage_node.SafetyCageNode`) and its thresholds
(`cage/cage.yaml`) belong to the thesis repository listed under *Related work*
in the [root README](../../README.md). This repository contains the ROS
wrapper, the message definition and the perception supervisor.
`cage_ros_node` imports the `cage` package, which is installed once from the
root of the thesis repository (`pip install -e .`).

### Perception supervisor

[`cage_perception.py`](../../src/cobraflex_rl/cobraflex_rl/cage_perception.py)
combines three components, each testable without ROS, in every cycle:

| Component | Requirement | Detects |
| --- | --- | --- |
| [`CvLaneEstimator`](../../src/cobraflex_rl/cobraflex_rl/cv_lane_estimator.py) | — | Produces the lane state |
| [`PerceptionHealthMonitor`](../../src/cobraflex_rl/cobraflex_rl/perception_health.py) | SR-013 (H-11) | Perception loss: stale or dropped frame, low confidence, missing features |
| [`LanePlausibilityCheck`](../../src/cobraflex_rl/cobraflex_rl/lane_plausibility.py) | SR-014 (H-12) | Implausible estimate: geometry outside the ODD, implausible jump |

Its output is the cage state and `perception_invalid`, which feeds the
perception trigger of C-05 (inactive unless enabled in `cage.yaml`).

### `/cage_status`

[`CageStatus.msg`](../../src/cobraflex_safety_msgs/msg/CageStatus.msg)
contains per cycle: `intervention_active`, `emergency_mode`, `rules_triggered`
(for example `["C-01", "C-06"]`), the command before and after the cage, the
`cage.yaml` version, an oscillation monitor (rates per rule, persistence flag)
and `cycles_since_last_state`. `cage_logger_node` writes it to CSV.

### Actuation and operating modes

[`vehicle_control_node.py`](../../src/cobraflex_rl/cobraflex_rl/vehicle_control_node.py)
converts `/safe_action` into `/cmd_vel` and forces `linear.x` to zero while
`/emergency` is latched. During training the cage runs in-process in
**enforcement** mode (the safe action drives the vehicle) or **monitoring**
mode (the raw action drives the vehicle; the cage only records its decisions).

### Deadman timers

Independently of the cage, three timers stop the robot when their input stops:
the driver `cmd_timeout` (0.5 s), the `scan_timeout` of the LiDAR avoidance
node (0.5 s) and the `watchdog_timeout_sec` of the Gazebo lane keeper (1.5 s).
These timers must not be removed.

---

## Common errors

- **Missing `cage` package.** The import fails until the `cage` package of the
  thesis repository is installed.
- **Missing EKF.** On the robot, `/odometry/filtered` is the only speed source
  of the cage. Without it, the time to lane crossing is always infinite, C-04
  never detects excess speed and the high-energy trigger of C-05 cannot
  activate.
- **Throttle range.** The cage expects throttle in $[0, 1]$; the symmetric
  $[-1, 1]$ output of a policy must be mapped first
  ([07](07_reinforcement_learning.md)).

---

## Commands

```bash
# Gazebo demonstration: lane perception + PD baseline + cage + vehicle control
ros2 launch safety_cage lane_following.launch.py

# Cage monitoring
ros2 topic echo /cage_status --field rules_triggered
ros2 topic echo /emergency
ros2 topic pub --once /cage_reset std_msgs/msg/Empty "{}"
```

Exercise: run the worn-paint and gap worlds, and record which rules are
triggered and when `perception_invalid` becomes true.

---

## Lecture references

- **ADAS (SE4ADS) chapter 2**: operational design domain; definition of the
  ODD of the lane keeper.
- **ADAS (SE4ADS) chapter 5**: functional safety (ISO 26262), SOTIF
  (ISO 21448), FTA and STAMP/STPA; the chain from hazards to the six rules.

## Further reading

- L. Sha, "Using simplicity to control complexity", *IEEE Software*, 2001 (simplex architecture).
- N. Leveson, J. Thomas, *STPA Handbook*, 2018: <https://psas.scripts.mit.edu/home/materials/>
- ISO 21448:2022, *Road vehicles — Safety of the intended functionality*.
