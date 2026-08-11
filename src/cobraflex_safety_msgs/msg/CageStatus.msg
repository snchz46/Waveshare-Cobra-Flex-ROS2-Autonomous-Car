# Per-cycle output of the safety cage node.
#
# Hybrid topic strategy (F2 D-decision): action topics are stock Twist
# (interoperable with the existing /cmd_vel pipeline), and only the
# diagnostic /cage_status topic uses this custom message. Twist fields
# carry steering on `angular.z` and the policy/PD throttle proxy on
# `linear.x`, mirroring RosGazeboInterface.send_action() in
# src/cobraflex_rl/cobraflex_rl/ros_interface.py.

std_msgs/Header header

# True iff any rule applied a non-identity correction this cycle.
bool intervention_active

# True iff C-05 is in emergency mode this cycle.
bool emergency_mode

# Rule IDs that fired this cycle, e.g. ["C-01", "C-06"]. Empty when no
# rule intervened. Order matches the cage evaluation order
# (C-06 -> C-04 -> C-02 -> C-03 -> C-01 -> C-05).
string[] rules_triggered

# Pre-cage policy command (raw_action) and post-cage actuator command
# (safe_action), both as geometry_msgs/Twist. Fields are interpreted as
# above; brake (when used) goes on linear.x with negative sign.
geometry_msgs/Twist action_raw
geometry_msgs/Twist action_safe

# cage.yaml version string at run time. Captured per cycle to keep logs
# self-describing when the config is bumped mid-experiment.
string yaml_version

# Inter-cycle oscillation monitor (SR-010 Part 2). The arrays are
# parallel: osc_rule_ids[i] identifies which rule the rate at
# oscillation_rates_hz[i] measures (typically ["C-01","C-02","C-03"]).
string[] osc_rule_ids
float64[] oscillation_rates_hz
bool osc_persistent

# Number of step() calls without a fresh /state_obs. Zero in steady
# state; increments when state_obs goes silent. Used by the offline
# logger to correlate missing-state triggers of C-05.
uint16 cycles_since_last_state
