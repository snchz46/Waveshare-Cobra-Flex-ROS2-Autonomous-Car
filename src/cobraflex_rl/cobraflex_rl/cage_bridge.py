"""
cage_bridge — pure (ROS-free) glue between GazeboLaneEnv and the safety cage.

This module lets `GazeboLaneEnv` route raw policy actions through the *same*
`cage.cage_node.SafetyCageNode` that `cage_ros_node` wraps in deployment, but
**in-process** instead of over ROS2 topics (decision D-34, F3 task TS-01). The
cage logic and `cage.yaml` are identical to deployment; only the invocation
mechanism differs (synchronous call inside the gym loop, for determinism under
a fixed seed and for speed).

Everything here is deliberately ROS-free (no rclpy import) so it is unit-testable
on any host. The two mapping helpers are exact mirrors of the F2 nodes:

  * `build_cage_state`           ↔ lane_perception_node._tick (state assembly)
  * `target_speed_from_throttle` ↔ vehicle_control_node._target_speed_from_throttle
  * `safe_action_to_cmd`         ↔ vehicle_control_node._on_safe

Keep the actuation constants (throttle_nominal, yaw_gain, min_speed_scale,
fixed_speed) in sync with `src/cobraflex/.../vehicle_control_node.py` defaults.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Tuple


def _bootstrap_cage_import() -> None:
    """Ensure `from cage.cage_node import SafetyCageNode` works.

    Mirrors cage_ros_node._bootstrap_cage_import: try the import (works when the
    repo was `pip install -e .`'d or is already on PYTHONPATH); otherwise walk up
    from this file to find the repo-root `cage/` package and prepend its parent
    to sys.path.
    """
    try:
        import cage.cage_node  # noqa: F401

        return
    except ImportError:
        pass

    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "cage" / "cage_node.py"
        if candidate.is_file():
            if str(parent) not in sys.path:
                sys.path.insert(0, str(parent))
            return


_bootstrap_cage_import()

from cage.cage_node import SafetyCageNode  # noqa: E402
from cage.rules import State  # noqa: E402


def resolve_cage_yaml(explicit_path: str = "") -> str:
    """Return a path to cage.yaml.

    If `explicit_path` is given, use it verbatim; otherwise walk up from this
    file to the repo-root `cage/cage.yaml` (mirror of
    cage_ros_node._resolve_cage_yaml so training and deployment load the same
    single-source-of-truth config).
    """
    if explicit_path:
        return str(explicit_path)

    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "cage" / "cage.yaml"
        if candidate.is_file():
            return str(candidate)
    raise RuntimeError(
        "cage yaml_path not provided and cage/cage.yaml not found by walking up "
        "from cage_bridge.py. Set cfg['cage']['yaml_path'] to an explicit path."
    )


def build_cage_state(
    lateral_offset: float,
    heading_error: float,
    speed: float,
    road_width: float,
    curvature_ahead: float,
    timestamp: float,
) -> State:
    """Assemble a cage `State` from the tracker output.

    Mirrors lane_perception_node._tick: distances to the boundaries are measured
    against the *road* half-width (the env terminates at the road edge; the cage
    handles lane-keeping within it), with `lateral_offset` positive to the left.
    The env feeds ground-truth pose (/odom_truth), so state_valid is always True
    and no EMA smoothing is applied (deployment perception smooths noisy encoder
    odom; here the source is already clean).
    """
    half = float(road_width) / 2.0
    ey = float(lateral_offset)
    return State(
        lateral_offset=ey,
        heading_error=float(heading_error),
        speed=float(speed),
        curvature_ahead=float(curvature_ahead),
        distance_left=max(0.0, half - ey),
        distance_right=max(0.0, half + ey),
        state_valid=True,
        timestamp=float(timestamp),
    )


def target_speed_from_throttle(
    throttle: float,
    fixed_speed: float,
    throttle_nominal: float,
    min_speed_scale: float,
) -> float:
    """Map a normalised throttle in [0, throttle_nominal] to a speed in m/s.

    Exact mirror of vehicle_control_node._target_speed_from_throttle (with
    use_safe_throttle implicitly True): the cruise speed `fixed_speed` is scaled
    by throttle/throttle_nominal, clamped to [min_speed_scale, 1.0]; a throttle
    of ~0 yields a full stop.
    """
    throttle = max(0.0, min(float(throttle_nominal), float(throttle)))
    if throttle <= 1e-9:
        return 0.0
    scale = throttle / float(throttle_nominal)
    scale = max(float(min_speed_scale), min(1.0, scale))
    return float(fixed_speed) * scale


def safe_action_to_cmd(
    safe_steering: float,
    safe_throttle: float,
    emergency: bool,
    *,
    fixed_speed: float,
    throttle_nominal: float,
    min_speed_scale: float,
    yaw_gain: float,
) -> Tuple[float, float]:
    """Convert the cage's safe (steering, throttle) into a (linear_x, angular_z)
    /cmd_vel command, replicating vehicle_control_node._on_safe.

    Under emergency the controlled stop zeroes both axes (so the robot does not
    pivot in place); otherwise angular.z = steering·yaw_gain and linear.x is the
    throttle-scaled cruise speed.
    """
    if emergency:
        return 0.0, 0.0
    angular_z = float(safe_steering) * float(yaw_gain)
    linear_x = target_speed_from_throttle(
        safe_throttle, fixed_speed, throttle_nominal, min_speed_scale
    )
    return linear_x, angular_z


# --- 2-D action (steering + throttle) mapping — D-50, Isaac posterior track ----
#
# The three helpers below carry the throttle-as-action expansion (D-49 future
# work, taken up for the Isaac sim-to-real track). Design constraints:
#
#   * The cage rules already operate on a (steering, throttle) tuple with
#     throttle on a [0, 1]-like scale — C-04 attenuates it by
#     k_throttle_per_mps=5.0 per m/s of speed excess and C-06 rate-limits it at
#     delta_max_throttle_per_cycle=0.10 — so the policy's throttle is mapped to
#     u ∈ [0, 1] and fed to the cage UNCHANGED in meaning. No cage code or
#     cage.yaml change is needed for the rules to arbitrate.
#   * `max_speed_mps` defaults to 0.5 = C-04's v_max_straight = ODD-1.V_MAX, so
#     the policy has genuine speed authority ABOVE the curve ceiling
#     (v_max_curve 0.25) and the warning band (C-05 v_warning 0.4): the speed
#     rules become genuinely exercisable instead of structurally latent (the
#     1-D actuation capped speed at fixed_speed=0.20 < every ceiling).
#   * The map is linear with a full-stop deadband: the policy can command a
#     true stop, which is what makes SR-009's stall/liveness sub-mode (M-P6)
#     and the SC-PERT-03 negative test well-posed (D-49).
#   * C-06's throttle rate limit then bounds commanded acceleration to
#     max_speed · 0.10 / control_dt = 0.5 m/s² at 10 Hz — matching the
#     platform's measured max linear accel (0.53 m/s², docs/14 §2.3).


def policy_throttle_to_cage(action_throttle: float) -> float:
    """Map the policy's symmetric throttle command a ∈ [-1, 1] (SB3-friendly
    Box) to the cage's normalised throttle u ∈ [0, 1]: u = (a + 1) / 2."""
    a = max(-1.0, min(1.0, float(action_throttle)))
    return 0.5 * (a + 1.0)


def target_speed_from_throttle_2d(
    throttle: float,
    max_speed: float,
    throttle_deadband: float = 0.05,
) -> float:
    """2-D-action speed map: linear u ∈ [0, 1] → [0, max_speed] m/s, with a
    full stop below ``throttle_deadband`` (a true stall must be commandable,
    SR-009). Unlike `target_speed_from_throttle` (the 1-D cruise-scaling
    deployment map, floor 0.35·cruise), this map has no lower speed clamp —
    the cage's C-04 attenuation therefore has authority all the way to zero."""
    u = max(0.0, min(1.0, float(throttle)))
    if u < float(throttle_deadband):
        return 0.0
    return float(max_speed) * u


def safe_action_to_cmd_2d(
    safe_steering: float,
    safe_throttle: float,
    emergency: bool,
    *,
    max_speed: float,
    throttle_deadband: float,
    yaw_gain: float,
) -> Tuple[float, float]:
    """2-D-action counterpart of `safe_action_to_cmd`: the cage's safe
    (steering, throttle) → (linear_x, angular_z), with the linear axis on the
    `target_speed_from_throttle_2d` map. Emergency still zeroes both axes
    (C-05 controlled stop)."""
    if emergency:
        return 0.0, 0.0
    angular_z = float(safe_steering) * float(yaw_gain)
    linear_x = target_speed_from_throttle_2d(safe_throttle, max_speed, throttle_deadband)
    return linear_x, angular_z
