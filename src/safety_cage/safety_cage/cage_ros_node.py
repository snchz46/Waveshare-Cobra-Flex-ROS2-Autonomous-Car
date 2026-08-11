"""
cage_ros_node — ROS2 wrapper around cage.cage_node.SafetyCageNode.

Topic contract (hybrid F2 strategy):
    Subscriptions
        /raw_action      geometry_msgs/Twist
                           angular.z  -> steering in [-1, 1]
                           linear.x   -> throttle in [-1, 1]
        /state_obs       std_msgs/Float64MultiArray, fixed ordering:
                           [lateral_offset_m,
                            heading_error_rad,
                            speed_mps,
                            curvature_ahead_inv_m,
                            distance_left_m,
                            distance_right_m,
                            state_valid (1.0/0.0)]
        /external_stop   std_msgs/Bool        latched into ctx for next cycle
        /perception_invalid  std_msgs/Bool    latched into ctx (C-05 Trigger 8,
                           track 'E': published by cv_lane_estimator_node; inert
                           unless cage.yaml enables perception_trigger_enabled)
        /cage_reset      std_msgs/Empty       calls cage.reset_emergency()

    Publications
        /safe_action     geometry_msgs/Twist  (same convention as /raw_action)
        /cage_status     cobraflex_safety_msgs/CageStatus
        /emergency       std_msgs/Bool        latched, repeated each cycle

Cycle trigger: the /raw_action callback drives the cage. /state_obs is
buffered; if /raw_action arrives before any /state_obs ever did, the
cage emits the neutral safe-stop and flags missing-state. If /state_obs
has been received before but staleness exceeds the cage's threshold,
C-05 fires via its 'stale' trigger inside SafetyCageNode.step().

Importing cage.cage_node:
    The pure-Python cage package lives at the repo root (not under
    src/). For the import to succeed under colcon-installed layouts,
    install the repo as an editable Python package once:
        cd <repo_root> && pip install -e .
    A fallback path-walk handles the case where the node is launched
    directly from the source tree without pip install.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional


def _bootstrap_cage_import() -> None:
    """Ensure `from cage.cage_node import SafetyCageNode` works.

    Strategy:
        1. Try the import. If it works (because the repo was pip-installed
           or because PYTHONPATH already contains the repo root), we are
           done.
        2. Otherwise, walk up from this file looking for a sibling
           directory named `cage/` with `cage_node.py` in it. Prepend
           its parent to sys.path.
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

import rclpy  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSPresetProfiles  # noqa: E402
from std_msgs.msg import Bool, Empty, Float64MultiArray  # noqa: E402

from cage.cage_node import SafetyCageNode  # noqa: E402
from cage.rules import State  # noqa: E402
from cobraflex_safety_msgs.msg import CageStatus  # noqa: E402


_STATE_OBS_LAYOUT = (
    "lateral_offset",
    "heading_error",
    "speed",
    "curvature_ahead",
    "distance_left",
    "distance_right",
    "state_valid",
)


class CageRosNode(Node):
    """ROS2 transport for SafetyCageNode (topic contract in the module docstring)."""

    def __init__(self) -> None:
        super().__init__("safety_cage")

        self.declare_parameter("cage_yaml", "")
        self.declare_parameter("mode", "enforcement")
        self.declare_parameter("state_obs_topic", "/state_obs")
        self.declare_parameter("raw_action_topic", "/raw_action")
        self.declare_parameter("safe_action_topic", "/safe_action")
        self.declare_parameter("cage_status_topic", "/cage_status")
        self.declare_parameter("emergency_topic", "/emergency")
        self.declare_parameter("external_stop_topic", "/external_stop")
        self.declare_parameter("perception_invalid_topic", "/perception_invalid")
        self.declare_parameter("reset_topic", "/cage_reset")

        cage_yaml = self._resolve_cage_yaml()
        mode = self.get_parameter("mode").get_parameter_value().string_value or "enforcement"
        self.cage = SafetyCageNode(cage_yaml, mode=mode)
        self.get_logger().info(
            f"Loaded cage.yaml v{self.cage.version} from {cage_yaml} in '{mode}' mode."
        )

        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value
        reliable_qos = QoSPresetProfiles.SYSTEM_DEFAULT.value

        self._latest_state: Optional[State] = None
        self._external_stop = False
        self._perception_invalid = False

        self.create_subscription(
            Float64MultiArray,
            self.get_parameter("state_obs_topic").value,
            self._on_state_obs,
            sensor_qos,
        )
        self.create_subscription(
            Twist,
            self.get_parameter("raw_action_topic").value,
            self._on_raw_action,
            sensor_qos,
        )
        self.create_subscription(
            Bool,
            self.get_parameter("external_stop_topic").value,
            self._on_external_stop,
            reliable_qos,
        )
        self.create_subscription(
            Bool,
            self.get_parameter("perception_invalid_topic").value,
            self._on_perception_invalid,
            reliable_qos,
        )
        self.create_subscription(
            Empty,
            self.get_parameter("reset_topic").value,
            self._on_reset,
            reliable_qos,
        )

        self._safe_pub = self.create_publisher(
            Twist, self.get_parameter("safe_action_topic").value, sensor_qos
        )
        self._status_pub = self.create_publisher(
            CageStatus, self.get_parameter("cage_status_topic").value, reliable_qos
        )
        self._emergency_pub = self.create_publisher(
            Bool, self.get_parameter("emergency_topic").value, reliable_qos
        )

        self.get_logger().info(
            "Subscriptions: %s, %s, %s, %s"
            % (
                self.get_parameter("state_obs_topic").value,
                self.get_parameter("raw_action_topic").value,
                self.get_parameter("external_stop_topic").value,
                self.get_parameter("reset_topic").value,
            )
        )
        self.get_logger().info(
            "Publications: %s, %s, %s"
            % (
                self.get_parameter("safe_action_topic").value,
                self.get_parameter("cage_status_topic").value,
                self.get_parameter("emergency_topic").value,
            )
        )

    def _resolve_cage_yaml(self) -> str:
        """Cage config path: the ``cage_yaml`` param or the repo's cage/cage.yaml."""
        param_value = self.get_parameter("cage_yaml").get_parameter_value().string_value
        if param_value:
            return param_value

        here = Path(__file__).resolve()
        for parent in here.parents:
            candidate = parent / "cage" / "cage.yaml"
            if candidate.is_file():
                return str(candidate)
        raise RuntimeError(
            "cage_yaml parameter empty and cage/cage.yaml not found by walking "
            "up from the node source file. Pass --ros-args -p cage_yaml:=<path>."
        )

    def _on_state_obs(self, msg: Float64MultiArray) -> None:
        """Buffer the latest state sample, stamped with the node clock."""
        if len(msg.data) < len(_STATE_OBS_LAYOUT):
            self.get_logger().warning(
                f"/state_obs has {len(msg.data)} fields, expected "
                f"{len(_STATE_OBS_LAYOUT)} ({_STATE_OBS_LAYOUT}); ignoring."
            )
            return

        sim_time_s = float(self.get_clock().now().nanoseconds) * 1e-9
        self._latest_state = State(
            lateral_offset=float(msg.data[0]),
            heading_error=float(msg.data[1]),
            speed=float(msg.data[2]),
            curvature_ahead=float(msg.data[3]),
            distance_left=float(msg.data[4]),
            distance_right=float(msg.data[5]),
            state_valid=bool(msg.data[6] >= 0.5),
            timestamp=sim_time_s,
        )

    def _on_external_stop(self, msg: Bool) -> None:
        self._external_stop = bool(msg.data)
        if msg.data:
            self.get_logger().warning("External stop signal received.")

    def _on_perception_invalid(self, msg: Bool) -> None:
        # cv_lane_estimator_node publishes every tick, so the latest value is
        # the current supervisor verdict (no sticky latch needed here — C-05
        # itself latches the emergency once Trigger 8 fires).
        if bool(msg.data) and not self._perception_invalid:
            self.get_logger().warning("Perception invalid signal received (C-05 Trigger 8).")
        self._perception_invalid = bool(msg.data)

    def _on_reset(self, _msg: Empty) -> None:
        # Direct call already flips C-05's _reset_requested for the next
        # step(); no need to also pass ctx["reset"]=True (would just set the
        # same flag a second time inside the rule).
        self.cage.reset_emergency()
        self.get_logger().info("/cage_reset received -> cage.reset_emergency() called.")

    def _on_raw_action(self, msg: Twist) -> None:
        """Cycle trigger: run one cage step on the buffered state and publish."""
        raw_steering = float(msg.angular.z)
        raw_throttle = float(msg.linear.x)
        raw_action = (raw_steering, raw_throttle)

        sim_time_s = float(self.get_clock().now().nanoseconds) * 1e-9
        ctx = {
            "current_time": sim_time_s,
            "external_stop": self._external_stop,
            "perception_invalid": self._perception_invalid,
        }

        result = self.cage.step(self._latest_state, raw_action, ctx=ctx)
        self._publish(result, header_stamp_s=sim_time_s)

    def _publish(self, result: dict, header_stamp_s: float) -> None:
        """Fan one step() result out to /safe_action, /cage_status and /emergency."""
        safe_steering, safe_throttle = result["safe_action"]
        safe = Twist()
        safe.angular.z = float(safe_steering)
        safe.linear.x = float(safe_throttle)
        self._safe_pub.publish(safe)

        status = CageStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = "cage"

        interventions = result["interventions"]
        status.intervention_active = len(interventions) > 0
        status.emergency_mode = bool(result["emergency"])
        status.rules_triggered = [iv["rule"] for iv in interventions]

        raw_steering, raw_throttle = result["raw_action"]
        status.action_raw.angular.z = float(raw_steering)
        status.action_raw.linear.x = float(raw_throttle)
        status.action_safe.angular.z = float(safe_steering)
        status.action_safe.linear.x = float(safe_throttle)

        status.yaml_version = result.get("cage_version", "unknown")

        osc_rates = result.get("oscillation_rates_hz", {}) or {}
        status.osc_rule_ids = list(osc_rates.keys())
        status.oscillation_rates_hz = [float(v) for v in osc_rates.values()]
        status.osc_persistent = bool(result.get("oscillation_persistent", False))
        status.cycles_since_last_state = int(result.get("cycles_since_last_state", 0))

        self._status_pub.publish(status)

        emergency_msg = Bool()
        emergency_msg.data = bool(result["emergency"])
        self._emergency_pub.publish(emergency_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CageRosNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
