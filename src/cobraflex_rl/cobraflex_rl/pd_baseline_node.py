"""
pd_baseline_node — wraps policy.baseline_pd.BaselinePD as a ROS2 node.

Subscriptions
    /state_obs   std_msgs/Float64MultiArray with the layout declared in
                 src/safety_cage/safety_cage/cage_ros_node.py.

Publications
    /raw_action  geometry_msgs/Twist
                 angular.z = steering, linear.x = throttle (PD output).

The throttle component of /raw_action is computed by the PD per
policy/baseline_pd.yaml (curve-aware nominal). vehicle_control_node can
use the cage's safe throttle to scale the straight cruise speed, so this
field is part of both safety evaluation and physical actuation.
"""

from __future__ import annotations

import sys
from pathlib import Path


def _bootstrap_policy_import() -> None:
    """Make ``policy``/``cage`` importable when the repo isn't pip-installed.

    Colcon installs this node without the repo root on sys.path; walking up
    to the directory containing ``policy/baseline_pd.py`` recovers it. A
    prior ``pip install -e .`` makes this a no-op (the preferred setup).
    """
    try:
        import policy.baseline_pd  # noqa: F401
        return
    except ImportError:
        pass
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "policy" / "baseline_pd.py"
        if candidate.is_file():
            if str(parent) not in sys.path:
                sys.path.insert(0, str(parent))
            return


_bootstrap_policy_import()

import rclpy  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSPresetProfiles  # noqa: E402
from std_msgs.msg import Float64MultiArray  # noqa: E402

from cage.rules import State  # noqa: E402
from policy.baseline_pd import BaselinePD  # noqa: E402


class PdBaselineNode(Node):
    """ROS2 wrapper around the pure-Python PD baseline (see module docstring)."""

    def __init__(self) -> None:
        super().__init__("pd_baseline")

        self.declare_parameter("baseline_yaml", "")
        self.declare_parameter("state_obs_topic", "/state_obs")
        self.declare_parameter("raw_action_topic", "/raw_action")

        baseline_yaml = self._resolve_baseline_yaml()
        self._pd = BaselinePD.from_yaml(baseline_yaml)
        self.get_logger().info(f"BaselinePD loaded from {baseline_yaml}.")

        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value
        self.create_subscription(
            Float64MultiArray,
            self.get_parameter("state_obs_topic").value,
            self._on_state_obs,
            sensor_qos,
        )
        self._pub = self.create_publisher(
            Twist,
            self.get_parameter("raw_action_topic").value,
            sensor_qos,
        )

    def _resolve_baseline_yaml(self) -> str:
        """PD config path: the ``baseline_yaml`` param or the repo's policy/baseline_pd.yaml."""
        param = self.get_parameter("baseline_yaml").get_parameter_value().string_value
        if param:
            return param
        here = Path(__file__).resolve()
        for parent in here.parents:
            candidate = parent / "policy" / "baseline_pd.yaml"
            if candidate.is_file():
                return str(candidate)
        raise RuntimeError(
            "baseline_yaml parameter empty and policy/baseline_pd.yaml not "
            "found by walking up from the node source file."
        )

    def _on_state_obs(self, msg: Float64MultiArray) -> None:
        """Run one PD step on each /state_obs sample and publish /raw_action."""
        if len(msg.data) < 7:
            self.get_logger().warning(
                f"/state_obs has {len(msg.data)} fields, expected 7; ignoring."
            )
            return

        state = State(
            lateral_offset=float(msg.data[0]),
            heading_error=float(msg.data[1]),
            speed=float(msg.data[2]),
            curvature_ahead=float(msg.data[3]),
            distance_left=float(msg.data[4]),
            distance_right=float(msg.data[5]),
            state_valid=bool(msg.data[6] >= 0.5),
        )
        current_t = float(self.get_clock().now().nanoseconds) * 1e-9
        steering, throttle = self._pd.step(state, current_t=current_t)

        out = Twist()
        out.angular.z = float(steering)
        out.linear.x = float(throttle)
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PdBaselineNode()
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
