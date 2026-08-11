"""
vehicle_control_node — relay /safe_action to /cmd_vel.

For the F2 demo (1D-steering decision), `fixed_speed_mps` is the straight
cruise speed. When `use_safe_throttle` is true, /safe_action.linear.x
scales that cruise speed so the PD and C-04 curve slowdown are actuated
instead of only logged. Emergency mode is honoured via /emergency: when
latched True, /cmd_vel.linear.x is forced to zero so the controlled stop
of C-05 is actuated.

The steering on /cmd_vel.angular.z always comes from /safe_action.angular.z.

`speed_map` selects which of the two actuation maps in `cage_bridge` this node
reproduces — they are NOT interchangeable, and picking the wrong one silently
changes what the cage's speed authority means:

  * `cruise_scaling` (default, the frozen 1-D/F2 contract) —
    `cage_bridge.target_speed_from_throttle`: scale `fixed_speed_mps` by
    safe_throttle/`throttle_nominal`, clamped to [`min_speed_scale`, 1]. The
    floor is deliberate there: the 1-D policy has no speed authority, so the
    cage may only attenuate, never stop, outside C-05.
  * `linear_2d` (the 2-D `steer_throttle` contract, D-50/D-66) —
    `cage_bridge.target_speed_from_throttle_2d`: `max_speed_mps · u` with a full
    stop below `throttle_deadband` and NO lower clamp, so C-04 has attenuation
    authority all the way to zero and a stall stays commandable (SR-009).
    Under `cruise_scaling` a 2-D checkpoint would reach full speed already at
    u = `throttle_nominal` (0.5) and could never be brought below
    `min_speed_scale`·`fixed_speed_mps`.

The map is imported from `cage_bridge`, not re-implemented, so the deployed
speed cannot drift from the one the campaign scored.
"""

from __future__ import annotations

from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from std_msgs.msg import Bool

from .cage_bridge import target_speed_from_throttle_2d


class VehicleControlNode(Node):
    """Actuation relay /safe_action → /cmd_vel with emergency + watchdog logic."""

    def __init__(self) -> None:
        super().__init__("vehicle_control")

        self.declare_parameter("fixed_speed_mps", 0.2)
        self.declare_parameter("steering_to_yaw_rate_gain", 0.8)
        self.declare_parameter("use_safe_throttle", True)
        self.declare_parameter("throttle_nominal", 0.5)
        self.declare_parameter("min_speed_scale", 0.35)
        # Which cage_bridge actuation map to reproduce (see module docstring).
        self.declare_parameter("speed_map", "cruise_scaling")
        self.declare_parameter("throttle_deadband", 0.05)
        self.declare_parameter("safe_action_topic", "/safe_action")
        self.declare_parameter("emergency_topic", "/emergency")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        # Stale-command watchdog. If /safe_action stops arriving (cage
        # crash, perception stuck), the Gazebo DiffDrive plugin would hold
        # the last cmd_vel and the robot would keep moving unsupervised.
        # The watchdog publishes a zero Twist if no /safe_action has been
        # received in the last `safe_action_timeout_s` seconds.
        self.declare_parameter("safe_action_timeout_s", 0.5)

        self._fixed_speed = float(
            self.get_parameter("fixed_speed_mps").get_parameter_value().double_value
            or 0.2
        )
        # The DiffDrive plugin interprets /cmd_vel.angular.z as a yaw rate
        # in rad/s. The cage / PD operate on a normalised steering in
        # [-1, 1]; this gain maps one to the other. Default 0.8 gives a
        # min turn radius of v/omega = 0.2/0.8 = 0.25 m at v=0.2 m/s,
        # leaving comfortable headroom over the oval's R=0.8 m curves;
        # the previous 0.4 left only ~0.5 m and caused the PD to
        # saturate against C-05 on the curve.
        self._yaw_gain = float(
            self.get_parameter("steering_to_yaw_rate_gain")
            .get_parameter_value()
            .double_value
            or 0.8
        )
        self._use_safe_throttle = bool(
            self.get_parameter("use_safe_throttle").get_parameter_value().bool_value
        )
        self._throttle_nominal = max(
            1e-6,
            float(
                self.get_parameter("throttle_nominal")
                .get_parameter_value()
                .double_value
                or 0.5
            ),
        )
        self._min_speed_scale = max(
            0.0,
            min(
                1.0,
                float(
                    self.get_parameter("min_speed_scale")
                    .get_parameter_value()
                    .double_value
                    or 0.35
                ),
            ),
        )
        self._speed_map = (
            self.get_parameter("speed_map").get_parameter_value().string_value
            or "cruise_scaling"
        )
        if self._speed_map not in ("cruise_scaling", "linear_2d"):
            raise RuntimeError(
                "vehicle_control_node: speed_map must be 'cruise_scaling' or "
                f"'linear_2d', got {self._speed_map!r}"
            )
        self._throttle_deadband = float(
            self.get_parameter("throttle_deadband").get_parameter_value().double_value
            or 0.05
        )
        self._emergency = False
        self._last_safe: Optional[Twist] = None
        # Wall-clock timestamp of the last /safe_action received. Wall time
        # (not sim time) so the watchdog also fires if Gazebo dies, pauses,
        # or the EKF clock has not started yet.
        self._wall_clock = Clock()
        self._last_safe_wall_t: Optional[float] = None
        self._safe_action_timeout = float(
            self.get_parameter("safe_action_timeout_s")
            .get_parameter_value()
            .double_value
            or 0.5
        )
        self._watchdog_warned = False

        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value
        reliable_qos = QoSPresetProfiles.SYSTEM_DEFAULT.value

        self.create_subscription(
            Twist,
            self.get_parameter("safe_action_topic").value,
            self._on_safe,
            sensor_qos,
        )
        self.create_subscription(
            Bool,
            self.get_parameter("emergency_topic").value,
            self._on_emergency,
            reliable_qos,
        )
        self._pub = self.create_publisher(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
            reliable_qos,
        )

        # Watchdog tick: 10 Hz on wall clock, twice the default 0.5 s
        # timeout. Fires regardless of sim time so a paused/dead Gazebo
        # cannot mask the loss of /safe_action.
        self.create_timer(0.1, self._on_watchdog_tick, clock=Clock())

        self.get_logger().info(
            f"Relaying {self.get_parameter('safe_action_topic').value} -> "
            f"{self.get_parameter('cmd_vel_topic').value} "
            f"(fixed_speed={self._fixed_speed} m/s, "
            f"use_safe_throttle={self._use_safe_throttle}, "
            f"speed_map={self._speed_map}, emergency-aware)."
        )

    def _on_emergency(self, msg: Bool) -> None:
        self._emergency = bool(msg.data)

    def _wall_now_s(self) -> float:
        return float(self._wall_clock.now().nanoseconds) * 1e-9

    def _on_watchdog_tick(self) -> None:
        """Command a zero Twist whenever /safe_action has gone stale (see init)."""
        if self._last_safe_wall_t is None:
            # No /safe_action ever — no actuator command yet. Nothing to
            # override; cage_logger watchdog already surfaces this case.
            return
        elapsed = self._wall_now_s() - self._last_safe_wall_t
        if elapsed > self._safe_action_timeout:
            stop = Twist()
            stop.linear.x = 0.0
            stop.angular.z = 0.0
            self._pub.publish(stop)
            if not self._watchdog_warned:
                self.get_logger().error(
                    "No /safe_action in %.2f s (> %.2f s timeout); commanding "
                    "stop on /cmd_vel until the upstream pipeline recovers."
                    % (elapsed, self._safe_action_timeout)
                )
                self._watchdog_warned = True
        else:
            self._watchdog_warned = False

    def _on_safe(self, msg: Twist) -> None:
        """Map each /safe_action to /cmd_vel (or a full stop while emergency is latched)."""
        self._last_safe = msg
        self._last_safe_wall_t = self._wall_now_s()
        cmd = Twist()
        if self._emergency:
            # Controlled stop: zero both axes so the robot does not pivot
            # in place while linear.x is being braked. The cage may still
            # publish a frozen non-zero safe_action.angular.z (cf.
            # cage.yaml c05_emergency.freeze_steering), but actuating it
            # against linear.x=0 produces pure yaw, not a stop.
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            cmd.angular.z = float(msg.angular.z) * self._yaw_gain
            cmd.linear.x = self._target_speed_from_throttle(float(msg.linear.x))
        self._pub.publish(cmd)

    def _target_speed_from_throttle(self, safe_throttle: float) -> float:
        """Cage safe throttle → commanded speed, on the configured `speed_map`."""
        if not self._use_safe_throttle:
            return self._fixed_speed

        if self._speed_map == "linear_2d":
            # `fixed_speed_mps` IS action.max_speed_mps on this map (the launch
            # sets both from the same value), so u = 1 gives the contract's top
            # speed and u < deadband gives a true stop.
            return target_speed_from_throttle_2d(
                safe_throttle, self._fixed_speed, self._throttle_deadband
            )

        throttle = max(0.0, min(self._throttle_nominal, safe_throttle))
        if throttle <= 1e-9:
            return 0.0

        scale = throttle / self._throttle_nominal
        scale = max(self._min_speed_scale, min(1.0, scale))
        return self._fixed_speed * scale


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VehicleControlNode()
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
