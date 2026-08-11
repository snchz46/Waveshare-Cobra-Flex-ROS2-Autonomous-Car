"""
cv_lane_estimator_node — ROS2 wrapper for the cage's CV lane-estimator (D-43).

Deployment analogue of the in-process path used by `gazebo_lane_env`: replaces
`lane_perception_node` (odom + authored centerline) as the source of the
cage's `/state_obs` on track 'E', so the cage generalises to any road with
visible lane lines.

Subscribes:
    camera/image_raw_lane  sensor_msgs/Image  (the same frames the policy sees)
    odom_topic         nav_msgs/Odometry      (speed only — plausibility + cage state)

    ``odom_topic`` defaults to ``/odom``, which is what Gazebo publishes. **On the
    physical platform nothing publishes ``/odom``** — `cobraflex_ros_driver` emits
    `/cobraflex/wheel_speeds`, and the odometry is the EKF's `/odometry/filtered`
    (`cobraflex_sensors.launch.xml` → `robot_localization` over the ZED). Left at
    the default there, `speed` stays 0.0 forever and the cage's speed-dependent
    rules go silently inert: C-03 sees every TTLC as infinite (speed below
    `v_min_estimate_mps`), C-04 never finds an excess over `v_max_curve`, and
    C-05's high-energy variant (`v_warning_mps`) can never arm. Nothing errors —
    the rules just never fire. `deploy_cobraflex.launch.py` therefore passes
    `/odometry/filtered`, and this node warns if no odometry ever arrives.

Perception contract (D-43): the estimator parameters below are NOT cosmetic
tunables — they select which lane readout the cage acts on. The frozen GE4/G4
record used `near_secant` with gain 1.0 (the defaults here); the posterior
Gazebo contract the 2-D PPO 550k trunk was trained and scored against uses
`joint_pair_quadratic`, gain 1.6 and the T3 temporal gate (window 4). Deploying
with the defaults would put a *different* heading estimate under C-02/C-03 than
the campaign measured. The authoritative values live in the training config's
`cage:` block (`perception_heading_*`), and the deploy launch forwards them.

Publishes (at ``publish_rate_hz``):
    /state_obs           std_msgs/Float64MultiArray
                         [ey, epsi, speed, kappa_ahead, d_left, d_right, 1.0]
                         — same fixed ordering as lane_perception_node. The
                         publish is SUPPRESSED while the supervisor has no
                         acceptable estimate (F2 precedent: the cage's
                         missing-state path handles the gap; publishing
                         state_valid=0 would latch C-05 Trigger 4).
    /perception_invalid  std_msgs/Bool — the C-05 Trigger 8 input
                         (cage_ros_node latches it into ctx like
                         /external_stop). Published every tick.

The estimator, health monitor and plausibility check are the host-tested pure
modules (`cv_lane_estimator`, `perception_health`, `lane_plausibility`)
composed by `cage_perception.CagePerceptionSupervisor` — identical logic to
training/eval, only the transport differs.

Clock domain — run with ``use_sim_time:=true`` in simulation. The gz bridge
stamps camera frames with sim time; the supervisor's staleness check compares
those stamps against this node's clock, so a wall-clock node sees every frame
as ancient and latches perception_invalid permanently.
"""
from __future__ import annotations

import math
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64MultiArray

from .cage_perception import CagePerceptionSupervisor
from .camera_pipeline import decode_image
from .cv_lane_estimator import CvLaneEstimator, CvLaneEstimatorConfig


class CvLaneEstimatorNode(Node):
    """Camera → /state_obs + /perception_invalid via the perception supervisor."""

    def __init__(self) -> None:
        super().__init__("cv_lane_estimator_node")
        self.declare_parameter("image_topic", "camera/image_raw_lane")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("state_obs_topic", "/state_obs")
        self.declare_parameter("perception_invalid_topic", "/perception_invalid")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("road_width_m", 0.52)
        self.declare_parameter("heading_fit_mode", "near_secant")
        self.declare_parameter("heading_gain", 1.0)
        # Rest of the D-43 estimator contract (see module docstring). Defaults
        # reproduce the frozen GE4 estimator bit-identically, so a launch that
        # passes none of them keeps the old behaviour.
        self.declare_parameter("heading_bias_rad", 0.0)
        self.declare_parameter("heading_temporal_window", 0)
        self.declare_parameter("heading_temporal_ey_track_m", -1.0)
        self.declare_parameter("heading_temporal_ey_drift_m", -1.0)
        self.declare_parameter("heading_temporal_kappa_gate", -1.0)
        self.declare_parameter("heading_temporal_cap_rad", -1.0)
        # Supervisor invalid-persistence budget; <0 keeps CagePerceptionSupervisor's
        # own live-tuned default (min_invalid_cycles=4 at the 10 Hz control rate).
        self.declare_parameter("perception_min_invalid_cycles", -1)

        cfg_kwargs = dict(
            heading_fit_mode=str(self.get_parameter("heading_fit_mode").value),
            heading_gain=float(self.get_parameter("heading_gain").value),
            heading_bias_rad=float(self.get_parameter("heading_bias_rad").value),
            heading_temporal_window=int(
                self.get_parameter("heading_temporal_window").value
            ),
        )
        # A negative value means "not set" — ROS2 parameters have no null, and
        # every one of these is a physical quantity that is >= 0 when meant.
        for _key in (
            "heading_temporal_ey_track_m",
            "heading_temporal_ey_drift_m",
            "heading_temporal_kappa_gate",
            "heading_temporal_cap_rad",
        ):
            _val = float(self.get_parameter(_key).value)
            if _val >= 0.0:
                cfg_kwargs[_key] = _val

        estimator = CvLaneEstimator(config=CvLaneEstimatorConfig(**cfg_kwargs))
        sup_kwargs = {"estimator": estimator}
        _min_invalid = int(self.get_parameter("perception_min_invalid_cycles").value)
        if _min_invalid >= 0:
            from .perception_health import PerceptionHealthMonitor

            sup_kwargs["health"] = PerceptionHealthMonitor(
                min_confidence=0.10, min_invalid_cycles=_min_invalid
            )
        self._supervisor = CagePerceptionSupervisor(**sup_kwargs)
        self._image_msg: Optional[Image] = None
        self._speed = 0.0
        self._odom_seen = False
        self._odom_warned = False
        self._road_width = float(self.get_parameter("road_width_m").value)

        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self._on_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._on_odom,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self._state_pub = self.create_publisher(
            Float64MultiArray, str(self.get_parameter("state_obs_topic").value), 10
        )
        self._invalid_pub = self.create_publisher(
            Bool, str(self.get_parameter("perception_invalid_topic").value), 10
        )
        rate = float(self.get_parameter("publish_rate_hz").value or 10.0)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            "cv_lane_estimator_node up (D-43 cage perception source; "
            f"heading_fit_mode={estimator.config.heading_fit_mode}, "
            f"heading_gain={estimator.config.heading_gain:.3f}, "
            f"heading_temporal_window={estimator.config.heading_temporal_window}, "
            f"odom_topic={self.get_parameter('odom_topic').value})"
        )

    def _on_image(self, msg: Image) -> None:
        self._image_msg = msg

    def _on_odom(self, msg: Odometry) -> None:
        """Track planar speed only (the estimator never sees the odom pose)."""
        lin = msg.twist.twist.linear
        speed = math.sqrt(float(lin.x) ** 2 + float(lin.y) ** 2)
        if math.isfinite(speed):
            self._speed = speed
            self._odom_seen = True

    def _tick(self) -> None:
        """One supervisor cycle: decode the latest frame, publish invalid flag + state."""
        if not self._odom_seen and not self._odom_warned and self._image_msg is not None:
            # Frames are flowing but odometry is not: the cage will run with
            # speed = 0 and its speed-dependent rules (C-03 TTLC, C-04, C-05
            # high-energy) can never fire. Silent by construction — say it once.
            self._odom_warned = True
            self.get_logger().error(
                "camera frames are arriving but no odometry on "
                f"'{self.get_parameter('odom_topic').value}' — publishing speed 0.0. "
                "C-03/C-04 and C-05's high-energy trigger are inert until this is "
                "fixed (physical platform: /odometry/filtered from the EKF)."
            )
        now = self.get_clock().now().nanoseconds * 1e-9
        frame = None
        stamp = 0.0
        msg = self._image_msg
        if msg is not None:
            try:
                frame = decode_image(
                    msg.data, int(msg.height), int(msg.width),
                    msg.encoding, int(msg.step),
                )
                stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            except ValueError as exc:
                self.get_logger().warning(f"camera decode failed: {exc}")

        result = self._supervisor.update(
            frame, frame_timestamp_s=stamp, now_s=now, speed_mps=self._speed
        )

        invalid = Bool()
        invalid.data = bool(result.perception_invalid)
        self._invalid_pub.publish(invalid)

        if not result.state_available:
            return  # suppressed publish → cage missing-state path (F2 precedent)
        est = result.estimate
        half = self._road_width / 2.0
        out = Float64MultiArray()
        out.data = [
            float(est.ey),
            float(est.epsi),
            float(self._speed),
            float(est.curvature),
            max(0.0, half - est.ey),
            max(0.0, half + est.ey),
            1.0,
        ]
        self._state_pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CvLaneEstimatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
