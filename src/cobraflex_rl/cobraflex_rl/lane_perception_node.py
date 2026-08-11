"""
lane_perception_node — ground-truth Perception for the F2 pipeline.

Consumes:
    /odom    nav_msgs/Odometry

Publishes:
    /state_obs   std_msgs/Float64MultiArray with fixed ordering:
        [lateral_offset_m,
         heading_error_rad,
         speed_mps,
         curvature_ahead_inv_m,
         distance_left_m,
         distance_right_m,
         state_valid (1.0/0.0)]

Centerline source: YAML at the path given by parameter `centerline_yaml`.
Curvature look-ahead: the heading delta between the current segment and
the segment `lookahead_segments` ahead, divided by their arc length.

The node is the F2 "sim-only" Perception (cf. fase_2_detallada.md §9.1).
Sensor-noise and OOD handling come later; in F2 the only conditions
that force `state_valid=False` are:
    - never received /odom yet
    - pose contains NaN/Inf
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from std_msgs.msg import Float64MultiArray

from .polyline_tracker import PolylineTracker


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    """Yaw (rad) from a quaternion (planar robot: roll/pitch ignored)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class LanePerceptionNode(Node):
    """Odometry → /state_obs publisher with EMA smoothing and spike/warp guards."""

    def __init__(self) -> None:
        super().__init__("lane_perception")

        self.declare_parameter("centerline_yaml", "")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("lookahead_segments", 5)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("state_obs_topic", "/state_obs")
        # EMA smoothing for ey/epsi. Raw skid-steer encoder odom oscillates
        # ~20 cm between consecutive 50 ms ticks at the curve section, causing
        # ey to alternate between -0.14 and +0.06 and triggering C-05 Trigger
        # 7. The EKF-filtered odom (/odometry/filtered) is already smooth, so
        # a light alpha=0.7 is sufficient to suppress residual quantisation
        # noise while keeping PD response fast. For the raw Gazebo odom case,
        # a lower alpha (0.3–0.5) is needed.
        self.declare_parameter("ema_alpha", 0.7)

        centerline_path = self._resolve_centerline_path()
        with Path(centerline_path).open("r", encoding="utf-8") as handle:
            cfg = yaml.safe_load(handle)

        centerline_points = np.asarray(cfg["centerline"]["points"], dtype=float)
        self._lane_width = float(cfg["lane_width"])
        self._road_width = float(cfg.get("road_width", self._lane_width))
        self._tracker = PolylineTracker(centerline_points)
        self._lookahead = int(
            self.get_parameter("lookahead_segments").get_parameter_value().integer_value
            or 5
        )
        self._ema_alpha = float(self.get_parameter("ema_alpha").value)
        self._ey_smooth: Optional[float] = None
        self._epsi_smooth: Optional[float] = None
        # Warp detection: if /odom jumps by more than this distance between
        # consecutive frames, treat it as a teleport (Gazebo respawn, RL
        # episode reset) and drop the tracker's neighbourhood cache + EMA
        # state so they don't lock on a stale segment.
        self.declare_parameter("warp_threshold_m", 0.5)
        self._warp_threshold = float(self.get_parameter("warp_threshold_m").value)
        self._last_xy: Optional[tuple] = None
        # Gazebo physics spikes: a single odom sample that is >factor× the
        # smoothed speed is physically implausible (real vehicle can't change
        # speed that fast at 20 Hz) and would trip C-03/C-04/C-05. The spike
        # is held at the previous smooth value and a WARNING is emitted.
        # factor=5 → flags anything above ~0.35 m/s when cruising at 0.07 m/s,
        # while allowing genuine acceleration transients.
        self.declare_parameter("speed_spike_factor", 5.0)
        self._speed_spike_factor = float(self.get_parameter("speed_spike_factor").value)
        self._speed_smooth: Optional[float] = None

        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value
        self._odom_msg: Optional[Odometry] = None
        self._got_odom_once = False

        self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self._on_odom,
            sensor_qos,
        )
        self._pub = self.create_publisher(
            Float64MultiArray,
            self.get_parameter("state_obs_topic").value,
            sensor_qos,
        )

        period = 1.0 / float(self.get_parameter("publish_rate_hz").value or 20.0)
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f"Centerline loaded: {len(centerline_points)} points, "
            f"perimeter={self._tracker.cumulative_lengths[-1]:.3f} m, "
            f"lane_width={self._lane_width} m, road_width={self._road_width} m."
        )

    def _resolve_centerline_path(self) -> str:
        """Centerline YAML: the ``centerline_yaml`` param or the package config default."""
        param = self.get_parameter("centerline_yaml").get_parameter_value().string_value
        if param:
            return param

        here = Path(__file__).resolve()
        for parent in here.parents:
            candidate = parent / "config" / "oval_centerline.yaml"
            if candidate.is_file():
                return str(candidate)
        raise RuntimeError(
            "centerline_yaml parameter is empty and config/oval_centerline.yaml "
            "could not be located by walking up from the node source file."
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._odom_msg = msg
        self._got_odom_once = True

    def _tick(self) -> None:
        """Publish one /state_obs sample from the latest odom (see module docstring).

        Pipeline: validity checks → warp detection → centerline projection →
        speed-spike rejection → EMA smoothing → boundary distances.
        """
        msg = Float64MultiArray()
        if not self._got_odom_once or self._odom_msg is None:
            # Publishing state_valid=0.0 before odom arrives activates C-05
            # Trigger 4 (invalid state), which latches permanently with
            # require_explicit_reset=True. Suppress the publish entirely so
            # the cage_ros_node calls cage.step(None) and takes the safe
            # _no_state_ever_result path (emergency but C-05 not latched).
            return
        # Note: plausibility gate below also suppresses on the publish path.

        pose = self._odom_msg.pose.pose
        x = float(pose.position.x)
        y = float(pose.position.y)
        yaw = _yaw_from_quat(
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        )

        lin = self._odom_msg.twist.twist.linear
        speed_raw = math.sqrt(float(lin.x) ** 2 + float(lin.y) ** 2)

        if not (
            math.isfinite(x)
            and math.isfinite(y)
            and math.isfinite(yaw)
            and math.isfinite(speed_raw)
        ):
            msg.data = [
                0.0,
                0.0,
                0.0,
                0.0,
                self._road_width / 2,
                self._road_width / 2,
                0.0,
            ]
            self._pub.publish(msg)
            return

        if self._last_xy is not None:
            dx = x - self._last_xy[0]
            dy = y - self._last_xy[1]
            if (dx * dx + dy * dy) ** 0.5 > self._warp_threshold:
                self.get_logger().warning(
                    "Odom warp detected (>%.2f m jump); resetting polyline "
                    "tracker and EMA." % self._warp_threshold
                )
                self._tracker.reset_tracking()
                self._ey_smooth = None
                self._epsi_smooth = None
                self._speed_smooth = None
        self._last_xy = (x, y)

        track_state = self._tracker.track(x, y, yaw)
        kappa_ahead = self._curvature_ahead(track_state.segment_index)

        ey_raw = float(track_state.ey)
        epsi_raw = float(track_state.epsi)

        # Spike rejection for speed: clamp before EMA so one bad odom sample
        # cannot propagate into the cage state. Minimum reference of 0.01 m/s
        # avoids a divide-by-zero / always-spike condition at standstill.
        if self._speed_smooth is not None:
            threshold = self._speed_spike_factor * max(self._speed_smooth, 0.01)
            if speed_raw > threshold:
                self.get_logger().warning(
                    "Speed spike rejected: %.3f m/s (smooth=%.3f m/s)" % (
                        speed_raw, self._speed_smooth
                    )
                )
                speed_raw = self._speed_smooth

        # EMA smoothing: initialise to raw value on first observation so
        # there is no ramp-up transient at startup.
        if self._ey_smooth is None:
            self._ey_smooth = ey_raw
            self._epsi_smooth = epsi_raw
            self._speed_smooth = speed_raw
        else:
            a = self._ema_alpha
            self._ey_smooth = a * ey_raw + (1.0 - a) * self._ey_smooth
            # epsi lives in (-pi, pi]; a naive EMA across the seam averages
            # +3.10 with -3.10 to ~0 instead of staying near pi. Apply EMA
            # to the wrapped *delta* and re-wrap the result so the smoothed
            # estimate tracks the shortest-arc trajectory.
            delta = math.atan2(
                math.sin(epsi_raw - self._epsi_smooth),
                math.cos(epsi_raw - self._epsi_smooth),
            )
            updated = self._epsi_smooth + a * delta
            self._epsi_smooth = math.atan2(math.sin(updated), math.cos(updated))
            self._speed_smooth = a * speed_raw + (1.0 - a) * self._speed_smooth

        ey = self._ey_smooth
        epsi = self._epsi_smooth
        speed = self._speed_smooth
        d_left = max(0.0, (self._road_width / 2) - ey)
        d_right = max(0.0, (self._road_width / 2) + ey)

        msg.data = [ey, epsi, speed, kappa_ahead, d_left, d_right, 1.0]
        self._pub.publish(msg)

    def _curvature_ahead(self, segment_index: int) -> float:
        return self._tracker.curvature_ahead(segment_index, self._lookahead)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LanePerceptionNode()
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
