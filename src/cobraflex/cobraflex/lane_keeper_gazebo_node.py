#!/usr/bin/env python3
"""Camera lane keeper for Gazebo.

Logical (non-learned) lane-keeping controller, the fair baseline for the RL
camera agent. It uses the shared :class:`cobraflex_rl.cv_lane_controller.CVLaneController`
— the deterministic CV lane estimator (D-43, the same the safety cage reads)
plus a PD + curvature-feedforward law — so this deployment node and the scored
evaluation (``cobraflex_rl.eval_cv_controller``) drive identically.

It supersedes the previous histogram pure-P controller, whose uncalibrated
"lane centre = image centre" set-point could not hold the lane above ~0.1 m/s.
The CV+PD law tracks the nominal oval to RMSE ~10 mm at 0.2 m/s (req < 50 mm).
"""

import array
import time

import cv2
import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

from cobraflex_rl.cv_lane_controller import CVLaneController


def _ros_image_to_bgr(msg: Image) -> np.ndarray:
    """Convert a ROS Image message to a BGR OpenCV frame."""
    encoding = msg.encoding.lower()
    channels_by_encoding = {
        "mono8": 1,
        "8uc1": 1,
        "bgr8": 3,
        "rgb8": 3,
        "8uc3": 3,
        "bgra8": 4,
        "rgba8": 4,
        "8uc4": 4,
    }

    channels = channels_by_encoding.get(encoding)
    if channels is None:
        raise ValueError(f"Unsupported image encoding: {msg.encoding}")

    row_stride = int(msg.step) if msg.step > 0 else int(msg.width * channels)
    expected_size = int(msg.height * row_stride)
    data = np.frombuffer(msg.data, dtype=np.uint8)

    if data.size < expected_size:
        raise ValueError(
            f"Image buffer too small for {msg.encoding}: {data.size} < {expected_size}"
        )

    rows = data[:expected_size].reshape((msg.height, row_stride))

    if channels == 1:
        gray = np.ascontiguousarray(rows[:, : msg.width])
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    image = rows[:, : msg.width * channels].reshape((msg.height, msg.width, channels))
    image = np.ascontiguousarray(image)

    if channels == 3:
        if encoding in ("bgr8", "8uc3"):
            return image
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    if encoding in ("bgra8", "8uc4"):
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)


class LaneKeeperGazeboNode(Node):
    """Camera lane keeper: calibrated CV lane estimate + PD/feedforward steering."""

    def __init__(self):
        super().__init__("lane_keeper_gazebo_node")

        self.declare_parameter("image_topic", "camera/image_raw_lane")
        self.declare_parameter("linear_speed", 0.20)
        # Pure-pursuit law (CVLaneController): aim at the lane centre look_ahead_m
        # ahead. The legacy PD/feedforward gains are kept declared for backward
        # compatibility but no longer affect the control (the controller ignores
        # them); see docs/12 §3.
        self.declare_parameter("look_ahead_m", 0.40)
        self.declare_parameter("pursuit_gain", 1.0)
        self.declare_parameter("kp_ey", 6.0)
        self.declare_parameter("kd_epsi", 1.6)
        self.declare_parameter("kff_curv", 1.0)
        self.declare_parameter("max_angular_z", 0.9)
        # Stop (vs coast straight) when the estimator finds no usable lane.
        self.declare_parameter("stop_on_no_lane", True)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("show_debug_windows", False)
        self.declare_parameter("watchdog_timeout_sec", 1.5)

        debug_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.controller = CVLaneController(
            speed=float(self.get_parameter("linear_speed").value),
            look_ahead_m=float(self.get_parameter("look_ahead_m").value),
            pursuit_gain=float(self.get_parameter("pursuit_gain").value),
            max_angular_z=float(self.get_parameter("max_angular_z").value),
        )

        image_topic = str(self.get_parameter("image_topic").value)
        self.image_sub = self.create_subscription(
            Image, image_topic, self._image_callback, qos_profile_sensor_data
        )
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        # Debug image topics (view in RViz Image displays instead of cv2 windows):
        #   /lane/image_overlay — camera + detections + error bars
        #   /lane/mask          — the white-mask filter (mono8)
        self.debug_pub = self.create_publisher(Image, "/lane/image_overlay", debug_qos)
        self.mask_pub = self.create_publisher(Image, "/lane/mask", debug_qos)

        self.last_frame_time = 0.0
        self.last_warn_time = 0.0
        self.timer = self.create_timer(0.2, self._watchdog_callback)

        self.get_logger().info(
            f"lane_keeper_gazebo_node (CV+PD) listening on '{image_topic}'"
        )

    def _build_image_msg(self, image, stamp, frame_id):
        """Wrap a numpy image as a sensor_msgs/Image (bgr8 for 3-ch, mono8 for 2-D)."""
        if not image.flags["C_CONTIGUOUS"]:
            image = np.ascontiguousarray(image)
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = int(image.shape[0])
        msg.width = int(image.shape[1])
        msg.encoding = "mono8" if image.ndim == 2 else "bgr8"
        msg.is_bigendian = False
        msg.step = int(image.strides[0])
        # array.array('B', ...): the only fast path through rclpy's uint8[]
        # setter — see lane_keeper_node._build_image_msg for the measurement.
        msg.data = array.array("B", image.tobytes())
        return msg

    def _publish_zero(self):
        """Publish a zero Twist (stop)."""
        self.cmd_pub.publish(Twist())

    def _watchdog_callback(self):
        """Stop the robot when no camera frame arrived within the watchdog window."""
        timeout = float(self.get_parameter("watchdog_timeout_sec").value)
        if timeout <= 0.0 or self.last_frame_time <= 0.0:
            return
        now = time.time()
        if now - self.last_frame_time > timeout:
            self._publish_zero()
            if now - self.last_warn_time >= 1.0:
                self.get_logger().warning("No camera frames received, publishing zero cmd_vel")
                self.last_warn_time = now

    def _image_callback(self, msg: Image):
        """Per-frame control tick: CV estimate → PD/feedforward steering → /cmd_vel."""
        try:
            frame_bgr = _ros_image_to_bgr(msg)
        except ValueError as exc:
            self.get_logger().warning(f"Could not decode camera image: {exc}")
            return

        self.last_frame_time = time.time()
        angular, detected = self.controller.compute(frame_bgr)

        cmd = Twist()
        if detected:
            cmd.linear.x = float(self.get_parameter("linear_speed").value)
            cmd.angular.z = float(angular)
        elif not bool(self.get_parameter("stop_on_no_lane").value):
            cmd.linear.x = float(self.get_parameter("linear_speed").value)
        self.cmd_pub.publish(cmd)

        if bool(self.get_parameter("publish_debug_image").value) or bool(
            self.get_parameter("show_debug_windows").value
        ):
            try:
                mask = self.controller.estimator.white_mask(frame_bgr)
            except Exception:  # pragma: no cover - best effort
                mask = np.zeros(frame_bgr.shape[:2], dtype=np.uint8)
            overlay = self._render_overlay(frame_bgr, mask, cmd)
            if bool(self.get_parameter("publish_debug_image").value):
                self.debug_pub.publish(
                    self._build_image_msg(overlay, msg.header.stamp, msg.header.frame_id)
                )
                self.mask_pub.publish(
                    self._build_image_msg(mask, msg.header.stamp, msg.header.frame_id)
                )
            if bool(self.get_parameter("show_debug_windows").value):
                self._show_windows(frame_bgr, mask, overlay)

    def _render_overlay(self, frame_bgr, mask, cmd):
        """Camera frame + green mask tint + detected white-run points + error bars."""
        overlay = frame_bgr.copy()
        mask_bgr = np.zeros_like(overlay)
        mask_bgr[mask > 0] = (0, 255, 0)                       # detections in green
        overlay = cv2.addWeighted(overlay, 1.0, mask_bgr, 0.45, 0.0)
        # The per-row white-run centres the estimator actually used (red dots).
        for u, v in getattr(self.controller.estimator, "debug_candidates_px", []):
            cv2.circle(overlay, (int(u), int(v)), 3, (0, 0, 255), -1)
        d = self.controller.dbg
        if d.get("ok"):
            # CVLaneController.dbg (pure-pursuit, docs/12 §3) exposes
            # ok/ey/epsi/y_l/kappa_cmd/nL — there is no 'kappa' key (that was the
            # old PD law). Read via .get so a future dbg change can't crash the
            # node mid-callback.
            txt = (f"ey={d.get('ey', 0.0):+.3f}m  epsi={d.get('epsi', 0.0):+.3f}rad  "
                   f"k_cmd={d.get('kappa_cmd', 0.0):+.2f}  "
                   f"cmd=(v{cmd.linear.x:.2f}, w{cmd.angular.z:+.2f})")
            color = (0, 255, 255)
        else:
            txt = (f"NO LANE [{d.get('reason', '')}]  "
                   f"cmd=(v{cmd.linear.x:.2f}, w{cmd.angular.z:+.2f})")
            color = (0, 165, 255)
        cv2.putText(overlay, txt, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1,
                    cv2.LINE_AA)
        self._draw_error_bars(overlay, d, cmd)
        return overlay

    @staticmethod
    def _draw_error_bars(img, d, cmd):
        """Bottom panel of signed bars for the errors driving the command."""
        h, w = img.shape[:2]
        rows = [("ey (m)", d.get("ey", 0.0), 0.15),
                ("epsi (rad)", d.get("epsi", 0.0), 0.5),
                ("kappa_cmd", d.get("kappa_cmd", 0.0), 3.0),
                ("cmd w", cmd.angular.z, 1.0)]
        panel_h = 16 * len(rows) + 8
        y0 = h - panel_h
        cv2.rectangle(img, (0, y0), (w, h), (40, 40, 40), -1)
        cx = w // 2
        cv2.line(img, (cx, y0), (cx, h), (90, 90, 90), 1)
        for i, (label, val, full) in enumerate(rows):
            yc = y0 + 12 + i * 16
            frac = max(-1.0, min(1.0, val / full if full else 0.0))
            x_end = int(cx + frac * (w // 2 - 70))
            col = (0, 255, 255) if abs(frac) < 0.9 else (0, 0, 255)
            cv2.rectangle(img, (cx, yc - 5), (x_end, yc + 4), col, -1)
            cv2.putText(img, f"{label}:{val:+.3f}", (4, yc + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (220, 220, 220), 1, cv2.LINE_AA)

    def _show_windows(self, frame_bgr, mask, overlay):
        """Three OpenCV windows: raw camera, white-mask filter, detections+errors."""
        try:
            cv2.imshow("1 camera", frame_bgr)
            cv2.imshow("2 white mask (filter)", mask)
            cv2.imshow("3 detections + errors", overlay)
            cv2.waitKey(1)
        except cv2.error:
            if not getattr(self, "_warned_no_gui", False):
                self._warned_no_gui = True
                self.get_logger().warning(
                    "OpenCV has no GUI (headless build): cannot show debug windows. "
                    "Install GUI OpenCV (pip install opencv-python, not "
                    "opencv-python-headless) or view the published overlay with "
                    "`ros2 run rqt_image_view rqt_image_view /lane/image_overlay`.")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LaneKeeperGazeboNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Under `ros2 launch` a ctrl-c arrives as ExternalShutdownException,
        # not KeyboardInterrupt; letting it escape made the node exit 1.
        pass
    finally:
        if node is not None:
            node._publish_zero()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
