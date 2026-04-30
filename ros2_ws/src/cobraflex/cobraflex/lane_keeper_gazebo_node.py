#!/usr/bin/env python3
"""Minimal lane keeper for Gazebo using the bridged camera."""

import time

import cv2
import numpy as np

import rclpy
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


def _clamp(value, low, high):
    return max(low, min(value, high))


def _odd(value):
    value = max(1, int(value))
    if value % 2 == 0:
        value += 1
    return value


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
    """Minimal Gazebo lane keeper."""

    def __init__(self):
        super().__init__("lane_keeper_gazebo_node")

        self.declare_parameter("use_sim_time", True)
        self.declare_parameter("image_topic", "camera/image_raw")
        self.declare_parameter("proc_width", 480)
        self.declare_parameter("proc_height", 360)
        self.declare_parameter("roi_start_pct", 55)
        self.declare_parameter("white_sat_max", 70)
        self.declare_parameter("white_val_min", 150)
        self.declare_parameter("morph_k", 3)
        self.declare_parameter("peak_threshold", 6.0)
        self.declare_parameter("min_lane_width_px", 60.0)
        self.declare_parameter("lane_width_px", 110.0)
        self.declare_parameter("linear_speed", 0.10)
        self.declare_parameter("angular_gain", 1.25)
        self.declare_parameter("max_angular_z", 0.9)
        self.declare_parameter("min_linear_scale", 0.35)
        self.declare_parameter("single_side_scale", 0.65)
        self.declare_parameter("publish_debug_image", False)
        self.declare_parameter("show_debug_windows", False)
        self.declare_parameter("watchdog_timeout_sec", 1.5)

        debug_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        image_topic = str(self.get_parameter("image_topic").value)
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.debug_pub = self.create_publisher(Image, "/lane/image_overlay", debug_qos)

        self.last_frame_time = 0.0
        self.last_warn_time = 0.0
        self.lane_width_est = float(self.get_parameter("lane_width_px").value)
        self.timer = self.create_timer(0.2, self._watchdog_callback)

        self.get_logger().info(
            f"lane_keeper_gazebo_node listening on '{image_topic}'"
        )

    def _build_image_msg(self, image, stamp, frame_id):
        if not image.flags["C_CONTIGUOUS"]:
            image = np.ascontiguousarray(image)

        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = int(image.shape[0])
        msg.width = int(image.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = int(image.strides[0])
        msg.data = image.tobytes()
        return msg

    def _publish_zero(self):
        self.cmd_pub.publish(Twist())

    def _watchdog_callback(self):
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
        try:
            frame_bgr = _ros_image_to_bgr(msg)
        except ValueError as exc:
            self.get_logger().warning(f"Could not decode camera image: {exc}")
            return

        self.last_frame_time = time.time()

        cmd, debug_image = self._process_frame(frame_bgr)
        self.cmd_pub.publish(cmd)

        if bool(self.get_parameter("publish_debug_image").value):
            self.debug_pub.publish(
                self._build_image_msg(debug_image, msg.header.stamp, msg.header.frame_id)
            )

        if bool(self.get_parameter("show_debug_windows").value):
            cv2.imshow("Lane Keeper Gazebo", debug_image)
            cv2.waitKey(1)

    def _process_frame(self, frame_bgr):
        proc_width = int(self.get_parameter("proc_width").value)
        proc_height = int(self.get_parameter("proc_height").value)
        roi_start_pct = float(self.get_parameter("roi_start_pct").value)
        white_sat_max = int(self.get_parameter("white_sat_max").value)
        white_val_min = int(self.get_parameter("white_val_min").value)
        morph_k = _odd(self.get_parameter("morph_k").value)
        peak_threshold = float(self.get_parameter("peak_threshold").value)
        min_lane_width_px = float(self.get_parameter("min_lane_width_px").value)
        linear_speed = float(self.get_parameter("linear_speed").value)
        angular_gain = float(self.get_parameter("angular_gain").value)
        max_angular_z = float(self.get_parameter("max_angular_z").value)
        min_linear_scale = float(self.get_parameter("min_linear_scale").value)
        single_side_scale = float(self.get_parameter("single_side_scale").value)

        frame = cv2.resize(
            frame_bgr,
            (proc_width, proc_height),
            interpolation=cv2.INTER_AREA,
        )

        roi_y = int(proc_height * roi_start_pct / 100.0)
        roi_y = _clamp(roi_y, 0, proc_height - 1)
        roi = frame[roi_y:, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            np.array([0, 0, white_val_min], dtype=np.uint8),
            np.array([179, white_sat_max, 255], dtype=np.uint8),
        )

        kernel = np.ones((morph_k, morph_k), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.dilate(mask, kernel, iterations=1)

        histogram = mask.sum(axis=0).astype(np.float32) / 255.0
        histogram = np.convolve(histogram, np.ones(9, dtype=np.float32) / 9.0, mode="same")

        half = proc_width // 2
        left_hist = histogram[:half]
        right_hist = histogram[half:]

        left_x = None
        right_x = None

        if left_hist.size > 0 and float(left_hist.max()) >= peak_threshold:
            left_x = int(np.argmax(left_hist))

        if right_hist.size > 0 and float(right_hist.max()) >= peak_threshold:
            right_x = int(np.argmax(right_hist) + half)

        lane_center_x = None
        speed_scale = 1.0

        if (
            left_x is not None
            and right_x is not None
            and (right_x - left_x) >= min_lane_width_px
        ):
            measured_width = float(right_x - left_x)
            self.lane_width_est = 0.8 * self.lane_width_est + 0.2 * measured_width
            lane_center_x = 0.5 * (left_x + right_x)
        elif left_x is not None:
            lane_center_x = float(left_x) + self.lane_width_est * 0.5
            speed_scale = single_side_scale
        elif right_x is not None:
            lane_center_x = float(right_x) - self.lane_width_est * 0.5
            speed_scale = single_side_scale

        cmd = Twist()
        image_center_x = proc_width / 2.0

        if lane_center_x is not None:
            error_norm = (lane_center_x - image_center_x) / image_center_x
            angular_z = _clamp(
                -angular_gain * error_norm,
                -max_angular_z,
                max_angular_z,
            )
            linear_x = linear_speed * max(min_linear_scale, 1.0 - abs(error_norm))
            linear_x *= speed_scale

            cmd.linear.x = float(linear_x)
            cmd.angular.z = float(angular_z)

        debug = frame.copy()
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        debug[roi_y:, :] = cv2.addWeighted(roi, 0.6, mask_bgr, 0.4, 0.0)

        cv2.line(debug, (half, 0), (half, proc_height), (0, 255, 0), 2)
        cv2.line(debug, (0, roi_y), (proc_width, roi_y), (255, 255, 0), 2)

        if left_x is not None:
            cv2.line(debug, (left_x, roi_y), (left_x, proc_height - 1), (255, 0, 0), 2)

        if right_x is not None:
            cv2.line(debug, (right_x, roi_y), (right_x, proc_height - 1), (0, 0, 255), 2)

        if lane_center_x is not None:
            cx = int(round(lane_center_x))
            cv2.line(debug, (cx, roi_y), (cx, proc_height - 1), (0, 255, 255), 2)
            cv2.putText(
                debug,
                f"cmd=({cmd.linear.x:.2f}, {cmd.angular.z:.2f})",
                (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )
        else:
            cv2.putText(
                debug,
                "NO LANE",
                (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
            )

        return cmd, debug

    def destroy_node(self):
        try:
            self._publish_zero()
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LaneKeeperGazeboNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
