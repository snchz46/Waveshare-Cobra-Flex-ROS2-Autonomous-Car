#!/usr/bin/env python3
"""Lane keeping node for the CobraFlex robot."""
import math
import time
import cv2
import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker, MarkerArray


def _gstreamer_pipeline(sensor_id=0, width=1280, height=720, fps=60, flip_method=0):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width=(int){width}, height=(int){height}, "
        f"format=(string)NV12, framerate=(fraction){fps}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width=(int){width}, height=(int){height}, format=(string)BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=(string)BGR ! "
        f"appsink drop=true max-buffers=1 sync=false"
    )


def _nothing(_):
    pass


def _odd(x: int) -> int:
    if x < 1:
        x = 1
    if x % 2 == 0:
        x += 1
    return x


def _clamp(val, lo, hi):
    return max(lo, min(val, hi))


def _steering_text(error_norm, deadband):
    if error_norm is None:
        return "NO_LINE"
    if error_norm > deadband:
        return "RIGHT"
    if error_norm < -deadband:
        return "LEFT"
    return "STRAIGHT"


def _status_color(status):
    if status == "TRACK":
        return 0.10, 0.85, 0.20
    if status == "HOLD":
        return 1.00, 0.80, 0.10
    return 1.00, 0.40, 0.10


def _build_trapezoid_mask(binary_shape, top_y_pct, top_w_pct, bottom_w_pct):
    h, w = binary_shape
    mask = np.zeros((h, w), dtype=np.uint8)

    cx = w // 2
    top_y = int(h * top_y_pct / 100.0)
    top_y = _clamp(top_y, 0, h - 1)

    top_half = int(w * top_w_pct / 200.0)
    bottom_half = int(w * bottom_w_pct / 200.0)

    pts = np.array(
        [
            [_clamp(cx - bottom_half, 0, w - 1), h - 1],
            [_clamp(cx + bottom_half, 0, w - 1), h - 1],
            [_clamp(cx + top_half, 0, w - 1), top_y],
            [_clamp(cx - top_half, 0, w - 1), top_y],
        ],
        dtype=np.int32,
    )

    cv2.fillPoly(mask, [pts], 255)
    return mask, pts


def _threshold_image(roi_bgr, threshold_val, blur_k, morph_k, invert):
    gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (blur_k, blur_k), 0)

    if invert:
        _, binary = cv2.threshold(blur, threshold_val, 255, cv2.THRESH_BINARY_INV)
    else:
        _, binary = cv2.threshold(blur, threshold_val, 255, cv2.THRESH_BINARY)

    kernel = np.ones((morph_k, morph_k), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

    return gray, blur, binary


def _extract_peaks_from_band(band_binary, min_peak, min_gap_px):
    hist = band_binary.sum(axis=0).astype(np.float32) / 255.0

    kernel = np.ones(9, dtype=np.float32) / 9.0
    hist_smooth = np.convolve(hist, kernel, mode="same")

    candidates = []
    for i in range(1, len(hist_smooth) - 1):
        if (
            hist_smooth[i] >= min_peak
            and hist_smooth[i] >= hist_smooth[i - 1]
            and hist_smooth[i] > hist_smooth[i + 1]
        ):
            candidates.append({"x": i, "value": float(hist_smooth[i])})

    candidates.sort(key=lambda p: p["value"], reverse=True)

    selected = []
    for p in candidates:
        if all(abs(p["x"] - s["x"]) >= min_gap_px for s in selected):
            selected.append(p)

    selected = selected[:5]
    selected.sort(key=lambda p: p["x"])
    return selected, hist_smooth


def _choose_lane_center_from_peaks(
    peaks,
    lane_side,
    roi_center_x,
    lane_width_px_est,
):
    """Estimate the lane center from the strongest histogram peaks."""
    if len(peaks) >= 3:
        # Follow either the left or right lane boundary pair depending on the mode.
        if lane_side == 0:
            p_left = peaks[0]["x"]
            p_right = peaks[1]["x"]
        else:
            p_left = peaks[-2]["x"]
            p_right = peaks[-1]["x"]

        lane_center_x = 0.5 * (p_left + p_right)
        lane_width_px = abs(p_right - p_left)
        return lane_center_x, lane_width_px, 1.0

    if len(peaks) == 2:
        p_left = peaks[0]["x"]
        p_right = peaks[1]["x"]
        lane_center_x = 0.5 * (p_left + p_right)
        lane_width_px = abs(p_right - p_left)
        return lane_center_x, lane_width_px, 0.75

    if len(peaks) == 1 and lane_width_px_est is not None:
        p = peaks[0]["x"]

        if lane_side == 0:
            if p >= roi_center_x:
                lane_center_x = p - lane_width_px_est / 2.0
            else:
                lane_center_x = p + lane_width_px_est / 2.0
        else:
            if p <= roi_center_x:
                lane_center_x = p + lane_width_px_est / 2.0
            else:
                lane_center_x = p - lane_width_px_est / 2.0

        return lane_center_x, lane_width_px_est, 0.35

    return None, None, 0.0


class LaneKeeperNode(Node):
    """Follow a lane from the CSI camera and publish debug ROS topics."""

    def __init__(self):
        """Initialize publishers, parameters, state and camera capture."""
        super().__init__("lane_keeper")

        # ===== Parameters =====
        self.declare_parameter("sensor_id", 0)
        self.declare_parameter("capture_width", 1280)
        self.declare_parameter("capture_height", 720)
        self.declare_parameter("capture_fps", 60)
        self.declare_parameter("flip_method", 0)

        self.declare_parameter("proc_width", 640)
        self.declare_parameter("proc_height", 360)

        self.declare_parameter("timer_hz", 20.0)
        self.declare_parameter("show_debug_windows", True)
        self.declare_parameter("show_control_window", False)
        self.declare_parameter("debug_print_period", 0.5)
        self.declare_parameter("publish_raw_image", True)
        self.declare_parameter("publish_overlay_image", True)
        self.declare_parameter("publish_mask_image", True)
        self.declare_parameter("publish_histogram_image", True)
        self.declare_parameter("publish_camera_info", True)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("camera_frame_id", "camera_link_optical")
        self.declare_parameter("marker_frame_id", "base_footprint")
        self.declare_parameter("camera_hfov_deg", 90.0)

        self.declare_parameter("roi_start_pct", 58)
        self.declare_parameter("threshold_val", 145)
        self.declare_parameter("blur_k", 5)
        self.declare_parameter("morph_k", 5)
        self.declare_parameter("invert", True)

        self.declare_parameter("trap_top_y_pct", 18)
        self.declare_parameter("trap_top_w_pct", 26)
        self.declare_parameter("trap_bottom_w_pct", 82)

        self.declare_parameter("band_h_pct", 10)
        self.declare_parameter("peak_min", 8)
        self.declare_parameter("peak_gap_px", 45)

        self.declare_parameter("lane_width_init_px", 150.0)
        self.declare_parameter("lane_side", 1)   # 0 left, 1 right

        self.declare_parameter("deadband", 0.06)
        self.declare_parameter("ema_alpha", 0.35)
        self.declare_parameter("max_hold_frames", 6)

        self.declare_parameter("linear_speed", 0.12)
        self.declare_parameter("angular_gain", 1.3)
        self.declare_parameter("max_angular_z", 0.8)
        self.declare_parameter("slow_on_turn", True)
        self.declare_parameter("min_linear_scale", 0.45)
        self.declare_parameter("stop_on_lost", True)

        # ===== Publishers =====
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.error_pub = self.create_publisher(Float32, "/lane/error_norm", 10)
        self.status_pub = self.create_publisher(String, "/lane/status", 10)
        self.steer_pub = self.create_publisher(String, "/lane/steer", 10)
        self.confidence_pub = self.create_publisher(Float32, "/lane/confidence", 10)
        self.center_pub = self.create_publisher(Float32, "/lane/lane_center_px", 10)
        self.width_pub = self.create_publisher(Float32, "/lane/lane_width_px", 10)
        self.image_raw_pub = self.create_publisher(Image, "/lane/image_raw", 10)
        self.image_overlay_pub = self.create_publisher(Image, "/lane/image_overlay", 10)
        self.image_mask_pub = self.create_publisher(Image, "/lane/image_mask", 10)
        self.image_hist_pub = self.create_publisher(Image, "/lane/image_histogram", 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, "/lane/camera_info", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/lane/markers", 10)

        # ===== State =====
        self.prev_center_x = None
        self.prev_lane_width_px = None
        self.lost_frames = 0
        self.last_error_norm = None
        self.last_status = "INIT"
        self.last_cmd = Twist()
        self.last_debug_print = 0.0
        self.last_fps_time = time.time()
        self.controls_window_name = "Lane Controls"
        self.controls_ready = False

        # ===== Camera =====
        pipeline = _gstreamer_pipeline(
            sensor_id=self.get_parameter("sensor_id").value,
            width=self.get_parameter("capture_width").value,
            height=self.get_parameter("capture_height").value,
            fps=self.get_parameter("capture_fps").value,
            flip_method=self.get_parameter("flip_method").value,
        )

        self.get_logger().info(f"Using pipeline: {pipeline}")
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            raise RuntimeError("Could not open CSI camera")

        if self.get_parameter("show_control_window").value:
            self._create_control_window()

        timer_period = 1.0 / float(self.get_parameter("timer_hz").value)
        self.timer = self.create_timer(timer_period, self._timer_callback)

        self.get_logger().info("lane_keeper node started")

    def _read_cfg(self):
        cfg = {
            "proc_width": self.get_parameter("proc_width").value,
            "proc_height": self.get_parameter("proc_height").value,
            "show_debug_windows": self.get_parameter("show_debug_windows").value,
            "show_control_window": self.get_parameter("show_control_window").value,
            "debug_print_period": self.get_parameter("debug_print_period").value,
            "publish_raw_image": bool(self.get_parameter("publish_raw_image").value),
            "publish_overlay_image": bool(self.get_parameter("publish_overlay_image").value),
            "publish_mask_image": bool(self.get_parameter("publish_mask_image").value),
            "publish_histogram_image": bool(self.get_parameter("publish_histogram_image").value),
            "publish_camera_info": bool(self.get_parameter("publish_camera_info").value),
            "publish_markers": bool(self.get_parameter("publish_markers").value),
            "camera_frame_id": str(self.get_parameter("camera_frame_id").value),
            "marker_frame_id": str(self.get_parameter("marker_frame_id").value),
            "camera_hfov_deg": float(self.get_parameter("camera_hfov_deg").value),
            "roi_start_pct": self.get_parameter("roi_start_pct").value,
            "threshold_val": self.get_parameter("threshold_val").value,
            "blur_k": _odd(int(self.get_parameter("blur_k").value)),
            "morph_k": _odd(int(self.get_parameter("morph_k").value)),
            "invert": bool(self.get_parameter("invert").value),
            "trap_top_y_pct": self.get_parameter("trap_top_y_pct").value,
            "trap_top_w_pct": self.get_parameter("trap_top_w_pct").value,
            "trap_bottom_w_pct": self.get_parameter("trap_bottom_w_pct").value,
            "band_h_pct": self.get_parameter("band_h_pct").value,
            "peak_min": self.get_parameter("peak_min").value,
            "peak_gap_px": self.get_parameter("peak_gap_px").value,
            "lane_width_init_px": float(self.get_parameter("lane_width_init_px").value),
            "lane_side": int(self.get_parameter("lane_side").value),
            "deadband": float(self.get_parameter("deadband").value),
            "ema_alpha": float(self.get_parameter("ema_alpha").value),
            "max_hold_frames": int(self.get_parameter("max_hold_frames").value),
            "linear_speed": float(self.get_parameter("linear_speed").value),
            "angular_gain": float(self.get_parameter("angular_gain").value),
            "max_angular_z": float(self.get_parameter("max_angular_z").value),
            "slow_on_turn": bool(self.get_parameter("slow_on_turn").value),
            "min_linear_scale": float(self.get_parameter("min_linear_scale").value),
            "stop_on_lost": bool(self.get_parameter("stop_on_lost").value),
        }

        if cfg["show_control_window"] and self.controls_ready:
            cfg.update(self._get_control_window_values())

        return cfg

    def _create_control_window(self):
        try:
            cv2.namedWindow(self.controls_window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.controls_window_name, 560, 560)

            controls = [
                ("ROI start %", int(self.get_parameter("roi_start_pct").value), 90),
                ("Threshold", int(self.get_parameter("threshold_val").value), 255),
                ("Blur", int(self.get_parameter("blur_k").value), 31),
                ("Morph", int(self.get_parameter("morph_k").value), 21),
                ("Invert", 1 if self.get_parameter("invert").value else 0, 1),
                ("Trap top y %", int(self.get_parameter("trap_top_y_pct").value), 80),
                ("Trap top w %", int(self.get_parameter("trap_top_w_pct").value), 100),
                ("Trap bottom w %", int(self.get_parameter("trap_bottom_w_pct").value), 100),
                ("Band h %", int(self.get_parameter("band_h_pct").value), 30),
                ("Peak min", int(self.get_parameter("peak_min").value), 100),
                ("Peak gap px", int(self.get_parameter("peak_gap_px").value), 200),
                ("Lane width init px", int(self.get_parameter("lane_width_init_px").value), 400),
                ("Lane side (0L/1R)", int(self.get_parameter("lane_side").value), 1),
                (
                    "Deadband %",
                    int(round(float(self.get_parameter("deadband").value) * 100.0)),
                    40,
                ),
                ("EMA %", int(round(float(self.get_parameter("ema_alpha").value) * 100.0)), 100),
            ]

            for name, value, max_value in controls:
                cv2.createTrackbar(
                    name,
                    self.controls_window_name,
                    _clamp(int(value), 0, int(max_value)),
                    int(max_value),
                    _nothing,
                )

            self.controls_ready = True
            self.get_logger().info("Lane tuning window enabled")
        except cv2.error as exc:
            self.controls_ready = False
            self.get_logger().warning(
                f"Could not create control window, disabling tuning UI: {exc}"
            )

    def _get_control_window_values(self):
        return {
            "roi_start_pct": cv2.getTrackbarPos("ROI start %", self.controls_window_name),
            "threshold_val": cv2.getTrackbarPos("Threshold", self.controls_window_name),
            "blur_k": _odd(cv2.getTrackbarPos("Blur", self.controls_window_name)),
            "morph_k": _odd(cv2.getTrackbarPos("Morph", self.controls_window_name)),
            "invert": bool(cv2.getTrackbarPos("Invert", self.controls_window_name)),
            "trap_top_y_pct": cv2.getTrackbarPos("Trap top y %", self.controls_window_name),
            "trap_top_w_pct": cv2.getTrackbarPos("Trap top w %", self.controls_window_name),
            "trap_bottom_w_pct": cv2.getTrackbarPos("Trap bottom w %", self.controls_window_name),
            "band_h_pct": cv2.getTrackbarPos("Band h %", self.controls_window_name),
            "peak_min": cv2.getTrackbarPos("Peak min", self.controls_window_name),
            "peak_gap_px": cv2.getTrackbarPos("Peak gap px", self.controls_window_name),
            "lane_width_init_px": float(
                cv2.getTrackbarPos("Lane width init px", self.controls_window_name)
            ),
            "lane_side": cv2.getTrackbarPos("Lane side (0L/1R)", self.controls_window_name),
            "deadband": cv2.getTrackbarPos("Deadband %", self.controls_window_name) / 100.0,
            "ema_alpha": cv2.getTrackbarPos("EMA %", self.controls_window_name) / 100.0,
        }

    def _build_image_msg(self, image, encoding, stamp, frame_id):
        if not image.flags["C_CONTIGUOUS"]:
            image = np.ascontiguousarray(image)

        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = int(image.shape[0])
        msg.width = int(image.shape[1])
        msg.encoding = encoding
        msg.is_bigendian = False
        msg.step = int(image.strides[0])
        msg.data = image.tobytes()
        return msg

    def _build_camera_info_msg(self, width, height, stamp, frame_id, hfov_deg):
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.width = int(width)
        msg.height = int(height)
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Synthetic intrinsics so RViz can render the stream without a calibration file.
        hfov_rad = math.radians(_clamp(float(hfov_deg), 1.0, 179.0))
        fx = width / (2.0 * math.tan(hfov_rad / 2.0))
        fy = fx
        cx = (width - 1) / 2.0
        cy = (height - 1) / 2.0

        msg.k = [
            float(fx), 0.0, float(cx),
            0.0, float(fy), float(cy),
            0.0, 0.0, 1.0,
        ]
        msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        msg.p = [
            float(fx), 0.0, float(cx), 0.0,
            0.0, float(fy), float(cy), 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        return msg

    def _build_marker_array(
        self,
        cfg,
        stamp,
        status,
        steer,
        error_norm,
        cmd,
        confidence,
    ):
        markers = MarkerArray()
        frame_id = cfg["marker_frame_id"]
        color_r, color_g, color_b = _status_color(status)
        lifetime = Duration(seconds=0.4).to_msg()

        arrow = Marker()
        arrow.header.stamp = stamp
        arrow.header.frame_id = frame_id
        arrow.ns = "lane_keeper"
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.lifetime = lifetime
        arrow.scale.x = 0.06
        arrow.scale.y = 0.11
        arrow.scale.z = 0.14
        arrow.color.r = color_r
        arrow.color.g = color_g
        arrow.color.b = color_b
        arrow.color.a = 0.95

        start = Point(x=0.15, y=0.0, z=0.20)
        linear_norm = 0.0
        if cfg["linear_speed"] > 1e-6:
            linear_norm = _clamp(cmd.linear.x / cfg["linear_speed"], 0.0, 1.2)

        turn_norm = 0.0
        if cfg["max_angular_z"] > 1e-6:
            turn_norm = _clamp(cmd.angular.z / cfg["max_angular_z"], -1.0, 1.0)

        end = Point()
        end.x = 0.30 + 0.35 * linear_norm
        end.y = 0.30 * turn_norm
        end.z = 0.20
        arrow.points = [start, end]
        markers.markers.append(arrow)

        text = Marker()
        text.header.stamp = stamp
        text.header.frame_id = frame_id
        text.ns = "lane_keeper"
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.lifetime = lifetime
        text.scale.z = 0.16
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 0.95
        text.pose.position.x = 0.0
        text.pose.position.y = -0.35
        text.pose.position.z = 0.45
        err_str = "n/a" if error_norm is None else f"{error_norm:+.3f}"
        text.text = (
            f"{status} | {steer}\n"
            f"conf={confidence:.2f} err={err_str}\n"
            f"cmd=({cmd.linear.x:.2f}, {cmd.angular.z:.2f})"
        )
        markers.markers.append(text)

        return markers

    def _publish_visual_topics(self, cfg, debug, overlay, masked_vis, stamp):
        frame_id = cfg["camera_frame_id"]
        frame = debug["frame"]
        hist_image = debug["debug_hist_image"]
        height, width = frame.shape[:2]

        if cfg["publish_raw_image"]:
            self.image_raw_pub.publish(
                self._build_image_msg(frame, "bgr8", stamp, frame_id)
            )

        if cfg["publish_overlay_image"]:
            self.image_overlay_pub.publish(
                self._build_image_msg(overlay, "bgr8", stamp, frame_id)
            )

        if cfg["publish_mask_image"]:
            self.image_mask_pub.publish(
                self._build_image_msg(masked_vis, "bgr8", stamp, frame_id)
            )

        if cfg["publish_histogram_image"]:
            self.image_hist_pub.publish(
                self._build_image_msg(hist_image, "bgr8", stamp, frame_id)
            )

        if cfg["publish_camera_info"]:
            self.camera_info_pub.publish(
                self._build_camera_info_msg(
                    width=width,
                    height=height,
                    stamp=stamp,
                    frame_id=frame_id,
                    hfov_deg=cfg["camera_hfov_deg"],
                )
            )

    def _process_frame(self, frame_bgr, cfg):
        frame = cv2.resize(
            frame_bgr,
            (cfg["proc_width"], cfg["proc_height"]),
            interpolation=cv2.INTER_AREA
        )

        h, w = frame.shape[:2]
        roi_y = int(h * cfg["roi_start_pct"] / 100.0)
        roi_y = _clamp(roi_y, 0, h - 1)
        roi = frame[roi_y:, :]

        _, _, binary = _threshold_image(
            roi,
            cfg["threshold_val"],
            cfg["blur_k"],
            cfg["morph_k"],
            cfg["invert"],
        )

        trap_mask, trap_pts = _build_trapezoid_mask(
            binary.shape,
            cfg["trap_top_y_pct"],
            cfg["trap_top_w_pct"],
            cfg["trap_bottom_w_pct"],
        )

        masked = cv2.bitwise_and(binary, trap_mask)

        overlay = frame.copy()
        masked_vis = cv2.cvtColor(masked, cv2.COLOR_GRAY2BGR)

        roi_h, roi_w = masked.shape
        roi_center_x = roi_w / 2.0

        cv2.line(overlay, (w // 2, 0), (w // 2, h), (0, 255, 0), 2)
        cv2.line(overlay, (0, roi_y), (w, roi_y), (255, 255, 0), 2)

        trap_pts_frame = trap_pts.copy()
        trap_pts_frame[:, 1] += roi_y
        cv2.polylines(overlay, [trap_pts_frame], True, (255, 0, 255), 2)
        cv2.polylines(masked_vis, [trap_pts], True, (255, 0, 255), 2)
        cv2.line(masked_vis, (int(roi_center_x), 0), (int(roi_center_x), roi_h), (0, 255, 0), 2)

        band_h = int(roi_h * cfg["band_h_pct"] / 100.0)
        band_h = max(band_h, 6)

        band_centers_pct = [82, 68, 54]
        band_weights = [0.50, 0.30, 0.20]

        if self.prev_lane_width_px is not None:
            lane_width_guess = self.prev_lane_width_px
        else:
            lane_width_guess = cfg["lane_width_init_px"]

        band_estimates = []
        lane_width_measurements = []
        debug_hist_image = np.zeros((roi_h, roi_w, 3), dtype=np.uint8)

        for band_center_pct, band_weight in zip(band_centers_pct, band_weights):
            band_center_y = int(roi_h * band_center_pct / 100.0)
            y1 = _clamp(band_center_y - band_h // 2, 0, roi_h - 1)
            y2 = _clamp(band_center_y + band_h // 2, y1 + 1, roi_h)

            band = masked[y1:y2, :]

            peaks, hist_smooth = _extract_peaks_from_band(
                band,
                cfg["peak_min"],
                cfg["peak_gap_px"],
            )

            lane_center_x, lane_width_px, confidence = _choose_lane_center_from_peaks(
                peaks,
                cfg["lane_side"],
                roi_center_x,
                lane_width_guess,
            )

            cv2.rectangle(masked_vis, (0, y1), (roi_w - 1, y2), (0, 255, 255), 1)
            cv2.rectangle(overlay, (0, roi_y + y1), (w - 1, roi_y + y2), (0, 255, 255), 1)

            for p in peaks:
                px = int(p["x"])
                py = (y1 + y2) // 2
                cv2.circle(masked_vis, (px, py), 4, (255, 0, 0), -1)
                cv2.circle(overlay, (px, roi_y + py), 4, (255, 0, 0), -1)

            if lane_center_x is not None:
                cx = int(round(lane_center_x))
                cy = (y1 + y2) // 2

                cv2.circle(masked_vis, (cx, cy), 5, (0, 0, 255), -1)
                cv2.line(masked_vis, (int(roi_center_x), cy), (cx, cy), (0, 255, 255), 2)

                cv2.circle(overlay, (cx, roi_y + cy), 5, (0, 0, 255), -1)
                cv2.line(overlay, (w // 2, roi_y + cy), (cx, roi_y + cy), (0, 255, 255), 2)

                band_estimates.append(
                    {"center_x": lane_center_x, "weight": band_weight * confidence}
                )

                if lane_width_px is not None and lane_width_px > 20:
                    lane_width_measurements.append((lane_width_px, confidence))

            hist_norm = hist_smooth.copy()
            if hist_norm.max() > 0:
                hist_norm = hist_norm / hist_norm.max()

            for x in range(roi_w):
                hval = int(hist_norm[x] * (band_h - 1))
                y_top = y2 - 1 - hval
                debug_hist_image[y_top:y2, x] = (80, 80, 80)

        raw_lane_center_x = None
        new_lane_width_px = self.prev_lane_width_px

        if band_estimates:
            total_w = sum(b["weight"] for b in band_estimates)
            raw_lane_center_x = (
                sum(b["center_x"] * b["weight"] for b in band_estimates)
                / max(total_w, 1e-6)
            )

        if lane_width_measurements:
            total_w = sum(conf for _, conf in lane_width_measurements)
            raw_lane_width = (
                sum(width * conf for width, conf in lane_width_measurements)
                / max(total_w, 1e-6)
            )

            if self.prev_lane_width_px is None:
                new_lane_width_px = raw_lane_width
            else:
                new_lane_width_px = 0.25 * raw_lane_width + 0.75 * self.prev_lane_width_px

        track_confidence = _clamp(sum(b["weight"] for b in band_estimates), 0.0, 1.0)

        return {
            "frame": frame,
            "overlay": overlay,
            "masked_vis": masked_vis,
            "debug_hist_image": debug_hist_image,
            "raw_lane_center_x": raw_lane_center_x,
            "lane_width_px": new_lane_width_px,
            "image_center_x": w / 2.0,
            "track_confidence": track_confidence,
        }

    def _timer_callback(self):
        cfg = self._read_cfg()

        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warning("Failed to read frame from CSI camera")
            return

        debug = self._process_frame(frame, cfg)
        stamp = self.get_clock().now().to_msg()

        overlay = debug["overlay"]
        masked_vis = debug["masked_vis"]
        hist_vis = debug["debug_hist_image"]
        raw_lane_center_x = debug["raw_lane_center_x"]
        image_center_x = debug["image_center_x"]
        track_confidence = debug["track_confidence"]

        if debug["lane_width_px"] is not None:
            self.prev_lane_width_px = debug["lane_width_px"]

        status = "TRACK"

        if raw_lane_center_x is not None:
            self.lost_frames = 0
            if self.prev_center_x is None:
                self.prev_center_x = raw_lane_center_x
            else:
                a = cfg["ema_alpha"]
                self.prev_center_x = a * raw_lane_center_x + (1.0 - a) * self.prev_center_x
        else:
            self.lost_frames += 1
            if self.prev_center_x is not None and self.lost_frames <= cfg["max_hold_frames"]:
                status = "HOLD"
            else:
                status = "NO_LINE"
                self.prev_center_x = None

        error_px = None
        error_norm = None

        if self.prev_center_x is not None:
            error_px = int(round(self.prev_center_x - image_center_x))
            error_norm = float(error_px) / (overlay.shape[1] / 2.0)
            self.last_error_norm = error_norm

            cy = int(overlay.shape[0] * 0.82)
            cx = int(round(self.prev_center_x))
            cv2.circle(overlay, (cx, cy), 8, (0, 0, 255), -1)
            cv2.line(overlay, (overlay.shape[1] // 2, cy), (cx, cy), (0, 255, 255), 2)

        steer = _steering_text(error_norm, cfg["deadband"])

        if status == "HOLD" and raw_lane_center_x is None:
            hold_ratio = 1.0 - (self.lost_frames / max(cfg["max_hold_frames"] + 1, 1))
            track_confidence = max(0.05, 0.4 * hold_ratio)
        elif status == "NO_LINE":
            track_confidence = 0.0

        cmd = Twist()

        if error_norm is not None:
            angular_z = -cfg["angular_gain"] * error_norm
            angular_z = _clamp(
                angular_z,
                -cfg["max_angular_z"],
                cfg["max_angular_z"],
            )

            linear_x = cfg["linear_speed"]
            if cfg["slow_on_turn"]:
                scale = max(cfg["min_linear_scale"], 1.0 - abs(error_norm))
                linear_x *= scale

            if status == "HOLD":
                linear_x *= 0.6

            cmd.linear.x = float(linear_x)
            cmd.angular.z = float(angular_z)
        else:
            if cfg["stop_on_lost"]:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)
        self.last_cmd = cmd
        self.last_status = status

        msg_err = Float32()
        msg_err.data = float(error_norm) if error_norm is not None else float("nan")
        self.error_pub.publish(msg_err)

        msg_status = String()
        msg_status.data = status
        self.status_pub.publish(msg_status)

        msg_steer = String()
        msg_steer.data = steer
        self.steer_pub.publish(msg_steer)

        msg_conf = Float32()
        msg_conf.data = float(track_confidence)
        self.confidence_pub.publish(msg_conf)

        msg_center = Float32()
        if self.prev_center_x is not None:
            msg_center.data = float(self.prev_center_x)
        else:
            msg_center.data = float("nan")
        self.center_pub.publish(msg_center)

        msg_width = Float32()
        if self.prev_lane_width_px is not None:
            msg_width.data = float(self.prev_lane_width_px)
        else:
            msg_width.data = float("nan")
        self.width_pub.publish(msg_width)

        # overlay text
        now = time.time()
        fps = 1.0 / max(now - self.last_fps_time, 1e-6)
        self.last_fps_time = now

        lane_name = "LEFT" if cfg["lane_side"] == 0 else "RIGHT"

        cv2.putText(overlay, f"FPS: {fps:.1f}", (18, 34),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 255, 0), 2)
        cv2.putText(overlay, f"Lane: {lane_name}", (18, 66),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 0), 2)
        cv2.putText(overlay, f"Status: {status}", (18, 98),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 165, 255), 2)

        if self.prev_lane_width_px is not None:
            cv2.putText(overlay, f"Lane width px: {self.prev_lane_width_px:.1f}", (18, 130),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.70, (255, 255, 0), 2)

        if error_px is not None:
            cv2.putText(overlay, f"Error px: {error_px}", (18, 162),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.70, (255, 255, 0), 2)

        if error_norm is not None:
            cv2.putText(overlay, f"Error norm: {error_norm:+.3f}", (18, 194),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.70, (255, 255, 0), 2)

        cv2.putText(overlay, f"Confidence: {track_confidence:.2f}", (18, 226),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.70, (255, 255, 0), 2)

        steer_color = (0, 255, 0)
        if steer in ("LEFT", "RIGHT"):
            steer_color = (0, 0, 255)
        elif steer == "NO_LINE":
            steer_color = (0, 165, 255)

        cv2.putText(overlay, f"Steer: {steer}", (18, 258),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.85, steer_color, 2)

        self._publish_visual_topics(cfg, debug, overlay, masked_vis, stamp)

        if cfg["publish_markers"]:
            self.marker_pub.publish(
                self._build_marker_array(
                    cfg=cfg,
                    stamp=stamp,
                    status=status,
                    steer=steer,
                    error_norm=error_norm,
                    cmd=cmd,
                    confidence=track_confidence,
                )
            )

        if cfg["show_debug_windows"]:
            cv2.imshow("Lane Overlay", overlay)
            cv2.imshow("Lane ROI Masked", masked_vis)
            cv2.imshow("Lane Histogram Preview", hist_vis)

        if cfg["show_debug_windows"] or cfg["show_control_window"]:
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                self.get_logger().info("Closing node because q/ESC was pressed")
                self._publish_zero()
                rclpy.shutdown()
                return
            if key == ord("s"):
                stamp_text = time.strftime("%Y%m%d_%H%M%S")
                cv2.imwrite(f"lane_overlay_{stamp_text}.jpg", overlay)
                cv2.imwrite(f"lane_masked_{stamp_text}.jpg", masked_vis)
                cv2.imwrite(f"lane_hist_{stamp_text}.jpg", hist_vis)
                self.get_logger().info("Saved lane debug snapshots")

        # periodic console debug
        if now - self.last_debug_print >= cfg["debug_print_period"]:
            err_str = "None" if error_norm is None else f"{error_norm:+.3f}"
            width_str = (
                f"{self.prev_lane_width_px:.1f}"
                if self.prev_lane_width_px is not None
                else "None"
            )
            self.get_logger().info(
                f"status={status} lane={lane_name} err={err_str} "
                f"cmd=({cmd.linear.x:.3f},{cmd.angular.z:.3f}) "
                f"width_px={width_str} "
                f"lost={self.lost_frames}"
            )
            self.last_debug_print = now

    def _publish_zero(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def destroy_node(self):
        """Stop the robot and release camera resources before shutdown."""
        try:
            self._publish_zero()
        except Exception:
            pass
        try:
            if self.cap is not None:
                self.cap.release()
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    """Run the lane keeper node."""
    rclpy.init(args=args)
    node = None
    try:
        node = LaneKeeperNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
