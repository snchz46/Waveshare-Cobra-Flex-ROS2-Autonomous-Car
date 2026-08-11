"""ROS2/Gazebo I/O layer for the RL environment.

:class:`RosGazeboInterface` is the only place the env touches ROS2 or the gz
CLI: it subscribes to ground-truth odometry (and optionally the front camera),
publishes ``/cmd_vel``, paces the control cycle on *simulation* time, and
drives Gazebo services (teleport, physics RTF) via ``gz service`` subprocesses.
Everything above it (:class:`~cobraflex_rl.gazebo_lane_env.GazeboLaneEnv`) is
transport-agnostic.
"""

from __future__ import annotations

import math
import re
import subprocess
import time
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from .camera_pipeline import decode_image


class RosGazeboInterface(Node):
    """ROS2 node bridging the RL env to a running Gazebo world.

    Pose source is the OdometryPublisher ground truth (``/odom_truth`` by
    default), corrected by a per-episode constant odom→world offset set via
    :meth:`calibrate_pose_offset`. ``camera_topic=""`` (F-track) skips the
    image subscription entirely.
    """

    def __init__(
        self,
        world_name: str = "lane_following_oval",
        model_name: str = "cobraflex_robot",
        spawn_z: float = 0.05,
        service_timeout_ms: int = 3500,
        odom_topic: str = "/odom_truth",
        camera_topic: str = "",
    ) -> None:
        super().__init__("cobraflex_rl_interface")
        self._odom_msg: Optional[Odometry] = None
        # Track 'E' (D-41/D-43): optional camera subscription. Empty topic =
        # F-track behaviour, no image traffic. The latest raw msg is kept and
        # decoded lazily in get_camera_frame() so 20 Hz × 0.9 MB frames cost
        # nothing when the consumer only samples at the 10 Hz control rate.
        self._image_msg: Optional[Image] = None
        self.camera_topic = str(camera_topic)
        if self.camera_topic:
            self._image_sub = self.create_subscription(
                Image,
                self.camera_topic,
                self._image_callback,
                qos_profile_sensor_data,
            )
        self.world_name = world_name
        self.model_name = model_name
        self.spawn_z = float(spawn_z)
        self.service_timeout_ms = int(service_timeout_ms)
        self.odom_topic = str(odom_topic)
        self._model_entity_id: Optional[int] = None
        # We subscribe to the OdometryPublisher ground-truth pose (/odom_truth),
        # NOT the DiffDrive encoder /odom. Both plugins used to publish /odom, so
        # the RL subscriber saw interleaved truth + dead-reckoning samples; after
        # a set_pose teleport the dead-reckoning half does not jump, which
        # corrupted the tracked pose on every episode reset. Ground truth
        # reflects teleports immediately, so the per-episode offset below only
        # removes a constant odom->world frame translation.
        self._odom_dx: float = 0.0
        self._odom_dy: float = 0.0
        self._odom_dyaw: float = 0.0

        self._odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self._odom_callback,
            10,
        )
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def _odom_callback(self, msg: Odometry) -> None:
        self._odom_msg = msg

    def _image_callback(self, msg: Image) -> None:
        self._image_msg = msg

    def get_camera_frame(self):
        """Latest camera frame as ``(bgr_array, stamp_s)`` or ``None``.

        Decodes the most recent retained message on demand. Returns None when
        no frame has arrived yet (or no camera topic configured).
        """
        msg = self._image_msg
        if msg is None:
            return None
        frame = decode_image(
            msg.data, int(msg.height), int(msg.width), msg.encoding, int(msg.step)
        )
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        return frame, stamp

    def clear_camera_frame(self) -> None:
        """Drop the retained frame (episode reset: don't reuse a pre-teleport view)."""
        self._image_msg = None

    @staticmethod
    def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
        """Yaw (rad) from a quaternion (planar robot: roll/pitch ignored)."""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def wait_for_initial_data(self, timeout_sec: float = 10.0) -> bool:
        """Block until the first odometry message arrives (True) or timeout (False)."""
        deadline = time.monotonic() + float(timeout_sec)
        while rclpy.ok() and self._odom_msg is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._odom_msg is not None

    def send_action(self, steer: float, speed: float) -> None:
        """Publish a Twist command: ``angular.z = steer``, ``linear.x = speed``."""
        msg = Twist()
        msg.linear.x = float(speed)
        msg.angular.z = float(steer)
        self._cmd_pub.publish(msg)

    def sim_now(self) -> Optional[float]:
        """Public alias for the current sim time (latest odom stamp), the
        shared time base for camera-frame freshness in the E-track pipeline."""
        return self._odom_sim_time()

    def _odom_sim_time(self) -> Optional[float]:
        """Latest odom header stamp in seconds (simulation time), or None.

        The OdometryPublisher stamps each message with gz sim time, so this is a
        valid sim clock regardless of whether this node has use_sim_time set."""
        if self._odom_msg is None:
            return None
        stamp = self._odom_msg.header.stamp
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _drain_odom(self) -> None:
        """Process all immediately-available odom messages, leaving _odom_msg at
        the most recent buffered sample.

        gz service subprocesses (set_pose) block the executor for ~0.25 s, during
        which ~12 messages accumulate at 50 Hz. Without draining, sim-time waits
        would "advance" instantly by consuming those stale buffered stamps and
        return before the live sim has actually progressed — which calibrated the
        spawn pose against a pre-teleport sample (impossible step-1 ey)."""
        prev = self._odom_sim_time()
        for _ in range(100):
            rclpy.spin_once(self, timeout_sec=0.0)
            cur = self._odom_sim_time()
            if cur == prev:
                break
            prev = cur

    def step_ros(self, duration: float) -> None:
        """Advance the control cycle by ``duration`` seconds of *simulation* time.

        Waiting on sim time (from odom header stamps) instead of wall-clock keeps
        the control cadence correct when Gazebo runs faster than real time, so the
        per-step wall cost collapses to however fast the sim can produce the next
        ``duration`` of sim time. Falls back to wall-clock pacing when no sim-time
        reference is available or when the sim clock is stalled, so the worst case
        is the old real-time behaviour (no regression)."""
        duration = max(0.0, float(duration))
        if duration == 0.0:
            rclpy.spin_once(self, timeout_sec=0.0)
            return

        # Baseline must be the latest sample, not the head of a backlog, or the
        # wait below returns instantly by consuming stale buffered stamps.
        self._drain_odom()
        start_sim = self._odom_sim_time()
        start_wall = time.monotonic()
        if start_sim is None:
            self._spin_wall(start_wall + duration)
            return

        hard_deadline = start_wall + max(2.0, duration * 50.0)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            now_sim = self._odom_sim_time()
            sim_elapsed = (now_sim - start_sim) if now_sim is not None else 0.0
            if sim_elapsed >= duration:
                return
            wall_elapsed = time.monotonic() - start_wall
            # Stall guard: if the sim clock isn't advancing at all yet we have
            # already spent the nominal duration in wall time, treat the clock as
            # effectively wall-paced (or stalled) and stop — never hang.
            if sim_elapsed <= 1e-6 and wall_elapsed >= duration:
                return
            if wall_elapsed >= hard_deadline - start_wall:
                return

    def _spin_wall(self, end_time: float) -> None:
        while rclpy.ok() and time.monotonic() < end_time:
            remaining = max(0.0, end_time - time.monotonic())
            rclpy.spin_once(self, timeout_sec=min(0.05, remaining))

    def spin_wall(self, duration: float) -> None:
        """Spin for ``duration`` *real wall* seconds, continuously processing odom
        so ``_odom_msg`` ends at the latest live sample.

        Used where genuine elapsed time is required — chiefly letting a set_pose
        teleport propagate through the gz server before calibrating. Unlike the
        sim-time ``step_ros``, this cannot be fooled into returning early by a
        backlog of buffered odom stamps, because it gates on wall time."""
        self._spin_wall(time.monotonic() + max(0.0, float(duration)))

    def set_real_time_factor(
        self, real_time_factor: float, max_step_size: float = 0.001
    ) -> bool:
        """Bump the live real-time-factor cap via /world/<name>/set_physics.

        Lets headless training run faster than real time without touching the
        (generated, hash-tracked) .world file or changing physics fidelity —
        ``max_step_size`` is held at the world's value so the integrator is
        unchanged. ``real_time_factor`` of 0 means "as fast as possible"; if a
        given gz build ignores 0, pass a high finite value instead. Best-effort:
        logs a warning and returns False on failure (e.g. service unavailable)."""
        request = (
            f"max_step_size: {float(max_step_size):.9f} "
            f"real_time_factor: {float(real_time_factor):.6f}"
        )
        return self._call_gz_service(
            service=f"/world/{self.world_name}/set_physics",
            request_type="gz.msgs.Physics",
            response_type="gz.msgs.Boolean",
            request=request,
        )

    def _get_raw_pose(self) -> Tuple[float, float, float]:
        """Uncorrected ``(x, y, yaw)`` straight from the latest odom message."""
        if self._odom_msg is None:
            raise RuntimeError("No odometry data received yet.")
        pose = self._odom_msg.pose.pose
        orientation = pose.orientation
        yaw = self.quaternion_to_yaw(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        return float(pose.position.x), float(pose.position.y), float(yaw)

    def get_pose(self) -> Tuple[float, float, float]:
        """World-frame ``(x, y, yaw)``: raw odom plus the calibrated offset."""
        raw_x, raw_y, raw_yaw = self._get_raw_pose()
        corrected_yaw = math.atan2(
            math.sin(raw_yaw + self._odom_dyaw),
            math.cos(raw_yaw + self._odom_dyaw),
        )
        return raw_x + self._odom_dx, raw_y + self._odom_dy, corrected_yaw

    def calibrate_pose_offset(self, true_x: float, true_y: float, true_yaw: float) -> None:
        """Compute the additive offset that maps current raw odom → known world pose.

        Call once after wait_for_initial_data + step_ros following each teleport.
        With the ground-truth /odom_truth source this offset is just the constant
        odom->world frame translation (the OdometryPublisher already reflects the
        set_pose teleport), so get_pose() returns world coordinates thereafter.
        """
        raw_x, raw_y, raw_yaw = self._get_raw_pose()
        self._odom_dx = float(true_x) - raw_x
        self._odom_dy = float(true_y) - raw_y
        self._odom_dyaw = math.atan2(
            math.sin(float(true_yaw) - raw_yaw),
            math.cos(float(true_yaw) - raw_yaw),
        )

    def get_speed(self) -> float:
        """Magnitude of the latest odom linear velocity (m/s)."""
        if self._odom_msg is None:
            raise RuntimeError("No odometry data received yet.")

        linear = self._odom_msg.twist.twist.linear
        return float(math.sqrt(linear.x**2 + linear.y**2 + linear.z**2))

    def reset_world(self) -> None:
        """Soft reset: just command zero motion (the world itself is not reloaded)."""
        self.send_action(0.0, 0.0)

    def set_vehicle_pose(self, x: float, y: float, yaw: float) -> None:
        """Teleport the model via the gz ``set_pose`` service (up to three retries).

        The gz CLI does service discovery on each call, so under load (e.g. the
        rapid resets of a degenerate short-episode regime) it occasionally times
        out. A failed teleport is *silent* — the episode then trains on a stale
        spawn calibration — so prefer extra retries here over corrupt episodes.
        """
        # Resolve entity ID once and cache it for the rest of the training run
        # to avoid a subprocess timeout on every episode reset.
        if self._model_entity_id is None:
            self._model_entity_id = self._lookup_model_entity_id()
        entity_id = self._model_entity_id or 0

        half_yaw = 0.5 * float(yaw)
        request = (
            f'name: "{self.model_name}" '
            f"id: {entity_id} "
            f"position {{ x: {float(x):.9f} y: {float(y):.9f} z: {self.spawn_z:.9f} }} "
            f"orientation {{ z: {math.sin(half_yaw):.9f} w: {math.cos(half_yaw):.9f} }}"
        )
        # Clear cached odom before attempting teleport so wait_for_initial_data
        # always blocks until fresh post-teleport data arrives, regardless of
        # whether the service call succeeds or fails.
        self._odom_msg = None
        attempts = 4
        for attempt in range(attempts):
            if self._call_gz_service(
                service=f"/world/{self.world_name}/set_pose",
                request_type="gz.msgs.Pose",
                response_type="gz.msgs.Boolean",
                request=request,
            ):
                return
            if attempt < attempts - 1:
                time.sleep(0.5)

    def _call_gz_service(
        self,
        service: str,
        request_type: str,
        response_type: str,
        request: str,
    ) -> bool:
        """Invoke a gz service via the CLI; True iff it answered ``data: true``."""
        cmd = [
            "gz",
            "service",
            "-s",
            service,
            "--reqtype",
            request_type,
            "--reptype",
            response_type,
            "--timeout",
            str(self.service_timeout_ms),
            "--req",
            request,
        ]
        try:
            completed = subprocess.run(
                cmd,
                check=False,
                capture_output=True,
                text=True,
                timeout=(self.service_timeout_ms / 1000.0) + 1.0,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            self._warn_gz_service_failure(service, str(exc))
            return False

        output = f"{completed.stdout}\n{completed.stderr}"
        if completed.returncode == 0 and "data: true" in output.lower():
            return True

        self._warn_gz_service_failure(service, output.strip() or "service returned false")
        return False

    def _warn_gz_service_failure(self, service: str, reason: str) -> None:
        self.get_logger().warning(
            f"Gazebo service call failed for {service}: {reason}"
        )

    def _lookup_model_entity_id(self) -> Optional[int]:
        """Resolve the model's gz entity id from one pose/info sample (cached)."""
        cmd = [
            "gz",
            "topic",
            "-e",
            "-t",
            f"/world/{self.world_name}/pose/info",
            "-n",
            "1",
        ]
        try:
            completed = subprocess.run(
                cmd,
                check=False,
                capture_output=True,
                text=True,
                timeout=2.0,
            )
        except (OSError, subprocess.TimeoutExpired):
            return None

        if completed.returncode != 0:
            return None

        pattern = rf'name:\s+"{re.escape(self.model_name)}"\s+id:\s+(\d+)'
        match = re.search(pattern, completed.stdout)
        if match is None:
            return None

        self._model_entity_id = int(match.group(1))
        return self._model_entity_id
