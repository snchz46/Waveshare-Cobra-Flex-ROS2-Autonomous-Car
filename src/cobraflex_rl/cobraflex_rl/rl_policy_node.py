"""
rl_policy_node — the trained RL camera policy as a ROS2 inference node (Phase 5).

The distributed track-'E' loop (mirrors the F2 physical demo, now camera-driven):

    camera/image_raw_lane ─► rl_policy_node ──────────────► /raw_action (Twist)
    camera/image_raw_lane ─► cv_lane_estimator_node ─────► /state_obs + /perception_invalid
    /raw_action + /state_obs + /perception_invalid ─► cage_ros_node ─► /safe_action + /cage_status
    /safe_action ─► vehicle_control_node ─► /cmd_vel ─► (real motor driver)
    /cage_status ─► cage_logger_node ─► CSV

This node is the ONE piece the sim did in-process (inside ``GazeboLaneEnv.step``);
on hardware the policy must run as its own node. It reuses the EXACT preprocessing
the policy trained on — ``camera_pipeline.decode_image`` + ``to_observation`` (84×84
grayscale) + a k=4 frame stack (the ``VecFrameStack`` equivalent, identical to
``eval_policy._FrameStacker``) — so the CNN sees bit-identical observations to
Gazebo. Inference is ``model.predict(obs, deterministic=True)``; the 2-D action
[steer, throttle] ∈ [-1, 1]² is published as a Twist with the convention
``cage_ros_node`` expects (``angular.z``=steering, ``linear.x``=throttle), so the
cage arbitrates the raw command exactly as in simulation.

THE TWO CONTRACTS THIS NODE MUST NOT BREAK (both were wrong before 2026-08-05):

1. **Throttle domain.** ``/raw_action.linear.x`` is the cage's normalised throttle
   u ∈ [0, 1], NOT the policy's raw symmetric action a ∈ [-1, 1]. In simulation
   ``GazeboLaneEnv.step`` applies ``cage_bridge.policy_throttle_to_cage``
   (u = (a+1)/2) *before* the cage sees the action, so publishing ``a`` here fed
   C-04/C-06 — and ``vehicle_control_node`` — a number in the wrong domain: a = 0
   (the middle of the policy's range, u = 0.5 → 0.11 m/s in sim) would read as
   throttle 0, i.e. a full stop. The mapping is imported, not re-implemented, so
   the two paths cannot drift.
2. **Control rate.** The policy and the cage step at ``control_rate_hz`` (10 Hz =
   the training ``control_dt`` 0.1 s), NOT at the camera's 20 Hz. Inference used to
   run in the image callback, which doubled the loop rate on hardware and with it
   (i) C-06's effective steering slew authority — ``delta_max_steering_per_cycle``
   is per *cage cycle*, and the cage cycles on ``/raw_action`` — and (ii) the
   k=4 frame stack's time horizon (0.2 s instead of the trained 0.4 s). The timer
   consumes the latest buffered frame, exactly as ``GazeboLaneEnv.step`` does.

Parameters
    checkpoint        (str)  SB3 .zip to load (the verdict/deploy checkpoint).
    algorithm         (str)  'ppo' (default, the deployed trunk) | 'sac' — the class
                             the checkpoint was trained with.
    image_topic       (str)  camera frames the policy saw (default camera/image_raw_lane).
    raw_action_topic  (str)  where the raw policy action is published (default /raw_action).
    frame_stack       (int)  k (default 4, the E-design value).
    grayscale         (bool) default True (the E-design observation).
    device            (str)  'cpu' (default; CNN inference at 10 Hz is ample on CPU).
    control_rate_hz   (float) 10.0 — the trained control rate (1/control_dt). 0 =
                             legacy per-frame inference (camera rate); do not use
                             on hardware, see contract 2 above.
    throttle_map      (str)  'policy_2d' (default) applies policy_throttle_to_cage;
                             'raw' publishes the action unmapped (only for a
                             checkpoint already trained in cage throttle scale).
    throttle_nominal  (float) 0.5 — the throttle published for a steering-only
                             (1-D) checkpoint, matching the fixed cruise nominal
                             ``GazeboLaneEnv._apply_cage`` substitutes there.

**STATUS: Phase-5 deployment scaffolding — NOT yet run on the physical CobraFlex.**
The preprocessing + action-mapping logic is unit-tested host-side
(``policy/tests/test_rl_policy_node.py``); the ROS wiring is validated only by the
sim components it reuses. First hardware bring-up must follow
``docs/17_physical_deployment.md`` (camera extrinsics, safety pre-checks, e-stop).
"""
from __future__ import annotations

from typing import List, Optional

import numpy as np

from .cage_bridge import policy_throttle_to_cage
from .camera_pipeline import decode_image, to_observation


def action_to_twist_fields(
    action,
    *,
    throttle_map: str = "policy_2d",
    throttle_nominal: float = 0.5,
) -> tuple:
    """Map a policy action to the (angular_z, linear_x) Twist fields
    ``cage_ros_node`` reads.

    ``angular.z`` is the steering command in [-1, 1]. ``linear.x`` is the cage's
    normalised throttle u ∈ [0, 1] — under the default ``policy_2d`` map that is
    ``policy_throttle_to_cage(a[1])``, the identical transform
    ``GazeboLaneEnv.step`` applies before the cage on the 2-D path (D-50). A
    steering-only (1-D) checkpoint has no throttle axis, so it publishes
    ``throttle_nominal``, the fixed cruise nominal ``_apply_cage`` substitutes on
    the frozen 1-D path. ``throttle_map='raw'`` restores the unmapped passthrough.
    """
    a = np.asarray(action, dtype=float).reshape(-1)
    steer = float(np.clip(a[0], -1.0, 1.0))
    if a.size < 2:
        return steer, float(throttle_nominal)
    if throttle_map == "raw":
        return steer, float(np.clip(a[1], -1.0, 1.0))
    if throttle_map != "policy_2d":
        raise ValueError(
            f"throttle_map must be 'policy_2d' or 'raw', got {throttle_map!r}"
        )
    return steer, float(policy_throttle_to_cage(a[1]))  # (angular.z, linear.x)


class _FrameStacker:
    """VecFrameStack equivalent (identical to eval_policy._FrameStacker): last k
    frames concatenated on the channel axis, newest last."""

    def __init__(self, k: int) -> None:
        self.k = int(k)
        self._frames: List[np.ndarray] = []

    def reset(self, frame: np.ndarray) -> np.ndarray:
        self._frames = [frame] * self.k
        return np.concatenate(self._frames, axis=-1)

    def step(self, frame: np.ndarray) -> np.ndarray:
        self._frames = (self._frames + [frame])[-self.k:]
        return np.concatenate(self._frames, axis=-1)


def main(argv: Optional[List[str]] = None) -> None:
    # ROS + SB3 imported inside main so the pure helpers above stay host-testable
    # without rclpy / stable_baselines3 installed.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Image
    from stable_baselines3 import PPO, SAC

    class RlPolicyNode(Node):
        def __init__(self) -> None:
            super().__init__("rl_policy_node")
            self.declare_parameter("checkpoint", "")
            # ppo: the deployed trunk is the 2-D PPO cap-0.22 550k checkpoint
            # (D-66/D-67); the SAC 2-D arms are findings, not the verdict of record.
            self.declare_parameter("algorithm", "ppo")
            self.declare_parameter("image_topic", "camera/image_raw_lane")
            self.declare_parameter("raw_action_topic", "/raw_action")
            self.declare_parameter("frame_stack", 4)
            self.declare_parameter("grayscale", True)
            self.declare_parameter("device", "cpu")
            # The trained control rate (config control_dt 0.1 s). See contract 2.
            self.declare_parameter("control_rate_hz", 10.0)
            self.declare_parameter("throttle_map", "policy_2d")
            self.declare_parameter("throttle_nominal", 0.5)

            ckpt = str(self.get_parameter("checkpoint").value)
            if not ckpt:
                raise RuntimeError("rl_policy_node: 'checkpoint' parameter is required")
            algo = str(self.get_parameter("algorithm").value).lower()
            cls = {"sac": SAC, "ppo": PPO}.get(algo)
            if cls is None:
                raise RuntimeError(f"rl_policy_node: unknown algorithm {algo!r}")
            self._grayscale = bool(self.get_parameter("grayscale").value)
            self._stacker = _FrameStacker(int(self.get_parameter("frame_stack").value))
            self._first = True
            self._throttle_map = str(self.get_parameter("throttle_map").value)
            self._throttle_nominal = float(self.get_parameter("throttle_nominal").value)
            if self._throttle_map not in ("policy_2d", "raw"):
                raise RuntimeError(
                    f"rl_policy_node: throttle_map must be 'policy_2d' or 'raw', "
                    f"got {self._throttle_map!r}"
                )

            self.get_logger().info(f"Loading {algo.upper()} checkpoint: {ckpt}")
            self._model = cls.load(ckpt, device=str(self.get_parameter("device").value))

            self._pub = self.create_publisher(
                Twist, str(self.get_parameter("raw_action_topic").value), 10
            )
            self.create_subscription(
                Image, str(self.get_parameter("image_topic").value),
                self._on_image, qos_profile_sensor_data,
            )

            # Latest frame only, consumed by the control timer — the hardware
            # equivalent of GazeboLaneEnv.step reading the newest camera sample
            # once per control_dt. Buffering (rather than inferring per frame)
            # is what keeps the cage cycling at the trained rate; see contract 2.
            self._latest_image: Optional[Image] = None
            self._starved = 0
            rate = float(self.get_parameter("control_rate_hz").value)
            if rate > 0.0:
                self.create_timer(1.0 / rate, self._on_control_tick)
                self.get_logger().info(
                    f"rl_policy_node ready: control loop {rate} Hz, "
                    f"throttle_map={self._throttle_map} (Phase-5 scaffolding)."
                )
            else:
                # Legacy per-frame inference. Kept for bench replay against a
                # recorded topic; on hardware it runs the cage at camera rate.
                self.get_logger().warn(
                    "control_rate_hz=0 → inferring once per camera frame. The cage "
                    "then cycles at the camera rate, not the trained 10 Hz "
                    "(C-06's per-cycle slew budget is applied twice as often)."
                )

        def _on_image(self, msg: Image) -> None:
            if float(self.get_parameter("control_rate_hz").value) > 0.0:
                self._latest_image = msg
                return
            self._infer(msg)

        def _on_control_tick(self) -> None:
            msg = self._latest_image
            if msg is None:
                # No frame this cycle: publish nothing. /raw_action is the cage's
                # cycle trigger, so the silence propagates to vehicle_control's
                # timeout and stops the car (csi_camera_node's fail-safe chain).
                self._starved += 1
                if self._starved % 10 == 1:
                    self.get_logger().warn(
                        f"no camera frame for {self._starved} control cycles — "
                        "publishing no /raw_action"
                    )
                return
            self._starved = 0
            self._latest_image = None
            self._infer(msg)

        def _infer(self, msg: Image) -> None:
            try:
                frame_bgr = decode_image(msg.data, msg.height, msg.width,
                                         msg.encoding, msg.step)
            except ValueError as exc:  # a malformed frame must not crash the loop
                self.get_logger().warn(f"dropped frame: {exc}")
                return
            obs_frame = to_observation(frame_bgr, grayscale=self._grayscale)
            obs = (self._stacker.reset(obs_frame) if self._first
                   else self._stacker.step(obs_frame))
            self._first = False
            action, _ = self._model.predict(obs, deterministic=True)
            steer, throttle = action_to_twist_fields(
                action,
                throttle_map=self._throttle_map,
                throttle_nominal=self._throttle_nominal,
            )
            twist = Twist()
            twist.angular.z = steer
            twist.linear.x = throttle
            self._pub.publish(twist)

    rclpy.init(args=argv)
    node = None
    try:
        node = RlPolicyNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
