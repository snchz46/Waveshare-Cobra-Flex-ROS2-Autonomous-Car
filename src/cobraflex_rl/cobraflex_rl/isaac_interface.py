"""In-process Isaac-Sim I/O layer for the RL environment (track 'E' / F-track).

:class:`IsaacSimInterface` is the Isaac-Sim counterpart of
:class:`~cobraflex_rl.ros_interface.RosGazeboInterface`. It exposes the *same*
duck-typed surface that :class:`~cobraflex_rl.gazebo_lane_env.GazeboLaneEnv`
calls (``reset_world`` / ``set_vehicle_pose`` / ``send_action`` / ``step_ros`` /
``get_pose`` / ``get_speed`` / ``get_camera_frame`` / ``sim_now`` / …), so the
gym env runs unchanged — but every operation is a direct Python call on the
live Isaac ``World`` / articulation instead of a ROS2 round-trip:

- **reset / teleport** → ``articulation.set_world_pose`` + zeroed velocities,
  the per-episode respawn the ROS bring-up could not expose (no ``gz`` server).
- **actuation** → the diff-drive twist is mapped to 4 wheel velocities (same
  kinematics as the bring-up's ``WHEEL_SCRIPT``) and applied via
  ``ArticulationAction``.
- **stepping** → ``world.step()`` advanced ``control_dt / physics_dt`` sub-steps,
  rendering once per control step when the Lane Cam is active.
- **pose / speed** → read straight from the articulation root (ground truth, so
  the per-episode odom→world calibration the Gazebo path needs is a no-op here).
- **camera** → the latest Replicator ``rgb`` annotator frame, RGBA→BGR to match
  ``camera_pipeline.decode_image``'s output for the policy obs + cage CV path.

This module stays import-light (numpy only at top level); the one Isaac import
(``ArticulationAction``) is deferred into a method so importing the package off
the Isaac host (pytest, the campaign runner) never pulls in ``isaacsim``. The
live Isaac handles (``world``, ``articulation``, ``annotator``) are constructed
by the entry point (``tools/isaac_train.py``) and passed in.
"""

from __future__ import annotations

import math
from collections import deque
from typing import List, Optional, Sequence, Tuple

import numpy as np


class _PrintLogger:
    """Minimal stand-in for an rclpy node logger (``get_logger()``)."""

    def __init__(self, name: str = "isaac_interface") -> None:
        self._name = name

    def info(self, msg: str) -> None:
        print(f"[{self._name}] {msg}")

    def warning(self, msg: str) -> None:
        print(f"[{self._name}] WARN: {msg}")

    # rclpy loggers expose `warn` as an alias; mirror it.
    warn = warning

    def error(self, msg: str) -> None:
        print(f"[{self._name}] ERROR: {msg}")

    def debug(self, msg: str) -> None:  # pragma: no cover - parity only
        pass


class IsaacSimInterface:
    """Drive the RL env against a live in-process Isaac-Sim world.

    Parameters
    ----------
    world:
        The ``isaacsim.core.api.World`` (already ``reset()``); advanced via
        ``world.step()``.
    articulation:
        The robot ``SingleArticulation`` (already ``initialize()``).
    wheel_dof_indices:
        DOF indices of the four wheel joints in the order
        ``[front_left, rear_left, front_right, rear_right]`` — matching the
        ``[v_l, v_l, v_r, v_r]`` command order (see ``isaac_scene.WHEEL_JOINTS``).
    wheel_radius, wheel_separation:
        Diff-drive geometry (``isaac_scene.WHEEL_RADIUS`` / ``WHEEL_SEPARATION``).
    physics_dt:
        Sim seconds advanced per ``world.step`` (the world's physics dt).
    spawn_z:
        Teleport height for ``set_vehicle_pose`` (``isaac_scene.SPAWN_Z``).
    annotator:
        A Replicator ``rgb`` annotator already attached to the Lane Cam render
        product, or ``None`` for the F-track state-vector path (no camera).
    randomizer:
        An optional :class:`isaac_dr.IsaacDomainRandomizer`. When present,
        ``reset_world`` re-samples its per-episode parameters, ``send_action``
        scales the commanded yaw by ``randomizer.yaw_scale``, and the wheel
        command is delayed by ``randomizer.latency_steps`` control cycles
        (sim-to-real DR levers #2/#3). ``None`` => no randomization (the existing
        deterministic path; the Gazebo interface never gets one).
    """

    def __init__(
        self,
        world,
        articulation,
        wheel_dof_indices: Sequence[int],
        *,
        wheel_radius: float,
        wheel_separation: float,
        physics_dt: float,
        spawn_z: float,
        annotator=None,
        render_always: bool = False,
        randomizer=None,
    ) -> None:
        self.world = world
        self.articulation = articulation
        self._wheel_dof_indices = np.asarray(list(wheel_dof_indices), dtype=np.int64)
        self._wheel_radius = float(wheel_radius)
        self._wheel_half = float(wheel_separation) / 2.0
        self._physics_dt = float(physics_dt)
        self._spawn_z = float(spawn_z)
        self.annotator = annotator
        self.camera_enabled = annotator is not None
        # Force a render on the last sub-step even without a camera, so the GUI
        # viewport refreshes when a human is watching the state-vector path
        # (headless training never renders → frozen viewport). Costs one render
        # per control step; off by default so headless campaigns pay nothing.
        self.render_always = bool(render_always)
        self.randomizer = randomizer

        self._logger = _PrintLogger()
        self._sim_time = 0.0
        self._wheel_cmd: List[float] = [0.0, 0.0, 0.0, 0.0]
        # Actuation-latency buffer (DR lever #3): holds pending wheel commands so
        # the one applied this sub-step is `latency_steps` cycles old. Empty / no
        # delay unless a randomizer supplies a positive latency.
        self._cmd_buffer: "deque" = deque()
        self._cam_frame: Optional[Tuple[np.ndarray, float]] = None
        self._articulation_action_cls = None  # lazily imported (see _apply_wheel)

        # Number of DOFs, for the velocity zeroing on reset. Fall back to the
        # max wheel index + 1 when the attribute is unavailable.
        num_dof = getattr(articulation, "num_dof", None)
        if callable(num_dof):  # some builds expose it as a method
            try:
                num_dof = num_dof()
            except Exception:  # pragma: no cover - defensive
                num_dof = None
        if num_dof is None:
            num_dof = int(self._wheel_dof_indices.max()) + 1
        self._num_dof = int(num_dof)

    # --- logging / lifecycle -------------------------------------------------
    def get_logger(self) -> _PrintLogger:
        return self._logger

    def destroy_node(self) -> None:
        """Best-effort teardown (parity with the ROS node API)."""
        try:
            if self.annotator is not None:
                self.annotator.detach()
        except Exception:  # pragma: no cover - best effort
            pass

    def set_real_time_factor(self, real_time_factor: float, max_step_size: float = 0.001) -> bool:
        """No-op in-process: headless stepping already runs as fast as the host
        can render/integrate. Kept for API parity with the Gazebo interface."""
        self._logger.info(
            "set_real_time_factor is a no-op for in-process Isaac stepping "
            f"(requested {real_time_factor}); runs as fast as the host allows."
        )
        return True

    # --- time ----------------------------------------------------------------
    def sim_now(self) -> Optional[float]:
        return float(self._sim_time)

    # --- actuation -----------------------------------------------------------
    def send_action(self, steer: float, speed: float) -> None:
        """Map a diff-drive twist (``angular.z = steer``, ``linear.x = speed``)
        to the four wheel velocities, in the same way as the bring-up's
        ``WHEEL_SCRIPT``. The command is held and (re-)applied each sub-step.

        With a randomizer, the commanded yaw is scaled by ``yaw_scale`` (dynamics
        DR) and the resulting wheel command is delayed by ``latency_steps``
        control cycles via ``_cmd_buffer`` (actuation-latency DR): until the
        buffer fills the actuator is idle, mirroring real start-up latency."""
        v = float(speed)
        w = float(steer)
        latency = 0
        if self.randomizer is not None:
            w *= float(self.randomizer.yaw_scale)
            latency = int(self.randomizer.latency_steps)
        v_l = (v - w * self._wheel_half) / self._wheel_radius
        v_r = (v + w * self._wheel_half) / self._wheel_radius
        cmd = [v_l, v_l, v_r, v_r]
        if latency <= 0:
            self._wheel_cmd = cmd
            return
        self._cmd_buffer.append(cmd)
        if len(self._cmd_buffer) > latency:
            self._wheel_cmd = self._cmd_buffer.popleft()
        else:
            self._wheel_cmd = [0.0, 0.0, 0.0, 0.0]

    def _apply_wheel(self) -> None:
        if self._articulation_action_cls is None:
            from isaacsim.core.utils.types import ArticulationAction
            self._articulation_action_cls = ArticulationAction
        action = self._articulation_action_cls(
            joint_velocities=np.asarray(self._wheel_cmd, dtype=np.float32),
            joint_indices=self._wheel_dof_indices,
        )
        self.articulation.apply_action(action)

    def _capture_frame(self) -> None:
        """Pull the freshest Lane Cam frame from the annotator (RGBA→BGR uint8),
        stamped with the current sim time. No-op without a camera."""
        if not self.camera_enabled or self.annotator is None:
            return
        try:
            data = self.annotator.get_data()
        except Exception:  # pragma: no cover - annotator not ready yet
            return
        if data is None:
            return
        arr = np.asarray(data)
        if arr.ndim < 3 or arr.shape[0] == 0 or arr.shape[1] == 0:
            return
        # Replicator "rgb" yields HxWx4 RGBA (or HxWx3 RGB); drop alpha, RGB→BGR
        # so the frame matches camera_pipeline.decode_image's BGR contract.
        bgr = np.ascontiguousarray(arr[..., 2::-1]).astype(np.uint8, copy=False)
        self._cam_frame = (bgr, float(self._sim_time))

    def _advance(self, duration: float) -> None:
        """Step the sim by ``duration`` sim-seconds, holding the current wheel
        command, rendering (and capturing a frame) once on the last sub-step
        when the Lane Cam is active."""
        n = max(1, int(round(float(duration) / self._physics_dt)))
        for i in range(n):
            self._apply_wheel()
            render = (i == n - 1) and (self.camera_enabled or self.render_always)
            self.world.step(render=render)
            self._sim_time += self._physics_dt
            if render:
                self._capture_frame()

    def step_ros(self, duration: float) -> None:
        """Advance one control cycle (the env calls this after ``send_action``)."""
        self._advance(duration)

    def spin_wall(self, duration: float) -> None:
        """In-process equivalent of 'let real time pass': just advance the sim so
        fresh state/frames appear (used during reset settle + cage priming)."""
        self._advance(duration)

    def wait_for_initial_data(self, timeout_sec: float = 10.0) -> bool:
        """Step once so pose/velocity (and a first camera frame) are valid."""
        self._advance(self._physics_dt)
        return True

    # --- reset / teleport ----------------------------------------------------
    def reset_world(self) -> None:
        """Soft reset: command zero motion (the world is not reloaded) and, with
        a randomizer, re-sample this episode's physics/scene/latency parameters.
        The latency buffer is cleared so no command leaks across episodes."""
        self._wheel_cmd = [0.0, 0.0, 0.0, 0.0]
        self._cmd_buffer.clear()
        if self.randomizer is not None:
            self.randomizer.randomize_episode()

    def set_vehicle_pose(self, x: float, y: float, yaw: float) -> None:
        """Teleport the robot to ``(x, y, yaw)`` and zero its velocities — the
        per-episode respawn. Orientation quaternion is (w, x, y, z), Isaac's
        convention for ``set_world_pose``."""
        half = 0.5 * float(yaw)
        position = np.array([float(x), float(y), self._spawn_z], dtype=np.float32)
        orientation = np.array(
            [math.cos(half), 0.0, 0.0, math.sin(half)], dtype=np.float32
        )
        self.articulation.set_world_pose(position=position, orientation=orientation)
        self._zero_velocities()
        self._wheel_cmd = [0.0, 0.0, 0.0, 0.0]
        self._apply_wheel()

    def _zero_velocities(self) -> None:
        """Kill residual root + joint velocity so momentum never carries across
        episodes. Best-effort: a missing setter is warned, not fatal."""
        zeros3 = np.zeros(3, dtype=np.float32)
        try:
            self.articulation.set_linear_velocity(zeros3)
            self.articulation.set_angular_velocity(zeros3)
            self.articulation.set_joint_velocities(
                np.zeros(self._num_dof, dtype=np.float32)
            )
        except Exception as exc:  # pragma: no cover - API-shape guard
            self._logger.warning(
                f"could not fully zero velocities on reset ({exc}); residual "
                "momentum may leak across episodes."
            )

    def calibrate_pose_offset(self, true_x: float, true_y: float, true_yaw: float) -> None:
        """No-op: the articulation pose is already ground-truth world frame, so
        there is no odom→world offset to calibrate (kept for API parity)."""
        return None

    # --- sensing -------------------------------------------------------------
    def get_pose(self) -> Tuple[float, float, float]:
        """World-frame ``(x, y, yaw)`` from the articulation root (ground truth)."""
        position, orientation = self.articulation.get_world_pose()
        x = float(np.asarray(position).reshape(-1)[0])
        y = float(np.asarray(position).reshape(-1)[1])
        q = np.asarray(orientation).reshape(-1)
        yaw = self._yaw_from_quat(float(q[0]), float(q[1]), float(q[2]), float(q[3]))
        return x, y, yaw

    def get_speed(self) -> float:
        """Magnitude of the articulation root linear velocity (m/s)."""
        try:
            v = self.articulation.get_linear_velocity()
        except Exception:  # pragma: no cover - before first step
            return 0.0
        if v is None:
            return 0.0
        return float(np.linalg.norm(np.asarray(v, dtype=float).reshape(-1)))

    def clear_camera_frame(self) -> None:
        self._cam_frame = None

    def get_camera_frame(self) -> Optional[Tuple[np.ndarray, float]]:
        """Latest Lane Cam frame as ``(bgr_uint8, sim_stamp_s)`` or ``None``."""
        return self._cam_frame

    @staticmethod
    def _yaw_from_quat(w: float, x: float, y: float, z: float) -> float:
        """Planar yaw (rad) from an (w, x, y, z) quaternion."""
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
