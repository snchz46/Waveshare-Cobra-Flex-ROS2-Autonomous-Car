"""Gymnasium environment wrapping the Gazebo lane-following loop.

:class:`GazeboLaneEnv` is the single training/eval environment for both tracks:

- **State obs** (F-track): ``[ey, epsi, speed, prev_steer, kappa_near, kappa_far]``
  from the ground-truth pose projected onto the centerline.
- **Camera obs** (track 'E', D-41): 84×84 grayscale front-camera frames; the
  safety cage then reads the deterministic CV lane-estimator (D-43), never the
  ground truth, which remains the reward/termination/metrics oracle only.

Two config-gated extensions carry the Isaac posterior track (D-49/D-50), both
inert by default so every frozen F/E-track run is bit-identical:

- **2-D action** (``action.type: steer_throttle``): the policy commands
  steering *and* throttle; the cage's speed rules (C-04/C-05/C-06) then
  genuinely arbitrate against the policy instead of seeing a fixed cruise
  nominal, and SR-009's stall/liveness sub-mode becomes well-posed.
- **Multi-circuit training** (``circuits=[...]``): one circuit is sampled per
  episode (seeded) so the policy generalises across track shapes instead of
  overfitting a single loop.

The safety cage runs *in-process* inside :meth:`step` (D-34/TS-01) using the
same ``SafetyCageNode``/``cage.yaml`` as deployment, in ``enforcement`` (safe
action actuated) or ``monitoring`` (raw action actuated, cage shadow-logged)
mode. F4 scenario execution hooks in through ``reset(options=...)``: spawn
arc-length/offsets, runtime perturbations (SC-PERT-*) and visual degradations.
Design spec: docs/09_environment_design.md; reward: docs/10_reward_function.md.
"""

from __future__ import annotations

import math
from collections import deque
from typing import TYPE_CHECKING, Any, Dict, Mapping, Optional

import gymnasium as gym
from gymnasium import spaces
import numpy as np

from .cage_bridge import (
    SafetyCageNode,
    build_cage_state,
    policy_throttle_to_cage,
    resolve_cage_yaml,
    safe_action_to_cmd,
    safe_action_to_cmd_2d,
    target_speed_from_throttle_2d,
)
from .cage_perception import CagePerceptionSupervisor
from .cage_viz import CageViz
from .camera_pipeline import CameraPipeline
from .polyline_tracker import PolylineTracker, TrackState
from .rewards import compute_reward
from .scenario_perturbations import NONE as NO_PERTURBATION

# The env is transport-agnostic: it drives any object implementing the
# interface contract — RosGazeboInterface (ROS2/Gazebo, pulls in rclpy) or
# IsaacSimInterface (in-process Isaac Sim). Import only for typing so the env
# can be used on the Isaac host without rclpy installed.
if TYPE_CHECKING:
    from .ros_interface import RosGazeboInterface
from .scenario_perturbations import ScenarioPerturbation
from .visual_degradation import degrade
from .visual_domain_randomization import (
    DomainRandomizationConfig,
    VisualDomainRandomizer,
)


class GazeboLaneEnv(gym.Env):
    """Lane-following env over a live Gazebo instance (see module docstring).

    Default (F/E-track verdict path): the policy controls steering only
    ([-1, 1]); throttle is a fixed cruise nominal so the cage's speed rules see
    a realistic throttle stream (Training Spec §7.2.2). With
    ``action.type: steer_throttle`` (D-50, Isaac track) the policy additionally
    commands throttle in [-1, 1], mapped to the cage's normalised u ∈ [0, 1].
    All simulator I/O goes through ``ros_interface`` (Gazebo/ROS2 or in-process
    Isaac — the env is transport-agnostic).
    """

    metadata = {"render_modes": []}

    def __init__(
        self,
        ros_interface: RosGazeboInterface,
        centerline: Optional[np.ndarray] = None,
        lane_width: Optional[float] = None,
        cfg: Mapping[str, Any] = None,
        road_width: Optional[float] = None,
        road_centerline: Optional[np.ndarray] = None,
        circuits: Optional[Any] = None,
        calibration_mode: bool = False,
    ) -> None:
        super().__init__()
        self.ros_interface = ros_interface
        self.cfg = dict(cfg)
        self.fixed_speed = float(self.cfg.get("fixed_speed", 0.2))
        self.control_dt = float(self.cfg.get("control_dt", 0.1))
        self.max_episode_steps = int(self.cfg.get("max_episode_steps", 500))
        # Evidence-only escape hatch. Normal training/evaluation retains its
        # terminate-on-C-05 semantics; the D-43/C-02 calibrator uses this to
        # observe the full controlled stop after a moving heading injection.
        self.calibration_mode = bool(calibration_mode)

        # --- Action space (D-49/D-50) -----------------------------------------
        # "steer" (default): the frozen 1-D steering-only contract (ED-2) — every
        # F/E-track verdict ran on it and stays bit-identical. "steer_throttle"
        # (Isaac posterior track): 2-D [steer, throttle] in [-1, 1]²; throttle is
        # mapped to the cage scale u ∈ [0, 1] and actuated on the
        # target_speed_from_throttle_2d map (up to max_speed_mps), giving the
        # policy genuine speed authority above the C-04 ceilings so the cage's
        # speed rules arbitrate for real (see cage_bridge for the full design).
        action_cfg = dict(self.cfg.get("action", {}))
        self.action_type = str(action_cfg.get("type", "steer"))
        if self.action_type not in ("steer", "steer_throttle"):
            raise ValueError(
                f"action.type must be 'steer' or 'steer_throttle', got "
                f"{self.action_type!r}"
            )
        self.throttle_as_action = self.action_type == "steer_throttle"
        self.max_speed = float(action_cfg.get("max_speed_mps", 0.5))
        self.throttle_deadband = float(action_cfg.get("throttle_deadband", 0.05))
        # Highest speed the actuation map can command — the along-track advance
        # bound for progress-bounded tracking below.
        speed_ceiling = self.max_speed if self.throttle_as_action else self.fixed_speed

        # Progress-bounded tracking (opt-in): cap how far the centerline
        # projection may advance per control step so the ground-truth ey (and
        # thus the off-road termination) stays valid on a self-approaching
        # circuit such as complex_b — where leaving the lane otherwise lands the
        # agent near a *different* track section and collapses |ey|. Default off
        # => the convex F-track oval is unchanged. Auto cap = max along-track
        # travel per step (speed ceiling * control_dt) with a 1.5x margin for
        # projection discretisation; max_track_advance_m overrides.
        max_advance: Optional[float] = None
        if bool(self.cfg.get("progress_bounded_tracking", False)):
            max_advance = float(
                self.cfg.get(
                    "max_track_advance_m", 1.5 * speed_ceiling * self.control_dt
                )
            )
        # With progress-bounded tracking the projection deliberately *lags* when
        # the agent races off the lane, so off-road is judged on the Euclidean
        # distance to the (lagging) closest point — which captures both the
        # lateral departure and the longitudinal lag — rather than the perpendicular
        # ey alone, which collapses when the lane folds back near the agent.
        self.progress_bounded = max_advance is not None

        # --- Circuits (D-50 multi-track training) -----------------------------
        # The legacy single-circuit signature (centerline/lane_width/road_width/
        # road_centerline) is folded into a one-element circuit list; `circuits`
        # (a sequence of mappings with keys name/centerline/lane_width/
        # road_width/road_centerline) enables per-episode track sampling. Each
        # circuit pre-builds its trackers once; reset() switches the active one.
        #
        # Off-road geometry tracker per circuit: the reward centerline is the
        # *right lane* (offset), but "left the painted road" is a property of the
        # road, which is centred on the road centreline — so off-road is judged
        # by the global distance to the road centreline vs road_width/2 (the
        # edge). This is robust to a circuit approaching itself (complex_b),
        # where the stateful cross-track ey collapses when the vehicle drifts
        # near a different track section. Falls back to the reward tracker's ey
        # when no road centreline is supplied (the F-track oval).
        if circuits is None:
            if centerline is None or lane_width is None:
                raise ValueError(
                    "either (centerline, lane_width) or circuits=[...] required"
                )
            circuits = [{
                "name": "circuit_0",
                "centerline": centerline,
                "lane_width": lane_width,
                "road_width": road_width,
                "road_centerline": road_centerline,
            }]
        self._circuits = []
        for i, spec in enumerate(circuits):
            spec = dict(spec)
            c_lane = float(spec["lane_width"])
            c_road = (
                float(spec["road_width"])
                if spec.get("road_width") is not None
                else c_lane
            )
            c_road_cl = spec.get("road_centerline")
            self._circuits.append({
                "name": str(spec.get("name", f"circuit_{i}")),
                "tracker": PolylineTracker(
                    np.asarray(spec["centerline"], dtype=float),
                    max_advance_m=max_advance,
                ),
                "road_tracker": (
                    PolylineTracker(np.asarray(c_road_cl, dtype=float))
                    if c_road_cl is not None
                    else None
                ),
                "lane_width": c_lane,
                "road_width": c_road,
            })
        self._multi_circuit = len(self._circuits) > 1
        self._active_circuit = 0
        self._select_circuit(0)
        # Optional RViz view of the cage/agent runtime state (off by default so
        # headless campaigns pay nothing). Publishes /cage/markers (road edges,
        # vehicle state) and /agent/obs_image each step. See cage_viz.CageViz.
        self._viz = None
        if bool(self.cfg.get("viz", False)):
            # CageViz publishes to the ROS bus for RViz. The Gazebo interface is
            # a live rclpy node; the in-process Isaac interface is not (no ROS),
            # so it cannot create publishers. Skip viz with a warning there
            # rather than crashing the run — headless Isaac training has no RViz.
            if hasattr(self.ros_interface, "create_publisher"):
                if self._multi_circuit:
                    self.ros_interface.get_logger().warning(
                        "viz=true with multiple circuits: RViz markers show "
                        "circuit 0 only (static overlay)."
                    )
                viz_circuit = self._circuits[0]
                self._viz = CageViz(
                    self.ros_interface,
                    reward_centerline=np.asarray(
                        viz_circuit["tracker"].points, dtype=float
                    ),
                    road_centerline=(
                        np.asarray(viz_circuit["road_tracker"].points, dtype=float)
                        if viz_circuit["road_tracker"] is not None
                        else None
                    ),
                    road_width=self.road_width,
                )
            else:
                self.ros_interface.get_logger().warning(
                    "viz=true requested but the sim interface cannot publish to "
                    "ROS (in-process Isaac); RViz markers disabled for this run."
                )
        self.prev_steer = 0.0
        # Raw (pre-cage) policy steering of the previous cycle. The smoothness
        # reward term penalises the *raw* delta, not the post-cage applied delta,
        # so the policy pays for its own bang-bang instead of letting C-06 absorb
        # it for free (§7.2.5, §7.5.2; reward v1.2). Kept separate from
        # prev_steer, which is the applied steering exposed in the observation.
        self.prev_policy_steer = 0.0
        # Raw policy throttle (cage scale u ∈ [0, 1]) of the previous cycle, for
        # the 2-D throttle smoothness term (same raw-not-post-cage rationale;
        # D-50). Unused (stays 0.0) on the 1-D path.
        self.prev_policy_throttle = 0.0
        self.step_count = 0
        self.last_track_state: Optional[TrackState] = None
        self._last_pose = (0.0, 0.0, 0.0)  # world (x, y, yaw), refreshed each cycle
        self.prev_s = 0.0

        # Signed-curvature preview added to the observation so the policy can
        # anticipate the upcoming curve instead of only reacting to ey/epsi (the
        # cage already consumes curvature internally; the policy did not). Two
        # look-ahead horizons (near + far, in centerline segments).
        obs_cfg = dict(self.cfg.get("observation", {}))
        self.curv_lookahead_near = int(obs_cfg.get("curvature_lookahead_near", 3))
        self.curv_lookahead_far = int(obs_cfg.get("curvature_lookahead_far", 8))

        if self.throttle_as_action:
            # [steer, throttle], both in [-1, 1] (symmetric Box, SB3-friendly);
            # throttle is remapped to the cage scale u ∈ [0, 1] in step().
            self.action_space = spaces.Box(
                low=np.array([-1.0, -1.0], dtype=np.float32),
                high=np.array([1.0, 1.0], dtype=np.float32),
                dtype=np.float32,
            )
        else:
            self.action_space = spaces.Box(
                low=np.array([-1.0], dtype=np.float32),
                high=np.array([1.0], dtype=np.float32),
                dtype=np.float32,
            )
        # Track 'E' (D-41/D-43): observation.type "camera" switches the policy
        # obs to the front-camera image (84×84 grayscale, docs/09 §10; frame
        # stacking k=4 is applied by the trainer via VecFrameStack). The cage's
        # state then comes from the deterministic CV lane-estimator behind the
        # CagePerceptionSupervisor — never from ground truth, which stays the
        # reward/termination/metrics oracle only.
        self.obs_type = str(obs_cfg.get("type", "state"))
        if self.obs_type == "camera":
            cam_cfg = dict(obs_cfg.get("camera", {}))
            grayscale = bool(cam_cfg.get("grayscale", True))
            obs_w = int(cam_cfg.get("width", 84))
            obs_h = int(cam_cfg.get("height", 84))
            self.camera_pipeline = CameraPipeline(
                obs_width=obs_w, obs_height=obs_h, grayscale=grayscale
            )
            # Optional Isaac-calibration override (D-55): the supervisor's
            # invalid-persistence budget was live-tuned on GAZEBO frames
            # (min_invalid_cycles=4 bridges its ~2-cycle dash gaps). The Isaac
            # renderer has ONE longer genuinely-blind stretch (U-turn exit,
            # lane leaves the image) that needs a wider bridge. Unset -> the
            # tuned default, bit-identical for every Gazebo config.
            _cage_cfg = dict(self.cfg.get("cage", {}) or {})
            invalid_cycles = _cage_cfg.get("perception_min_invalid_cycles")
            # Isaac heading-bias calibration (D-57): de-bias the estimator's
            # heading for the Isaac renderer. Unset/0.0 -> the Gazebo estimator,
            # bit-identical (D-43 verdicts untouched).
            heading_bias = float(_cage_cfg.get("perception_heading_bias_rad", 0.0))
            heading_fit_mode = str(
                _cage_cfg.get("perception_heading_fit_mode", "near_secant")
            )
            heading_gain = float(_cage_cfg.get("perception_heading_gain", 1.0))
            # Temporal heading-consistency gate (T3, D-62). 0 -> disabled, so
            # every config without the key builds a bit-identical estimator.
            heading_tw = int(_cage_cfg.get("perception_heading_temporal_window", 0))
            sup_kwargs = {}
            if (
                heading_bias != 0.0
                or heading_fit_mode != "near_secant"
                or heading_gain != 1.0
                or heading_tw > 0
            ):
                from .cv_lane_estimator import CvLaneEstimator, CvLaneEstimatorConfig
                _est_kwargs = dict(
                    heading_bias_rad=heading_bias,
                    heading_fit_mode=heading_fit_mode,
                    heading_gain=heading_gain,
                    heading_temporal_window=heading_tw,
                )
                # Optional T3 tunables; defaults match CvLaneEstimatorConfig.
                for _key, _cfgkey in (
                    ("heading_temporal_ey_track_m", "perception_heading_temporal_ey_track_m"),
                    ("heading_temporal_ey_drift_m", "perception_heading_temporal_ey_drift_m"),
                    ("heading_temporal_kappa_gate", "perception_heading_temporal_kappa_gate"),
                    ("heading_temporal_cap_rad", "perception_heading_temporal_cap_rad"),
                ):
                    if _cfgkey in _cage_cfg:
                        _est_kwargs[_key] = float(_cage_cfg[_cfgkey])
                sup_kwargs["estimator"] = CvLaneEstimator(
                    config=CvLaneEstimatorConfig(**_est_kwargs)
                )
            if invalid_cycles is not None:
                from .perception_health import PerceptionHealthMonitor
                sup_kwargs["health"] = PerceptionHealthMonitor(
                    min_confidence=0.10, min_invalid_cycles=int(invalid_cycles))
            self.cage_perception = CagePerceptionSupervisor(**sup_kwargs)
            self.observation_space = spaces.Box(
                low=0,
                high=255,
                shape=(obs_h, obs_w, 1 if grayscale else 3),
                dtype=np.uint8,
            )
            # Last processed (possibly degraded) native frame + sim stamp:
            # captured at the end of each cycle, consumed by the cage at the
            # start of the next (same cadence as last_track_state).
            self._cam_frame: Optional[np.ndarray] = None
            self._cam_stamp: float = 0.0
            self._last_obs_img: Optional[np.ndarray] = None
            # Training-side visual domain randomisation (H-10 mitigation,
            # SR-012): one degradation spec drawn per episode from the H-10
            # envelope, reproducible via self.np_random (seeded by reset).
            # Disabled for deterministic evaluation; scenario injectors
            # passed via reset options take precedence.
            dr_cfg = dict(self.cfg.get("domain_randomization", {}))
            self.dr_enabled = bool(dr_cfg.get("enabled", False))
            self.domain_randomizer: Optional[VisualDomainRandomizer] = None
            self._dr_spec = None
            if self.dr_enabled:
                self.domain_randomizer = VisualDomainRandomizer(
                    DomainRandomizationConfig(
                        p_degrade=float(dr_cfg.get("p_degrade", 0.5)),
                        modes=tuple(
                            dr_cfg.get(
                                "modes",
                                DomainRandomizationConfig().modes,
                            )
                        ),
                        level_range=tuple(
                            dr_cfg.get("level_range", (0.2, 1.0))
                        ),
                    )
                )
        else:
            # [ey, epsi, speed, prev_steer, kappa_near, kappa_far]
            self.observation_space = spaces.Box(
                low=np.array(
                    [-np.inf, -math.pi, 0.0, -1.0, -np.inf, -np.inf], dtype=np.float32
                ),
                high=np.array(
                    [np.inf, math.pi, np.inf, 1.0, np.inf, np.inf], dtype=np.float32
                ),
                dtype=np.float32,
            )

        # Safety cage in the training loop (D-34, TS-01). The cage is invoked
        # in-process via the same SafetyCageNode/cage.yaml that cage_ros_node
        # wraps in deployment; the actuation constants below mirror
        # vehicle_control_node so the env emits the same /cmd_vel mapping the
        # policy will face at deployment. cage_enabled=False keeps the legacy
        # no-cage loop for pipeline debugging (Training Spec §7.2.5).
        cage_cfg = dict(self.cfg.get("cage", {}))
        self.cage_enabled = bool(cage_cfg.get("enabled", True))
        self.cage_mode = str(cage_cfg.get("mode", "enforcement"))
        self.cage_yaml_path = str(cage_cfg.get("yaml_path", "") or "")
        self.lookahead_segments = int(cage_cfg.get("lookahead_segments", 5))
        self.throttle_nominal = float(cage_cfg.get("throttle_nominal", 0.5))
        self.yaw_gain = float(cage_cfg.get("yaw_gain", 0.8))
        self.min_speed_scale = float(cage_cfg.get("min_speed_scale", 0.35))
        self.cage: Optional[SafetyCageNode] = None

        # F4 runtime perturbation injection (SC-PERT-01/02, SC-EDGE-03). Set per
        # episode from reset(options["perturbation"]); NONE for training / nominal
        # eval. `_perceived_ey` is the (noisy) lateral offset fed to the policy
        # observation and the cage; `_cmd_delay` buffers /cmd_vel for actuation
        # latency. The verdict is always measured on the *true* pose.
        self._perturbation: ScenarioPerturbation = NO_PERTURBATION
        self._perceived_ey: float = 0.0
        self._cmd_delay: "deque" = deque()

        # Random spawn perturbation per episode (Training Spec §7.3) for
        # start-state diversity. Disabled (exact centerline spawn) for
        # deterministic evaluation. Reproducible via self.np_random, seeded by
        # reset(seed=...).
        spawn_cfg = dict(self.cfg.get("spawn_perturbation", {}))
        self.spawn_perturb_enabled = bool(spawn_cfg.get("enabled", True))
        self.spawn_heading_range = float(spawn_cfg.get("heading_rad", 0.15))
        self.spawn_lateral_range = float(spawn_cfg.get("lateral_m", 0.05))
        # Random along-track spawn (curriculum, isaac-diagnostic): when set, an
        # episode with no explicit start_s in reset(options) spawns at a uniform
        # random arc-length instead of always the start line. This gives the
        # policy experience of EVERY part of the track — notably the tight U-turn
        # it otherwise rarely reaches (it dies there → no gradient there → never
        # learns the slow-and-turn). Default False → deterministic eval and every
        # existing config/RNG stream are bit-identical.
        self.random_start_s = bool(spawn_cfg.get("random_start_s", False))

    def _select_circuit(self, index: int) -> None:
        """Make circuit ``index`` the active geometry (trackers, widths, arc
        length). The per-circuit trackers are pre-built at construction, so
        switching is O(1); reset() re-arms the tracking state afterwards."""
        circuit = self._circuits[int(index)]
        self._active_circuit = int(index)
        self.tracker = circuit["tracker"]
        self._road_tracker = circuit["road_tracker"]
        self.lane_width = float(circuit["lane_width"])
        self.road_width = float(circuit["road_width"])
        # Total centerline arc length, for the closed-loop progress wrap (§7.2.3).
        self._track_length = float(self.tracker.cumulative_lengths[-1])

    def reset(
        self,
        *,
        seed: Optional[int] = None,
        options: Optional[Dict[str, Any]] = None,
    ):
        """Start a fresh episode: teleport to the spawn pose and re-arm the cage.

        ``options`` carries the F4 scenario hooks — ``start_s_m`` /
        ``lateral_offset_m`` / ``heading_error_rad`` for deterministic initial
        conditions, ``perturbation`` (a :class:`ScenarioPerturbation`) for
        runtime stressors, and ``visual_injector`` / ``visual_degradation``
        for camera-frame corruption. All are optional; training passes none.
        """
        super().reset(seed=seed)
        self.step_count = 0
        # prev_steer tracks the steering actually applied (post-cage) last cycle
        # (for the observation); prev_policy_steer tracks the raw policy command
        # last cycle (for the smoothness reward term, §7.2.5). The raw policy
        # throttle starts at 0.0 = stopped (the vehicle spawns stationary).
        self.prev_steer = 0.0
        self.prev_policy_steer = 0.0
        self.prev_policy_throttle = 0.0

        # Fresh cage per episode: no latched C-05 emergency, clean rate-limiter
        # and oscillation history. Each RL episode is an independent rollout, so
        # the cage starts from a known state (D-34, TS-01).
        if self.cage_enabled:
            self.cage = SafetyCageNode(
                resolve_cage_yaml(self.cage_yaml_path), mode=self.cage_mode
            )

        # Scenario initial conditions (F4 executor): an SC-* run can start at a
        # given centerline arc-length (`start_s_m`, e.g. 1.5 = curve entry), with
        # a deterministic heading error (`heading_error_rad`, e.g. SC-EDGE-01's
        # 15 deg) and/or lateral offset (`lateral_offset_m`). Absent (training /
        # nominal eval), the spawn is the first centerline point as before.
        opts = options or {}
        # Circuit for this episode (D-50 multi-track training): pinned via
        # options["circuit_index"] (deterministic eval), else sampled uniformly
        # from the circuit list via the seeded np_random. Single-circuit envs
        # skip the sampling entirely (legacy behaviour, identical RNG stream).
        circuit_index = opts.get("circuit_index")
        if circuit_index is not None:
            self._select_circuit(int(circuit_index))
        elif self._multi_circuit:
            self._select_circuit(int(self.np_random.integers(len(self._circuits))))
        # Runtime perturbation for this episode (obs noise / latency / throttle
        # pulse). Reset the latency buffer so no command leaks across episodes.
        self._perturbation = opts.get("perturbation") or NO_PERTURBATION
        self._cmd_delay.clear()
        start_s = opts.get("start_s_m")
        # SC-EDGE-05 kappa_seed: spawn on the curve whose curvature matches the
        # grid anchor's kappa_seed_rad_m, so C-04's curvature-parameterised speed
        # ceiling sees real geometry (overrides the scenario's straight start_s).
        kappa_seed = opts.get("kappa_seed_rad_m")
        if kappa_seed is not None:
            start_s = self.tracker.arclength_at_curvature(float(kappa_seed))
        # Curriculum: uniform random along-track spawn (only when no explicit
        # start_s / kappa_seed was requested, so eval/scenarios are unaffected).
        if start_s is None and self.random_start_s:
            start_s = float(self.np_random.uniform(
                0.0, float(self.tracker.cumulative_lengths[-1])))
        if start_s is not None:
            base_x, base_y, base_heading = self.tracker.pose_at_arclength(
                float(start_s), float(opts.get("lateral_offset_m", 0.0) or 0.0)
            )
            base_heading += float(opts.get("heading_error_rad", 0.0) or 0.0)
        else:
            start_point = self.tracker.points[0]
            base_x, base_y = float(start_point[0]), float(start_point[1])
            base_heading = float(self.tracker.segment_headings[0])
        spawn_x, spawn_y, spawn_heading = self._perturbed_spawn(
            base_x, base_y, base_heading
        )

        self.ros_interface.reset_world()
        self.ros_interface.set_vehicle_pose(spawn_x, spawn_y, spawn_heading)
        self.tracker.reset_tracking()
        self.ros_interface.send_action(0.0, 0.0)

        if not self.ros_interface.wait_for_initial_data(timeout_sec=10.0):
            raise RuntimeError("Timed out waiting for initial /odom data.")

        # Ground-truth odom is reported in the fixed `odom` frame whose origin is
        # the run's initial spawn, so each episode recalibrates the constant
        # odom->world offset against the known spawn pose. A set_pose teleport
        # propagates to /odom_truth a few sim steps *after* the service returns,
        # so calibrating immediately can latch the previous-crash pose and yield
        # an impossible multi-metre ey on step 1. Settle first (see helper).
        if not self._calibrate_spawn_settled(spawn_x, spawn_y, spawn_heading):
            self.ros_interface.get_logger().warning(
                "Spawn pose did not settle after teleport; first-step ey may be unreliable."
            )
        track_state = self._compute_track_state()
        speed = self.ros_interface.get_speed()
        self.last_track_state = track_state
        self.prev_s = float(track_state.s)

        if self.obs_type == "camera":
            self.cage_perception.reset()
            self.camera_pipeline.set_injector(self._resolve_visual_injector(opts))
            # Drop any pre-teleport frame and wait for a fresh post-teleport one.
            self.ros_interface.clear_camera_frame()
            observation = self._capture_camera_obs(wait_timeout_s=5.0)
            if observation is None:
                raise RuntimeError(
                    "Timed out waiting for a camera frame after episode reset "
                    "(is the camera bridged on this world?)."
                )
            # Prime the perception supervisor on the settled spawn view so the
            # cage's first cycle starts from an accepted state (and a temporal
            # reference for SR-014). Without priming, one bad first frame put
            # the cage on its no-state-ever path → instant emergency → 1-step
            # episodes. A scenario injector active from t=0 (e.g. SC-PERT-05
            # high) may legitimately never prime — proceed after the deadline;
            # the cage then answers with the controlled stop, as specified.
            self._prime_cage_perception(timeout_s=2.0)
            observation = self._last_obs_img  # freshest frame after priming
        else:
            kappa_near, kappa_far = self._curvature_preview(track_state.segment_index)
            observation = self._make_observation(
                self._perceived_ey, track_state.epsi, speed, self.prev_steer,
                kappa_near, kappa_far,
            )
        info = self._make_info(track_state, speed)
        return observation, info

    def _resolve_visual_injector(self, opts: Dict[str, Any]):
        """Per-episode camera degradation (applied once, before both consumers
        — the policy obs and the cage CV estimator; D-43 common cause).

        Three sources, in precedence order:
        ``options["visual_injector"]`` (any frame→frame callable, e.g. a bound
        ``VisualDomainRandomizer.apply`` for training DR),
        ``options["visual_degradation"]`` ({"mode", "level"} from an SC-PERT
        scenario), or None (clean).
        """
        injector = opts.get("visual_injector")
        if injector is not None:
            return injector
        vd = opts.get("visual_degradation")
        if vd:
            mode = str(vd["mode"])
            level = float(vd["level"])
            return lambda frame: degrade(frame, mode, level)
        # Scenario visual stressor (SC-PERT-04..08): onset-timed, reading the
        # episode clock so SC-PERT-07/08's nominal lead-in stays clean.
        if self._perturbation.kind == "visual_degradation":
            perturbation = self._perturbation

            def scenario_injector(frame):
                active = perturbation.visual_degradation_at(
                    self.step_count * self.control_dt
                )
                if active is None:
                    return frame
                mode, level = active
                return degrade(frame, mode, level)

            return scenario_injector
        if self.dr_enabled and self.domain_randomizer is not None:
            spec = self.domain_randomizer.sample(self.np_random)
            self._dr_spec = spec
            if not spec.is_clean:
                return lambda frame: self.domain_randomizer.apply(frame, spec)
        return None

    def _prime_cage_perception(self, timeout_s: float = 2.0) -> bool:
        """Run the supervisor on fresh spawn frames until it accepts one (or
        the deadline passes). Returns True when primed."""
        import time as _time

        deadline = _time.monotonic() + max(0.0, float(timeout_s))
        while True:
            now_sim = self.ros_interface.sim_now()
            result = self.cage_perception.update(
                self._cam_frame,
                frame_timestamp_s=self._cam_stamp,
                now_s=float(now_sim) if now_sim is not None else self._cam_stamp,
                speed_mps=self.ros_interface.get_speed(),
            )
            if result.state_available:
                return True
            if _time.monotonic() >= deadline:
                return False
            self.ros_interface.spin_wall(0.1)
            self._capture_camera_obs()

    def _capture_camera_obs(self, wait_timeout_s: float = 0.0):
        """Fetch the latest camera frame, run the shared pipeline, retain the
        degraded native frame for the cage's next cycle, and return the policy
        observation. Returns the previous observation when no new frame exists
        (20 Hz camera vs 10 Hz control: normally there is always one)."""
        import time as _time

        deadline = _time.monotonic() + max(0.0, float(wait_timeout_s))
        result = self.ros_interface.get_camera_frame()
        while result is None and _time.monotonic() < deadline:
            self.ros_interface.spin_wall(0.05)
            result = self.ros_interface.get_camera_frame()
        if result is None:
            return self._last_obs_img
        frame, stamp = result
        consumer, obs_img = self.camera_pipeline.process(frame)
        self._cam_frame = consumer
        self._cam_stamp = float(stamp)
        self._last_obs_img = obs_img
        return obs_img

    def inject_heading_fault_for_calibration(self, heading_delta_rad: float) -> dict:
        """Apply a one-cycle yaw impulse and refresh the CV/GT state.

        This method is deliberately unavailable outside ``calibration_mode``.
        The impulse goes through the real simulator actuation interface while
        preserving measured forward speed. Ground truth is read only to report
        the realised perturbation and label the resulting evidence.
        """
        if not self.calibration_mode:
            raise RuntimeError("heading fault injection requires calibration_mode")
        if self.obs_type != "camera":
            raise RuntimeError("heading fault injection requires camera observation")
        delta = float(heading_delta_rad)
        if not math.isfinite(delta) or abs(delta) < 1e-9:
            raise ValueError("heading_delta_rad must be finite and non-zero")

        initial_state = self._compute_track_state()
        _, _, yaw_before = self._last_pose
        speed_before = float(self.ros_interface.get_speed())
        impulse_rate = math.copysign(
            min(4.8, max(1.0, abs(delta) / self.control_dt)), delta
        )
        substep = min(0.02, self.control_dt)
        track_state = initial_state
        realised_epsi_delta = 0.0
        for _ in range(max(1, int(math.ceil(1.0 / substep)))):
            self.ros_interface.send_action(impulse_rate, speed_before)
            self.ros_interface.step_ros(substep)
            track_state = self._compute_track_state()
            realised_epsi_delta = math.atan2(
                math.sin(track_state.epsi - initial_state.epsi),
                math.cos(track_state.epsi - initial_state.epsi),
            )
            if realised_epsi_delta * math.copysign(1.0, delta) >= abs(delta):
                break
        # Remove the impulse immediately; the next env.step lets the cage own
        # steering and speed again.
        self.ros_interface.send_action(0.0, speed_before)

        self.last_track_state = track_state
        self.prev_s = float(track_state.s)
        self._capture_camera_obs(wait_timeout_s=1.0)
        _, _, yaw_after = self._last_pose
        realised = math.atan2(
            math.sin(yaw_after - yaw_before), math.cos(yaw_after - yaw_before)
        )
        return {
            "requested_heading_delta_rad": delta,
            "realised_heading_delta_rad": float(realised),
            "realised_epsi_delta_rad": float(realised_epsi_delta),
            "speed_before_mps": speed_before,
            "epsi_after_rad": float(track_state.epsi),
        }

    def _curvature_preview(self, segment_index: int):
        """Signed curvature (rad/m, + = left) at a near and a far look-ahead, for
        the observation. Mirrors the cage's curvature source so policy and cage
        see a consistent road preview."""
        near = self.tracker.curvature_ahead(segment_index, self.curv_lookahead_near)
        far = self.tracker.curvature_ahead(segment_index, self.curv_lookahead_far)
        return float(near), float(far)

    def _calibrate_spawn_settled(
        self,
        spawn_x: float,
        spawn_y: float,
        spawn_heading: float,
        tol: float = 0.08,
        max_attempts: int = 8,
        settle_dt: float = 0.06,
    ) -> bool:
        """Calibrate the odom->world offset against a *settled* post-teleport pose.

        ``calibrate_pose_offset`` forces ``get_pose() == spawn`` at the instant it
        runs, so a stale (pre-teleport) calibration only reveals itself once the
        real teleport lands and the raw odom jumps. We exploit that: the car is
        stationary (zero command) this cycle, so after a *valid* calibration it
        must stay at ``spawn`` across a short settle step. If a stale offset was
        latched, the teleport propagates during that step and ``get_pose`` drifts
        far from ``spawn`` — we detect it and recalibrate against the now-correct
        raw pose. Converges in 1–2 attempts; returns False if it never settles
        (e.g. the teleport silently failed).

        Settling uses *wall-clock* spins, not the sim-time step_ros: the teleport
        needs real server time to propagate, and a sim-time wait can be fooled
        into returning instantly by the odom backlog accumulated during the
        set_pose subprocess (which is exactly what latched stale calibrations).
        """
        for _ in range(max_attempts):
            self.ros_interface.spin_wall(settle_dt)  # let the teleport propagate
            self.ros_interface.calibrate_pose_offset(spawn_x, spawn_y, spawn_heading)
            self.ros_interface.spin_wall(settle_dt)  # verify the car stays put
            x, y, _ = self.ros_interface.get_pose()
            if math.hypot(x - spawn_x, y - spawn_y) <= tol:
                return True
        return False

    def _perturbed_spawn(self, x: float, y: float, heading: float):
        """Apply a random spawn perturbation (Training Spec §7.3) for start-state
        diversity: a lateral offset perpendicular to the track tangent plus a
        heading jitter. Returns the unperturbed pose when disabled (e.g. during
        deterministic evaluation). Uses self.np_random (seeded via reset(seed))
        so the perturbation is reproducible."""
        if not self.spawn_perturb_enabled:
            return x, y, heading
        dlat = float(
            self.np_random.uniform(-self.spawn_lateral_range, self.spawn_lateral_range)
        )
        dpsi = float(
            self.np_random.uniform(-self.spawn_heading_range, self.spawn_heading_range)
        )
        # +dlat is to the left of the tangent, matching the +left ey convention.
        return (
            x - dlat * math.sin(heading),
            y + dlat * math.cos(heading),
            heading + dpsi,
        )

    def step(self, action):
        """One control cycle: cage the action, actuate, advance sim, score.

        Pipeline: clip the policy steering → route through the cage
        (:meth:`_apply_cage`) → apply actuation latency if perturbed → publish
        the command and step Gazebo by ``control_dt`` → recompute the track
        state and reward. Terminates on off-road (true pose beyond the road
        half-width) or on an *enforced* C-05 emergency stop; truncates at
        ``max_episode_steps``.
        """
        action_vec = np.asarray(action).reshape(-1)
        policy_steer = float(np.clip(action_vec[0], -1.0, 1.0))
        prev_policy_steer = self.prev_policy_steer
        prev_policy_throttle = self.prev_policy_throttle
        # 2-D action (D-50): the policy's symmetric throttle in [-1, 1] becomes
        # the cage-scale u ∈ [0, 1]. None on the 1-D path — _apply_cage then
        # substitutes the fixed cruise nominal (the frozen ED-2 contract).
        policy_throttle: Optional[float] = None
        if self.throttle_as_action:
            policy_throttle = policy_throttle_to_cage(action_vec[1])

        applied_steer, cmd_linear, cmd_angular, cage_info = self._apply_cage(
            policy_steer, policy_throttle
        )

        # Actuation latency (SC-PERT-02): the /cmd_vel command reaches the actuator
        # `latency_steps` cycles late. Identity for an unperturbed run.
        cmd_linear, cmd_angular = self._delayed_command(cmd_linear, cmd_angular)

        # send_action(steer, speed) publishes Twist(angular.z=steer, linear.x=speed).
        self.ros_interface.send_action(cmd_angular, cmd_linear)
        self.ros_interface.step_ros(self.control_dt)

        track_state = self._compute_track_state()
        speed = self.ros_interface.get_speed()
        self.last_track_state = track_state
        self.step_count += 1

        # Normalised progress along the centerline this cycle (§7.2.3): the
        # arc-length advance ds, unwrapped across the closed-loop s reset and
        # scaled by the nominal per-step advance so ~1.0 == one cruise step.
        ds = track_state.s - self.prev_s
        if ds < -0.5 * self._track_length:
            ds += self._track_length
        self.prev_s = float(track_state.s)
        ds_nominal = max(self.fixed_speed * self.control_dt, 1e-6)
        progress = float(np.clip(ds / ds_nominal, -2.0, 2.0))

        # End the episode the instant the cage latches a C-05 emergency stop:
        # the rollout has already failed (the policy drove into a state the cage
        # could only answer with an emergency), so the remaining frozen steps
        # carry no learning signal — they just burn wall-clock. Both conditions
        # set `terminated` (value target bootstraps from 0); they differ only in
        # the reward, see below. (Requested extension to D-34 / TS-01.)
        if self._road_tracker is not None:
            px, py, _ = self._last_pose
            off_road = self._road_tracker.distance_to(px, py) > (self.road_width * 0.5)
        elif self.progress_bounded:
            px, py, _ = self._last_pose
            cxp, cyp = track_state.closest_point
            off_road = math.hypot(px - cxp, py - cyp) > (self.road_width * 0.5)
        else:
            off_road = abs(track_state.ey) > (self.road_width * 0.5)
        cage_emergency = bool(cage_info.get("cage_emergency", False))
        # A cage emergency only ENDS the episode when the cage is actually
        # enforcing (the emergency stop is real). In monitoring the cage observes
        # only — the raw policy action is applied — so a *shadow* emergency must
        # not terminate the run; otherwise the no-cage counterfactual would stop
        # at the same point as enforcement and the cage-efficacy contrast (max
        # excursion / road-edge contact) would vanish. Training is enforcement, so
        # this leaves the training/eval terminate-on-emergency behaviour unchanged.
        cage_stop = (
            cage_emergency
            and (self.cage_mode == "enforcement")
            and not self.calibration_mode
        )
        terminated = off_road or cage_stop
        truncated = self.step_count >= self.max_episode_steps
        # Reward is computed on the resulting state (ey/epsi/progress) and, for
        # the smoothness term only, on the *raw* policy steering delta — D-34 /
        # Training Spec §7.2.5. The policy sees the cage as part of the
        # environment dynamics, not as an explicit penalty, so cage interventions
        # are never punished; but the smoothness term is the deliberate exception
        # (reward v1.2): measuring the post-cage delta is toothless because C-06
        # absorbs raw bang-bang for free (§7.5.2), so the policy pays for its own
        # raw jerk instead. Consistent with the no-punish-the-cage rule, a C-05
        # emergency termination carries NO termination penalty (the episode simply
        # ends, the policy only forgoes future reward); only a genuine off-road
        # failure, which predates the cage in the loop, incurs the penalty.
        reward = compute_reward(
            track_state=track_state,
            progress=progress,
            steer=policy_steer,
            prev_steer=prev_policy_steer,
            done=off_road,
            cfg=self.cfg,
            throttle=policy_throttle,
            prev_throttle=(
                prev_policy_throttle if policy_throttle is not None else None
            ),
        )

        self.prev_steer = applied_steer
        self.prev_policy_steer = policy_steer
        if policy_throttle is not None:
            self.prev_policy_throttle = policy_throttle
        if self.obs_type == "camera":
            observation = self._capture_camera_obs()
        else:
            kappa_near, kappa_far = self._curvature_preview(track_state.segment_index)
            observation = self._make_observation(
                self._perceived_ey, track_state.epsi, speed, self.prev_steer,
                kappa_near, kappa_far,
            )
        info = self._make_info(track_state, speed)
        info.update(cage_info)
        if terminated:
            info["termination_reason"] = "cage_emergency" if cage_stop else "off_road"
        elif truncated:
            info["termination_reason"] = "truncated"
        if self._viz is not None:
            self._viz.publish(
                self._last_pose,
                off_road=off_road,
                emergency=cage_emergency,
                interventions=cage_info.get("cage_interventions", []),
                obs=observation if self.obs_type == "camera" else None,
            )
        return observation, reward, terminated, truncated, info

    def _apply_cage(self, policy_steer: float, policy_throttle: Optional[float] = None):
        """Route the raw policy action through the safety cage in-process.

        Returns ``(applied_steer, cmd_linear_x, cmd_angular_z, info)`` where
        ``applied_steer`` is the normalised steering actually actuated (post-cage),
        used for the reward and the ``prev_steer`` observation. When the cage is
        disabled the raw action passes through with the legacy direct actuation
        (debug fallback, Training Spec §7.2.5).

        ``policy_throttle`` is the policy's cage-scale throttle u ∈ [0, 1] on the
        2-D action path (D-50), or ``None`` on the frozen 1-D path — the cage
        then sees the fixed cruise nominal as before (§7.2.2).
        """
        if not self.cage_enabled or self.cage is None or self.last_track_state is None:
            # Legacy no-cage loop: angular.z = steer; linear.x = fixed_speed
            # (1-D) or the 2-D throttle map (debug fallback, no cage in loop).
            if policy_throttle is not None:
                raw_throttle = policy_throttle
                cmd_linear = target_speed_from_throttle_2d(
                    policy_throttle, self.max_speed, self.throttle_deadband
                )
            else:
                raw_throttle = self.throttle_nominal
                cmd_linear = self.fixed_speed
            return (
                policy_steer,
                cmd_linear,
                policy_steer,
                {
                    "cage_enabled": False,
                    "cage_emergency": False,
                    "cage_interventions": [],
                    "cage_joint_envelope_violated": False,
                    "cage_oscillation_persistent": False,
                    "raw_steer": policy_steer,
                    "safe_steer": policy_steer,
                    "steer_correction": 0.0,
                    "raw_throttle": float(raw_throttle),
                    "safe_throttle": float(raw_throttle),
                    "throttle_correction": 0.0,
                },
            )

        # Raw throttle stream to the cage: the policy's own command on the 2-D
        # path (the cage speed rules then arbitrate *against* the policy, D-50),
        # else the fixed cruise nominal so C-04/C-05/C-06 act on a realistic
        # stream (§7.2.2). A throttle-override perturbation (SC-EDGE-03)
        # substitutes a timed pulse in either mode — the eval stressor takes
        # precedence over the policy. NB on the 1-D path the fixed-speed
        # actuation (target_speed_from_throttle) caps the resulting speed at
        # fixed_speed, so the *speed-excess magnitude* is limited there (see
        # scenario_perturbations / experiments/README); the 2-D map has real
        # speed authority up to max_speed instead.
        timestamp = self.step_count * self.control_dt
        override = self._perturbation.throttle_override(timestamp)
        base_throttle = (
            policy_throttle if policy_throttle is not None else self.throttle_nominal
        )
        raw_throttle = base_throttle if override is None else float(override)
        raw_action = (policy_steer, raw_throttle)
        if self.obs_type == "camera":
            state, ctx, perception_info = self._camera_cage_state(timestamp)
            result = self.cage.step(state, raw_action, ctx)
        else:
            perception_info = {}
            state = build_cage_state(
                lateral_offset=self._perceived_ey,
                heading_error=self.last_track_state.epsi,
                speed=self.ros_interface.get_speed(),
                road_width=self.road_width,
                curvature_ahead=self.tracker.curvature_ahead(
                    self.last_track_state.segment_index, self.lookahead_segments
                ),
                timestamp=timestamp,
            )
            result = self.cage.step(
                state, raw_action, {"current_time": timestamp, "external_stop": False}
            )
        safe_steer, safe_throttle = result["safe_action"]
        emergency = bool(result["emergency"])
        # In monitoring the cage observes only: the raw action is applied and a
        # *shadow* emergency must not brake the robot — that is the no-cage
        # counterfactual the efficacy study needs. The controlled stop is actuated
        # only when enforcing. The shadow flag is still recorded in cage_emergency
        # so the analysis can see "the cage would have stopped here".
        apply_emergency = emergency and (self.cage_mode == "enforcement")
        if policy_throttle is not None:
            # 2-D actuation map (D-50): the cage's safe throttle scales speed
            # linearly up to max_speed (full stop below the deadband) — the
            # cage has speed authority all the way to zero.
            cmd_linear, cmd_angular = safe_action_to_cmd_2d(
                safe_steer,
                safe_throttle,
                apply_emergency,
                max_speed=self.max_speed,
                throttle_deadband=self.throttle_deadband,
                yaw_gain=self.yaw_gain,
            )
        else:
            cmd_linear, cmd_angular = safe_action_to_cmd(
                safe_steer,
                safe_throttle,
                apply_emergency,
                fixed_speed=self.fixed_speed,
                throttle_nominal=self.throttle_nominal,
                min_speed_scale=self.min_speed_scale,
                yaw_gain=self.yaw_gain,
            )
        info = {
            "cage_enabled": True,
            "cage_emergency": emergency,
            "cage_interventions": [iv["rule"] for iv in result["interventions"]],
            # SR-010 (SC-EDGE-05) co-activation diagnostics: the cage's per-cycle
            # joint-envelope assertion + oscillation-persistence flags, surfaced so
            # eval_policy can count assertion failures / oscillation events per run.
            "cage_joint_envelope_violated": bool(result.get("joint_envelope_violated", False)),
            "cage_oscillation_persistent": bool(result.get("oscillation_persistent", False)),
            "raw_steer": policy_steer,
            "safe_steer": float(safe_steer),
            "steer_correction": float(safe_steer) - policy_steer,
            # Throttle stream diagnostics (2-D: policy vs cage arbitration;
            # 1-D: nominal/override vs cage attenuation — C-04's M-S2 signal).
            "raw_throttle": float(raw_throttle),
            "safe_throttle": float(safe_throttle),
            "throttle_correction": float(safe_throttle) - float(raw_throttle),
        }
        info.update(perception_info)
        return float(safe_steer), cmd_linear, cmd_angular, info

    def _camera_cage_state(self, timestamp: float):
        """Cage state from the CV lane-estimator (D-43) for the current cycle.

        Runs the perception supervisor on the last cycle's (degraded) native
        frame. A trustworthy estimate yields a cage `State` stamped with the
        frame's age (so C-05's staleness trigger keeps working in episode
        time); otherwise the cage gets ``state=None`` (its missing-state path)
        plus the ``perception_invalid`` flag for Trigger 8 once the supervisor's
        persistence elapses.
        """
        # Sample the freshest frame for the cage's cycle (through the same
        # degradation pipeline as the policy obs). Using the frame retained at
        # the end of the previous step left the cage ~1 control cycle behind,
        # which tripped both the supervisor's staleness budget and C-05
        # Trigger 3 at real-time frame rates (found live at E2).
        self._capture_camera_obs()
        speed = self.ros_interface.get_speed()
        now_sim = self.ros_interface.sim_now()
        if now_sim is None:
            now_sim = self._cam_stamp
        result = self.cage_perception.update(
            self._cam_frame,
            frame_timestamp_s=self._cam_stamp,
            now_s=float(now_sim),
            speed_mps=speed,
        )
        ctx = {
            "current_time": timestamp,
            "external_stop": False,
            "perception_invalid": result.perception_invalid,
        }
        est = result.estimate
        perception_info = {
            "cv_ok": bool(est.ok),
            "cv_ey": float(est.ey),
            "cv_epsi": float(est.epsi),
            "cv_lane_width": float(est.lane_width),
            "cv_curvature": float(est.curvature),
            "cv_confidence": float(est.confidence),
            "cv_n_lines": int(est.n_lines),
            "cv_reason": str(est.reason),
            "cv_state_available": bool(result.state_available),
            "cv_perception_invalid": bool(result.perception_invalid),
            "cv_health_reason": result.health_reason,
            "cv_plausibility_reason": result.plausibility_reason,
        }
        if not result.state_available:
            return None, ctx, perception_info
        frame_age = max(0.0, float(now_sim) - float(self._cam_stamp))
        state = build_cage_state(
            lateral_offset=est.ey,
            heading_error=est.epsi,
            speed=speed,
            road_width=self.road_width,
            curvature_ahead=est.curvature,
            timestamp=timestamp - frame_age,
        )
        return state, ctx, perception_info

    def _delayed_command(self, cmd_linear: float, cmd_angular: float):
        """Actuation latency (SC-PERT-02): return the command issued
        ``latency_steps`` cycles ago, buffering the current one. Until the buffer
        fills (start-of-episode latency) the actuator is idle. Identity (no-op)
        when the active perturbation carries no latency."""
        steps = self._perturbation.latency_steps
        if steps <= 0:
            return cmd_linear, cmd_angular
        self._cmd_delay.append((cmd_linear, cmd_angular))
        if len(self._cmd_delay) > steps:
            return self._cmd_delay.popleft()
        return 0.0, 0.0

    def close(self) -> None:
        """Best-effort stop command; the ROS interface itself is owned by the caller."""
        try:
            self.ros_interface.send_action(0.0, 0.0)
            self.ros_interface.step_ros(0.05)
        except RuntimeError:
            pass

    def _compute_track_state(self) -> TrackState:
        """Project the current ground-truth pose onto the centerline."""
        x, y, yaw = self.ros_interface.get_pose()
        # Cache the world pose so _make_info can expose it (x, y for the §7.5
        # trajectory plots; the cage/Frenet state is in ey/epsi/s).
        self._last_pose = (float(x), float(y), float(yaw))
        track_state = self.tracker.track(x, y, yaw)
        # Perceived lateral offset = true + sensor noise (SC-PERT-01), or the true
        # value for an unperturbed run. Fed to the policy observation and the cage
        # state; the true `track_state.ey` still drives metrics, reward and
        # termination (the verdict is on the true pose, Ch.8 §8.2.3).
        self._perceived_ey = self._perturbation.perceive_lateral(
            track_state.ey, self.np_random
        )
        return track_state

    @staticmethod
    def _make_observation(
        ey: float,
        epsi: float,
        speed: float,
        previous_steer: float,
        kappa_near: float,
        kappa_far: float,
    ) -> np.ndarray:
        # `ey` is the *perceived* lateral offset (true + sensor noise under
        # SC-PERT-01; true otherwise); `epsi` is the true heading error (the noise
        # channel is lateral_offset only).
        return np.array(
            [
                ey,
                epsi,
                speed,
                previous_steer,
                kappa_near,
                kappa_far,
            ],
            dtype=np.float32,
        )

    def _make_info(self, track_state: TrackState, speed: float) -> Dict[str, float]:
        x, y, yaw = self._last_pose
        sim_time = self.ros_interface.sim_now()
        info = {
            "sim_time_s": float(sim_time) if sim_time is not None else math.nan,
            "ey": float(track_state.ey),
            "epsi": float(track_state.epsi),
            "s": float(track_state.s),
            "speed": float(speed),
            # World pose (for the §7.5 trajectory overlay RL vs PD on the oval).
            "x": float(x),
            "y": float(y),
            "yaw": float(yaw),
        }
        if self._multi_circuit:
            info["circuit_index"] = self._active_circuit
            info["circuit_name"] = self._circuits[self._active_circuit]["name"]
        return info
