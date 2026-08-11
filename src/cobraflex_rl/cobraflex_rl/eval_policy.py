"""
eval_policy — evaluate a trained PPO lane-following policy and emit the §7.5
evidence (Training Specification §7.5.1): completed laps, cage intervention
rate, emergencies, and mean tracking error, written as a reproducible run under
`experiments/sim/runs/<run_id>/`.

The policy is evaluated through `GazeboLaneEnv`, so it runs under the **same**
in-process cage as training and deployment (D-34). Spawn perturbation is
disabled so the start is deterministic and comparable with the PD baseline run.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
import numpy as np
import rclpy
from rclpy.utilities import remove_ros_args
from stable_baselines3 import PPO, SAC
import yaml

from .campaign_metrics import compute_run_metrics
from .campaign_contract import (
    campaign_contract_fingerprint,
    validate_checkpoint_campaign_contract,
)
from .criterion_eval import evaluate as evaluate_criterion
from .criterion_eval import evaluate_labelled_arm
from .criterion_eval import is_labelled, labelled_arms
from .eval_metrics import summarize_eval
from .gazebo_lane_env import GazeboLaneEnv
from .ros_interface import RosGazeboInterface
from .run_io import git_commit, sha256_file
from .scenario_metrics import (
    max_excursion_m,
    road_edge_contact,
    time_to_recovery_heading,
)
from .scenario_runner import derive_run_config


PACKAGE_NAME = "cobraflex_rl"
SCENARIO_ID = "SC-NOM-01"


def resolve_share_directory() -> Path:
    """Installed package share dir, or the source tree when not colcon-built."""
    try:
        return Path(get_package_share_directory(PACKAGE_NAME))
    except PackageNotFoundError:
        return Path(__file__).resolve().parents[1]


def resolve_runs_dir(explicit: Optional[str]) -> Path:
    """Run-output root: explicit override or the repo's experiments/sim/runs."""
    if explicit:
        return Path(explicit)
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "experiments" / "sim" / "runs").is_dir():
            return parent / "experiments" / "sim" / "runs"
    return Path.cwd() / "experiments" / "sim" / "runs"


def load_yaml(path: Path) -> Dict[str, Any]:
    """Parse a YAML file into a dict."""
    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def parse_args(args: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """Parse CLI args, dropping the ROS-specific ones ros2 launch appends."""
    parser = argparse.ArgumentParser(description="Evaluate PPO lane-following policy.")
    parser.add_argument("--centerline-config", type=str, default=None)
    parser.add_argument("--road-centerline-config", type=str, default=None,
                        help="Road-centre centerline YAML for off-road geometry "
                             "(complex_b; see docs/11 §3.5).")
    parser.add_argument("--world-name", type=str, default=None,
                        help="Gazebo SDF world name for gz teleport services "
                             "(e.g. lane_following_complex_b).")
    parser.add_argument("--train-config", type=str, default=None)
    parser.add_argument("--model-path", type=str, default=None)
    parser.add_argument(
        "--scripted-stall", action="store_true",
        help="SC-PERT-03 metrology stimulus (D-64): ignore the policy and command "
             "a fixed full-stop action [steer=0, throttle=-1] every step, so the "
             "vehicle is a ground-truth stalled car. Verifies SR-009's stall "
             "detector (M-P6) fires on a genuine stall without training an "
             "adversary. The checkpoint is loaded only for the env/action-space "
             "machinery; its actions are overridden. Default off — verdict runs "
             "are bit-identical.",
    )
    parser.add_argument(
        "--algorithm", type=str, default=None, choices=["ppo", "sac"],
        help="SB3 class the checkpoint was trained with. Default: the "
             "train config's `algorithm` key (ppo when absent).",
    )
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument(
        "--max-steps", type=int, default=None,
        help="Override max_episode_steps for the eval (e.g. 4400 ≈ 10 continuous "
             "laps for the §7.5.1 lap-count comparison vs the PD). Default: the "
             "value in train_ppo.yaml.",
    )
    parser.add_argument("--run-id", type=str, default=None)
    parser.add_argument("--output-root", type=str, default=None,
                        help="Directory holding run subdirs (default experiments/sim/runs).")
    # F4 scenario-campaign mode. With --scenario the run is driven by an SC-*
    # YAML (initial conditions, commanded speed, timeout) instead of the nominal
    # SC-NOM-01 defaults, and the output carries the full per-run metric
    # catalogue + the pass_criterion verdict.
    parser.add_argument("--scenario", type=str, default=None,
                        help="Path to a scenario YAML (scenarios/<cat>/*.yaml). "
                             "Omit for the legacy SC-NOM-01 §7.5 eval.")
    parser.add_argument("--mode", type=str, default=None,
                        choices=["enforcement", "monitoring"],
                        help="Cage mode override (default: train_ppo.yaml cage.mode).")
    parser.add_argument("--rep", type=int, default=0,
                        help="Repetition index — seeds the per-run randomisation jitter.")
    parser.add_argument("--base-seed", type=int, default=0,
                        help="Base seed folded into the per-rep jitter (reproducibility).")
    parser.add_argument(
        "--criterion-arm", type=str, default=None,
        help="Label of one preregistered multi-arm criterion (SC-PERT-03: "
             "released|stall_variant). Omit for ordinary scenarios.",
    )
    parser.add_argument(
        "--protocol-manifest", type=str, default=None,
        help="Completed two-arm protocol manifest; its path/hash are recorded "
             "with every SC-PERT-03 run.",
    )
    cleaned_args = remove_ros_args(args=args)
    if cleaned_args and not cleaned_args[0].startswith("-"):
        cleaned_args = cleaned_args[1:]
    return parser.parse_args(cleaned_args)


def resolve_load_path(path: Path) -> Path:
    """The checkpoint file to load (append .zip if the bare path doesn't exist)."""
    if path.exists():
        return path
    return path if path.suffix == ".zip" else path.with_suffix(".zip")


def _disable_spawn_perturbation(train_cfg: Dict[str, Any]) -> None:
    """Force a deterministic spawn for eval (random spawn is training-only).

    Two independent spawn-randomisers must both be silenced: ``enabled`` gates
    the heading/lateral jitter (``_perturbed_spawn``), while ``random_start_s``
    (D-58, the 2-D curriculum lever) is read by the env *independently* of that
    flag and, on the nominal path (no scenario ⇒ ``start_s`` is None), draws a
    uniform along-track spawn. Left on, a run labelled SC-NOM-01 would start at a
    random arc-length — silently non-reproducible and not comparable to the frozen
    *_4k4 nominal evals. Scenario/campaign runs are already safe (they always pass
    an explicit ``start_s_m``), but zero it here so every eval path is deterministic.
    """
    spawn_cfg = dict(train_cfg.get("spawn_perturbation", {}))
    spawn_cfg["enabled"] = False
    spawn_cfg["random_start_s"] = False
    train_cfg["spawn_perturbation"] = spawn_cfg


def _record_from_info(episode: int, step: int, info: Dict[str, Any]) -> Dict[str, Any]:
    """Flatten one env-step info dict into the per-step run record."""
    return {
        "episode": episode,
        "step": step,
        "sim_time_s": float(info.get("sim_time_s", math.nan)),
        # World pose (x, y) → §7.5 trajectory overlay (RL vs PD on the oval);
        # ey/epsi/s are the Frenet/cage state at the same step.
        "x": float(info.get("x", 0.0)),
        "y": float(info.get("y", 0.0)),
        "yaw": float(info.get("yaw", 0.0)),
        "ey": float(info.get("ey", 0.0)),
        "epsi": float(info.get("epsi", 0.0)),
        "s": float(info.get("s", 0.0)),
        "speed": float(info.get("speed", 0.0)),
        "raw_steer": float(info.get("raw_steer", 0.0)),
        "safe_steer": float(info.get("safe_steer", 0.0)),
        "steer_correction": float(info.get("steer_correction", 0.0)),
        # Longitudinal (throttle) channel — the 2-D action's new evidence: the
        # cage's speed rules (C-04 attenuation, C-05-B, C-06 rate limit) arbitrate
        # the policy's throttle, so the correction magnitude is the SR-CL / SR-009
        # signal. Present as 0-correction on the frozen 1-D path (throttle is the
        # fixed nominal there), so the schema stays backward-compatible.
        "raw_throttle": float(info.get("raw_throttle", 0.0)),
        "safe_throttle": float(info.get("safe_throttle", 0.0)),
        "throttle_correction": float(info.get("throttle_correction", 0.0)),
        "interventions": list(info.get("cage_interventions", [])),
        "emergency": bool(info.get("cage_emergency", False)),
        # SR-010 / SC-EDGE-05 per-cycle cage co-activation flags (0/False on every
        # other scenario): aggregated into the per-run assertion/oscillation counters.
        "joint_envelope_violated": bool(info.get("cage_joint_envelope_violated", False)),
        "oscillation_persistent": bool(info.get("cage_oscillation_persistent", False)),
        # Track-'E' perception diagnostics (0/absent on the state-vector track):
        # the SC-PERT-07/08 verdicts need the Trigger-8 / availability trace.
        "cv_ok": bool(info.get("cv_ok", False)),
        "cv_state_available": bool(info.get("cv_state_available", False)),
        "cv_perception_invalid": bool(info.get("cv_perception_invalid", False)),
        "cv_ey": float(info.get("cv_ey", 0.0)),
        "cv_epsi": float(info.get("cv_epsi", 0.0)),
        "cv_confidence": float(info.get("cv_confidence", 0.0)),
        # Curve diagnostics: curvature (0 ⇒ no quadratic fit, cluster span too
        # short), lane width, line count, and the estimate's reason
        # ("ok" full pair vs "single_line" fallback) to localise curve misreads.
        "cv_curvature": float(info.get("cv_curvature", 0.0)),
        "cv_lane_width": float(info.get("cv_lane_width", 0.0)),
        "cv_n_lines": int(info.get("cv_n_lines", 0)),
        "cv_reason": str(info.get("cv_reason", "")),
    }


def _write_cage_status_csv(path: Path, records: List[Dict[str, Any]]) -> None:
    """Write the per-step trace CSV (one row per control cycle)."""
    fields = [
        "episode", "step", "sim_time_s", "x", "y", "yaw", "ey", "epsi", "s", "speed",
        "raw_steer", "safe_steer", "steer_correction",
        "raw_throttle", "safe_throttle", "throttle_correction",
        "interventions", "emergency",
        "joint_envelope_violated", "oscillation_persistent",
        "cv_ok", "cv_state_available", "cv_perception_invalid",
        "cv_ey", "cv_epsi", "cv_confidence",
        "cv_curvature", "cv_lane_width", "cv_n_lines", "cv_reason",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(fields)
        for r in records:
            writer.writerow([
                r["episode"], r["step"], f"{r.get('sim_time_s', math.nan):.6f}",
                f"{r['x']:.6f}", f"{r['y']:.6f}", f"{r['yaw']:.6f}",
                f"{r['ey']:.6f}", f"{r['epsi']:.6f}", f"{r['s']:.6f}",
                f"{r['speed']:.6f}", f"{r['raw_steer']:.6f}",
                f"{r['safe_steer']:.6f}", f"{r['steer_correction']:.6f}",
                f"{r.get('raw_throttle', 0.0):.6f}",
                f"{r.get('safe_throttle', 0.0):.6f}",
                f"{r.get('throttle_correction', 0.0):.6f}",
                ";".join(r["interventions"]), int(r["emergency"]),
                int(r.get("joint_envelope_violated", False)),
                int(r.get("oscillation_persistent", False)),
                int(r.get("cv_ok", False)), int(r.get("cv_state_available", False)),
                int(r.get("cv_perception_invalid", False)),
                f"{r.get('cv_ey', 0.0):.6f}", f"{r.get('cv_epsi', 0.0):.6f}",
                f"{r.get('cv_confidence', 0.0):.4f}",
                f"{r.get('cv_curvature', 0.0):.4f}", f"{r.get('cv_lane_width', 0.0):.4f}",
                int(r.get("cv_n_lines", 0)), r.get("cv_reason", ""),
            ])


class _FrameStacker:
    """Mirror of VecFrameStack for the single-env eval loop (camera mode):
    keeps the last k frames and concatenates along the channel axis, newest
    last — the same layout the policy saw in training."""

    def __init__(self, k: int):
        self.k = int(k)
        self._frames: List[np.ndarray] = []

    def reset(self, frame: np.ndarray) -> np.ndarray:
        self._frames = [frame] * self.k
        return np.concatenate(self._frames, axis=-1)

    def step(self, frame: np.ndarray) -> np.ndarray:
        self._frames = (self._frames + [frame])[-self.k:]
        return np.concatenate(self._frames, axis=-1)


def _print_summary(summary: Dict[str, Any]) -> None:
    """Console summary of the §7.5 evaluation metrics."""
    print("\n===== §7.5 evaluation summary (RL + cage) =====")
    print(f"  episodes / steps    : {summary['episodes']} / {summary['total_steps']}")
    print(f"  completed laps      : {summary['completed_laps']:.2f}")
    print(f"  cage emergencies    : {summary['emergency_steps']} steps")
    print(f"  cage interventions  : {summary['intervention_rate_pct']:.3f}% of steps "
          f"({summary['interventions_per_rule']})")
    print(f"  mean ey / |ey|      : {summary['mean_ey_m']:.4f} / {summary['mean_abs_ey_m']:.4f} m")
    print(f"  mean |epsi|         : {summary['mean_abs_epsi_rad']:.4f} rad")
    print(f"  duration            : {summary['duration_s']:.1f} s")
    print("===============================================\n")


def main(args: Optional[Sequence[str]] = None) -> None:
    """Run one evaluation (legacy §7.5 or F4 scenario mode) and write the run dir."""
    cli_args = parse_args(args)
    rclpy.init(args=args)

    share_dir = resolve_share_directory()
    centerline_path = Path(cli_args.centerline_config or share_dir / "config" / "oval_right_lane_centerline.yaml")
    train_cfg_path = Path(cli_args.train_config or share_dir / "config" / "train_ppo.yaml")

    centerline_cfg = load_yaml(centerline_path)
    train_cfg = load_yaml(train_cfg_path)
    # Validate opted-in posterior contracts before starting an evaluation. The
    # historical configs omit this block and remain unchanged.
    campaign_contract_fingerprint(train_cfg)
    road_centerline_points = None
    if cli_args.road_centerline_config:
        road_centerline_points = np.asarray(
            load_yaml(Path(cli_args.road_centerline_config))["centerline"]["points"],
            dtype=float,
        )
    # Evaluation starts from the deterministic scenario/nominal spawn (random
    # spawn perturbation off); the F4 scenario path injects its initial
    # conditions through reset(options=...) instead (§7.5, D-34).
    _disable_spawn_perturbation(train_cfg)
    # Same determinism rule for the camera track: H-10 visual domain
    # randomisation is a TRAINING-side mitigation (SR-012). In eval the only
    # visual stressor is the scenario's own perturbations block — otherwise a
    # nominal eval episode can draw a random degradation from the training
    # envelope (and a harsh draw blinds the CV estimator at spawn → instant
    # no-state-ever emergency, observed on rl_cam_eval_2024_*).
    if isinstance(train_cfg.get("domain_randomization"), dict):
        train_cfg["domain_randomization"] = {
            **train_cfg["domain_randomization"], "enabled": False,
        }

    # F4 scenario mode: derive the run config (initial conditions, commanded
    # speed, horizon) from the SC-* YAML for this repetition. Absent --scenario,
    # the legacy SC-NOM-01 §7.5 eval runs unchanged.
    run_config = None
    scenario_id = SCENARIO_ID
    reset_options: Optional[Dict[str, Any]] = None
    if cli_args.scenario:
        scenario_cfg = load_yaml(Path(cli_args.scenario))
        control_dt = float(train_cfg.get("control_dt", 0.1))
        run_config = derive_run_config(
            scenario_cfg, cli_args.rep, control_dt=control_dt, base_seed=cli_args.base_seed
        )
        scenario_id = run_config.scenario_id
        reset_options = {**run_config.reset_options, "perturbation": run_config.perturbation}
        train_cfg["fixed_speed"] = run_config.fixed_speed
        if not (cli_args.max_steps and cli_args.max_steps > 0) and run_config.max_steps > 0:
            train_cfg["max_episode_steps"] = run_config.max_steps

    # Cage mode override (enforcement vs monitoring campaign axis).
    if cli_args.mode:
        cage_cfg = dict(train_cfg.get("cage", {}))
        cage_cfg["mode"] = cli_args.mode
        train_cfg["cage"] = cage_cfg

    # Explicit --max-steps always wins (e.g. the §7.5.1 long lap-count run).
    if cli_args.max_steps is not None and cli_args.max_steps > 0:
        train_cfg["max_episode_steps"] = int(cli_args.max_steps)

    model_path = Path(cli_args.model_path or train_cfg.get("model_path", "cobraflex_ppo_lane"))
    model_path = model_path.expanduser()
    seed = int(train_cfg.get("seed", 42))
    # Env reset seed: in scenario mode use the per-rep env_seed so the obs-noise
    # stream (SC-PERT-01) is independent across reps yet reproducible; otherwise
    # the legacy policy seed (§7.5 eval).
    reset_seed = run_config.env_seed if run_config is not None else seed

    interface: Optional[RosGazeboInterface] = None
    env: Optional[GazeboLaneEnv] = None
    records: List[Dict[str, Any]] = []

    # Track 'E': camera observation mode mirrors train_ppo — the interface
    # subscribes to the image topic and the eval loop applies the same frame
    # stack the policy was trained with (VecFrameStack equivalent).
    obs_cfg = dict(train_cfg.get("observation", {}))
    camera_obs = str(obs_cfg.get("type", "state")) == "camera"
    cam_cfg = dict(obs_cfg.get("camera", {})) if camera_obs else {}
    stacker = _FrameStacker(int(cam_cfg.get("frame_stack", 4))) if camera_obs else None

    try:
        interface_kwargs = {
            "camera_topic": str(cam_cfg.get("topic", "/camera/image_raw_lane")) if camera_obs else ""
        }
        if cli_args.world_name:
            interface_kwargs["world_name"] = cli_args.world_name
        interface = RosGazeboInterface(**interface_kwargs)
        if not interface.wait_for_initial_data(timeout_sec=10.0):
            raise RuntimeError("Timed out waiting for /odom data.")

        centerline_points = np.asarray(centerline_cfg["centerline"]["points"], dtype=float)
        lane_width = float(centerline_cfg["lane_width"])
        road_width = float(centerline_cfg.get("road_width", lane_width))

        env = GazeboLaneEnv(
            ros_interface=interface,
            centerline=centerline_points,
            lane_width=lane_width,
            road_width=road_width,
            cfg=train_cfg,
            road_centerline=road_centerline_points,
        )
        perimeter = float(env.tracker.cumulative_lengths[-1])

        # Load on CPU: the lane-following policy is a tiny MLP, CPU inference at
        # 20 Hz is ample, and it avoids GPU VRAM exhaustion across the hundreds of
        # back-to-back runs of a campaign (the "CUDA out of memory" on the 4th
        # consecutive run that defaulting to the GPU caused).
        # The SB3 class must match the checkpoint (PPO.load on a SAC zip fails
        # on missing policy params): --algorithm, else the config's key.
        algorithm = (
            cli_args.algorithm or str(train_cfg.get("algorithm", "ppo"))
        ).lower()
        algo_cls = {"ppo": PPO, "sac": SAC}[algorithm]
        model = algo_cls.load(str(resolve_load_path(model_path)), device="cpu")
        validate_checkpoint_campaign_contract(model, train_cfg)

        # Guard against an action-dim mismatch between the checkpoint and the
        # --train-config. The 1-D and 2-D camera configs share an identical
        # (84,84,4) observation space, so SB3 does NOT catch a mispairing: a 2-D
        # [steer, throttle] checkpoint evaluated under the 1-D config would have
        # its throttle element silently dropped (gazebo_lane_env only reads
        # action_vec[0] on the 1-D path) and be scored as a 1-D policy driving at
        # the wrong fixed speed. Fail loudly instead — pass the matching config.
        model_act = getattr(model.action_space, "shape", None)
        env_act = getattr(env.action_space, "shape", None)
        if model_act != env_act:
            raise RuntimeError(
                f"Action-space mismatch: checkpoint action_space {model_act} != "
                f"env action_space {env_act} (from --train-config "
                f"{train_cfg_path.name}, action.type "
                f"{train_cfg.get('action', {}).get('type', 'steer')!r}). A 2-D "
                f"steer_throttle checkpoint must be evaluated with a "
                f"steer_throttle config (train_ppo_camera_2d.yaml), and vice versa."
            )

        last_terminated = False
        last_info: Dict[str, Any] = {}
        for episode in range(cli_args.episodes):
            observation, info = env.reset(seed=reset_seed + episode, options=reset_options)
            if stacker is not None:
                observation = stacker.reset(observation)
            terminated = False
            truncated = False
            step_index = 0
            print(f"Episode {episode + 1}")

            while not (terminated or truncated):
                if cli_args.scripted_stall:
                    # Ground-truth stall stimulus (D-64): full stop every step
                    # (throttle -1 → u=0 < deadband → speed 0). Policy ignored.
                    action = np.array([0.0, -1.0], dtype=np.float32)
                else:
                    action, _ = model.predict(observation, deterministic=True)
                observation, reward, terminated, truncated, info = env.step(action)
                if stacker is not None:
                    observation = stacker.step(observation)
                step_index += 1
                records.append(_record_from_info(episode, step_index, info))
                print(
                    f"step={step_index:04d} ey={info['ey']:.4f} "
                    f"epsi={info['epsi']:.4f} speed={info['speed']:.4f} "
                    f"reward={reward:.4f} interv={info.get('cage_interventions', [])}"
                )
            last_terminated = terminated
            last_info = info

        summary = summarize_eval(records, perimeter=perimeter, cycle_dt_s=env.control_dt)
        if cli_args.scripted_stall:
            # Honest provenance: actions were the scripted full-stop, not the
            # loaded checkpoint (which served only the env/action-space machinery).
            summary["scripted_stall"] = True
        if run_config is not None and run_config.perturbation.active:
            # Record which perturbation level this rep ran (σ / latency / pulse) so
            # the per-run summary.json is self-describing for the analysis.
            summary["perturbation"] = {
                "kind": run_config.perturbation.kind,
                "level": run_config.perturbation.level_label,
            }
        _print_summary(summary)
        # `completed` (M-P2): the run reached its horizon / goal without a
        # road-boundary termination. Used only in the scenario verdict.
        campaign = None
        if run_config is not None:
            campaign = _evaluate_scenario(
                records, run_config, completed=not last_terminated,
                control_dt=env.control_dt, road_half_m=0.5 * env.road_width,
                termination_reason=str(last_info.get("termination_reason", "")),
                criterion_arm=cli_args.criterion_arm,
            )
            _print_verdict(scenario_id, campaign)
        _write_run(
            cli_args=cli_args,
            train_cfg=train_cfg,
            centerline_path=centerline_path,
            model_path=resolve_load_path(model_path),
            seed=seed,
            records=records,
            summary=summary,
            scenario_id=scenario_id,
            campaign=campaign,
        )
    finally:
        if env is not None:
            env.close()
        if interface is not None:
            interface.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _jsonable(value: Any) -> Any:
    """JSON-safe copy: non-finite floats (e.g. time_to_recovery_heading == inf
    when the heading never recovers) -> None, recursively through dicts/lists."""
    import math as _math
    if isinstance(value, float):
        return value if _math.isfinite(value) else None
    if isinstance(value, dict):
        return {k: _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    return value


def _evaluate_scenario(
    records: List[Dict[str, Any]],
    run_config: Any,
    *,
    completed: bool,
    control_dt: float,
    road_half_m: Optional[float] = None,
    termination_reason: str = "",
    criterion_arm: Optional[str] = None,
) -> Dict[str, Any]:
    """Compute the full per-run metric catalogue + scenario-specific metrics and
    score the scenario's ``pass_criterion_per_run`` (three-valued; an unavailable
    metric is 'indeterminate', a real breach is a fail)."""
    res = compute_run_metrics(records, params={"control_dt": control_dt}, completed=completed)
    metrics = res["metrics"]
    values: Dict[str, Any] = dict(metrics)
    values["emergency"] = any(bool(r.get("emergency")) for r in records)
    values["time_to_recovery_heading"] = time_to_recovery_heading(records, control_dt=control_dt)
    # Frontier / out-of-ODD tokens (cage-efficacy contrast): how far out it got
    # and whether it reached the road edge (the harm proxy).
    values["max_excursion_m"] = max_excursion_m(records)
    values["road_edge_contact"] = (
        road_edge_contact(records, road_half_m) if road_half_m else None
    )
    # SR-010 / SC-EDGE-05 co-activation counters from the per-cycle cage flags.
    # joint_envelope_assertion_failures = #cycles the joint-envelope predicate was
    # violated; inter_cycle_oscillations = #distinct oscillation events (rising
    # edges of the cage's persistent-oscillation flag). Both 0 on every non-grid
    # scenario (the flags are False there), so they are inert in other criteria.
    values["joint_envelope_assertion_failures"] = sum(
        1 for r in records if r.get("joint_envelope_violated")
    )
    values["inter_cycle_oscillations"] = sum(
        1 for i, r in enumerate(records)
        if r.get("oscillation_persistent")
        and not (i > 0 and records[i - 1].get("oscillation_persistent"))
    )

    expr = run_config.pass_criterion_per_run
    pert = getattr(run_config, "perturbation", None)
    if is_labelled(expr) and criterion_arm:
        result = evaluate_labelled_arm(expr, criterion_arm, values)
        verdict, clauses = result["passed"], result["clauses"]
        note = f"preregistered criterion arm '{criterion_arm}'"
        if result.get("error"):
            note += f"; {result['error']}"
    elif is_labelled(expr) and pert is not None and pert.active:
        # Level-resolved labelled criterion (e.g. SC-PERT-05 low/high): this rep
        # ran ONE perturbation level, so score the matching arm only. Arm order
        # follows the YAML, which matches the scenario's level_levels order.
        arms = labelled_arms(expr)
        label, sub = arms[pert.level_index % len(arms)]
        result = evaluate_criterion(sub, values)
        verdict, clauses = result["passed"], result["clauses"]
        note = f"labelled arm '{label}' (level {pert.level_label or pert.level_index})"
        if result.get("error"):
            note += f"; {result['error']}"
    elif is_labelled(expr):
        # Labelled but not level-resolvable in a single run (e.g. SC-PERT-03's
        # finetune arms): the campaign driver runs/groups the arms separately.
        verdict, clauses, note = None, [], (
            "labelled multi-arm criterion not level-resolvable in single-run eval "
            "(grouped in the driver)"
        )
    else:
        result = evaluate_criterion(expr, values)
        verdict, clauses, note = result["passed"], result["clauses"], result.get("error")

    return {
        "scenario_id": run_config.scenario_id,
        "criterion": expr,
        # SC-EDGE-05 only: the co-activation grid point this rep ran (anchor id,
        # expected cage rules, bracket factor, scaled seed) so the SR-010 analysis
        # can compare expected vs observed co-activation. None for every other scenario.
        "grid_point": getattr(run_config, "grid_point", None),
        "verdict": verdict,           # True / False / None (indeterminate)
        "clauses": _jsonable(clauses),
        "values": _jsonable(values),
        "availability": res["availability"],
        "termination_reason": termination_reason,
        "note": note,
    }


def _print_verdict(scenario_id: str, campaign: Dict[str, Any]) -> None:
    """Console rendering of the per-run scenario verdict and its clauses."""
    label = {True: "PASS", False: "FAIL", None: "INDETERMINATE"}[campaign["verdict"]]
    print(f"\n===== {scenario_id} per-run verdict: {label} =====")
    print(f"  criterion: {campaign['criterion']}")
    for c in campaign["clauses"]:
        mark = {True: "ok ", False: "FAIL", None: "n/a "}.get(c.get("passed"))
        print(f"   [{mark}] {c.get('clause')}  (lhs={c.get('lhs')})")
    vals = campaign.get("values", {})
    if campaign.get("termination_reason") or vals.get("max_excursion_m") is not None:
        print(f"  termination={campaign.get('termination_reason')!r}  "
              f"max_excursion={vals.get('max_excursion_m')} m  "
              f"road_edge_contact={vals.get('road_edge_contact')}")
    if campaign.get("note"):
        print(f"  note: {campaign['note']}")
    print("=" * 47 + "\n")


def _write_run(
    cli_args: argparse.Namespace,
    train_cfg: Dict[str, Any],
    centerline_path: Path,
    model_path: Path,
    seed: int,
    records: List[Dict[str, Any]],
    summary: Dict[str, Any],
    scenario_id: str = SCENARIO_ID,
    campaign: Optional[Dict[str, Any]] = None,
) -> None:
    """Persist the run dir: trace CSV, summary.json and reproducibility metadata."""
    runs_dir = resolve_runs_dir(cli_args.output_root)
    run_id = cli_args.run_id or "rl_eval_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_dir = runs_dir / run_id
    out_dir.mkdir(parents=True, exist_ok=True)

    mode = str(train_cfg.get("cage", {}).get("mode", "enforcement"))
    cage_yaml = Path(train_cfg.get("cage", {}).get("yaml_path", "") or "")
    if not cage_yaml.is_file():
        for parent in Path(__file__).resolve().parents:
            candidate = parent / "cage" / "cage.yaml"
            if candidate.is_file():
                cage_yaml = candidate
                break

    _write_cage_status_csv(out_dir / "cage_status.csv", records)

    summary_out = dict(summary)
    summary_out.update({"run_id": run_id, "scenario_id": scenario_id, "mode": mode})
    if getattr(cli_args, "criterion_arm", None):
        summary_out["criterion_arm"] = cli_args.criterion_arm
    if campaign is not None:
        summary_out["verdict"] = campaign["verdict"]
        summary_out["campaign"] = campaign
    with (out_dir / "summary.json").open("w", encoding="utf-8") as handle:
        json.dump(summary_out, handle, indent=2)

    train_cfg_path = Path(
        cli_args.train_config
        or resolve_share_directory() / "config" / "train_ppo.yaml"
    )
    metadata = {
        "run_id": run_id,
        "scenario_id": scenario_id,
        "mode": mode,
        "timestamp_iso": datetime.now(timezone.utc).isoformat(),
        "duration_s": summary["duration_s"],
        "git_commit": git_commit(out_dir),
        "train_config": str(train_cfg_path),
        "train_config_hash": sha256_file(train_cfg_path),
        "cage_yaml": str(cage_yaml),
        "cage_yaml_hash": sha256_file(cage_yaml),
        "policy_checkpoint": str(model_path),
        "policy_checkpoint_hash": sha256_file(model_path),
        "centerline_yaml_hash": sha256_file(centerline_path),
        "scenario_yaml": str(cli_args.scenario or ""),
        "scenario_yaml_hash": sha256_file(Path(cli_args.scenario)) if cli_args.scenario else sha256_file(centerline_path),
        "criterion_arm": getattr(cli_args, "criterion_arm", None),
        "protocol_manifest": str(getattr(cli_args, "protocol_manifest", "") or ""),
        "protocol_manifest_hash": (
            sha256_file(Path(cli_args.protocol_manifest))
            if getattr(cli_args, "protocol_manifest", None) else None
        ),
        "rep": int(getattr(cli_args, "rep", 0)),
        "seed": seed,
        "algorithm": str(
            cli_args.algorithm or train_cfg.get("algorithm", "ppo")
        ).lower(),
        "action": dict(train_cfg.get("action", {})),
        "campaign_contract": campaign_contract_fingerprint(train_cfg),
        "platform": "sim",
        "status": "completed",
    }
    with (out_dir / "metadata.json").open("w", encoding="utf-8") as handle:
        json.dump(metadata, handle, indent=2)

    print(f"Wrote evaluation run to {out_dir}")


if __name__ == "__main__":
    main()
