"""
eval_cv_controller — evaluate the **logical camera lane-keeper** (the shared
:class:`cobraflex_rl.cv_lane_controller.CVLaneController` that also drives
cobraflex/lane_keeper_gazebo_node), through the exact same `GazeboLaneEnv` +
scoring as `eval_policy`.

Why: the RL camera agent (track 'E') must be compared against a *camera*
baseline, not the ground-truth PD (comparing a real-perception policy to a
PD that reads perfect state is meaningless). This driver runs the same CV
front-end the deployment node uses — the deterministic CV lane estimator
(CvLaneEstimator, D-43: HSV white mask → ground-plane row scan → line
clustering → lane-pair selection) plus a pure-pursuit look-ahead law
(aim at the lane centre at X=look_ahead_m; see docs/12 §3) — and feeds its steering to the env
as the action. Since the env publishes ``angular.z = action`` and runs at a
fixed cruise speed, this is exactly what the node would command — but scored
identically to the RL/PD runs (ey/epsi vs the right-lane centreline, completed
laps, cage interventions) for a like-for-like table.

It supersedes the previous histogram pure-P front-end (docs/12 §7). The
controller never sees ground truth: its only input is the env's raw camera
frame (`ros_interface.get_camera_frame()`), the same image stream the RL CNN
and the cage's CV estimator consume.
"""

from __future__ import annotations

import argparse
import json
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from rclpy.utilities import remove_ros_args

from .cv_lane_controller import CVLaneController
from .eval_metrics import summarize_eval
from .eval_policy import (
    SCENARIO_ID,
    _disable_spawn_perturbation,
    _evaluate_scenario,
    _print_summary,
    _print_verdict,
    _record_from_info,
    _write_cage_status_csv,
    load_yaml,
    resolve_runs_dir,
    resolve_share_directory,
)
from .gazebo_lane_env import GazeboLaneEnv
from .ros_interface import RosGazeboInterface
from .run_io import git_commit, sha256_file
from .scenario_runner import derive_run_config


def parse_args(args: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate the logical CV camera lane-keeper.")
    parser.add_argument("--centerline-config", type=str, default=None)
    parser.add_argument("--road-centerline-config", type=str, default=None,
                        help="Road-centre centerline YAML for off-road geometry "
                             "(off_road = global distance > road_width/2). Required "
                             "on self-approaching circuits like complex_b; see "
                             "docs/11 §3.5.")
    parser.add_argument("--world-name", type=str, default=None,
                        help="Gazebo SDF world name for gz teleport services "
                             "(e.g. lane_following_complex_b). Defaults to the "
                             "RosGazeboInterface default (lane_following_oval).")
    parser.add_argument("--train-config", type=str, default=None,
                        help="Env config (default train_ppo_camera.yaml — camera mode).")
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--max-steps", type=int, default=None)
    parser.add_argument("--run-id", type=str, default=None)
    parser.add_argument("--output-root", type=str, default=None)
    parser.add_argument("--scenario", type=str, default=None)
    parser.add_argument("--mode", type=str, default=None, choices=["enforcement", "monitoring"])
    parser.add_argument("--rep", type=int, default=0)
    parser.add_argument("--base-seed", type=int, default=0)
    parser.add_argument("--fixed-speed", type=float, default=None,
                        help="Override the env cruise speed (m/s). The logical "
                             "controller's native speed is 0.10; the RL camera eval "
                             "ran at 0.20, so |ey| is the speed-fair comparison.")
    parser.add_argument("--calibration-heading-injection-rad", type=float, default=0.0,
                        help="Evidence-only moving yaw impulse; requires a "
                             "category=calibration_only scenario.")
    parser.add_argument("--calibration-heading-injection-step", type=int, default=12,
                        help="Control step after which to inject the calibration yaw impulse.")
    cleaned = remove_ros_args(args=args)
    if cleaned and not cleaned[0].startswith("-"):
        cleaned = cleaned[1:]
    return parser.parse_args(cleaned)


def main(args: Optional[Sequence[str]] = None) -> None:
    cli = parse_args(args)
    rclpy.init(args=args)

    share_dir = resolve_share_directory()
    centerline_path = Path(cli.centerline_config or share_dir / "config" / "oval_right_lane_centerline.yaml")
    train_cfg_path = Path(cli.train_config or share_dir / "config" / "train_ppo_camera.yaml")

    centerline_cfg = load_yaml(centerline_path)
    train_cfg = load_yaml(train_cfg_path)
    road_centerline_points = None
    if cli.road_centerline_config:
        road_centerline_points = np.asarray(
            load_yaml(Path(cli.road_centerline_config))["centerline"]["points"], dtype=float
        )
    _disable_spawn_perturbation(train_cfg)
    # Eval determinism: no training-side visual domain randomisation (a harsh
    # draw blinds perception at spawn). Same rule as eval_policy.
    if isinstance(train_cfg.get("domain_randomization"), dict):
        train_cfg["domain_randomization"] = {**train_cfg["domain_randomization"], "enabled": False}

    obs_cfg = dict(train_cfg.get("observation", {}))
    if str(obs_cfg.get("type", "state")) != "camera":
        raise RuntimeError("eval_cv_controller requires a camera-mode train-config "
                           "(observation.type == 'camera').")
    cam_cfg = dict(obs_cfg.get("camera", {}))

    run_config = None
    scenario_id = SCENARIO_ID
    reset_options: Optional[Dict[str, Any]] = None
    if cli.scenario:
        scenario_cfg = load_yaml(Path(cli.scenario))
        if cli.calibration_heading_injection_rad and (
            scenario_cfg.get("category") != "calibration_only"
        ):
            raise RuntimeError(
                "heading injection is restricted to category=calibration_only"
            )
        control_dt = float(train_cfg.get("control_dt", 0.1))
        run_config = derive_run_config(scenario_cfg, cli.rep, control_dt=control_dt, base_seed=cli.base_seed)
        scenario_id = run_config.scenario_id
        reset_options = {**run_config.reset_options, "perturbation": run_config.perturbation}
        train_cfg["fixed_speed"] = run_config.fixed_speed
        if not (cli.max_steps and cli.max_steps > 0) and run_config.max_steps > 0:
            train_cfg["max_episode_steps"] = run_config.max_steps

    if cli.mode:
        cage_cfg = dict(train_cfg.get("cage", {}))
        cage_cfg["mode"] = cli.mode
        train_cfg["cage"] = cage_cfg
    if cli.max_steps is not None and cli.max_steps > 0:
        train_cfg["max_episode_steps"] = int(cli.max_steps)
    # Cruise-speed override wins over both the config and the scenario speed so
    # the controller can be evaluated at its native 0.10 m/s.
    if cli.fixed_speed is not None:
        train_cfg["fixed_speed"] = float(cli.fixed_speed)

    seed = int(train_cfg.get("seed", 42))
    reset_seed = run_config.env_seed if run_config is not None else seed

    interface: Optional[RosGazeboInterface] = None
    env: Optional[GazeboLaneEnv] = None
    records: List[Dict[str, Any]] = []
    no_lane_steps = 0

    try:
        interface_kwargs = {"camera_topic": str(cam_cfg.get("topic", "/camera/image_raw_lane"))}
        if cli.world_name:
            interface_kwargs["world_name"] = cli.world_name
        interface = RosGazeboInterface(**interface_kwargs)
        if not interface.wait_for_initial_data(timeout_sec=10.0):
            raise RuntimeError("Timed out waiting for /odom data.")

        centerline_points = np.asarray(centerline_cfg["centerline"]["points"], dtype=float)
        lane_width = float(centerline_cfg["lane_width"])
        road_width = float(centerline_cfg.get("road_width", lane_width))

        env = GazeboLaneEnv(
            ros_interface=interface, centerline=centerline_points,
            lane_width=lane_width, road_width=road_width, cfg=train_cfg,
            road_centerline=road_centerline_points,
            calibration_mode=bool(cli.calibration_heading_injection_rad),
        )
        perimeter = float(env.tracker.cumulative_lengths[-1])
        controller = CVLaneController(speed=float(train_cfg.get("fixed_speed", 0.2)))

        last_terminated = False
        last_info: Dict[str, Any] = {}
        # Ring buffer of the most recent (step, BGR frame) so a cage-emergency
        # stop can be debugged offline: the frames that drove the misread are
        # dumped next to the run (re-run the estimator on them to localise the
        # curve failure without re-launching Gazebo).
        frame_buffer: Deque[Tuple[int, np.ndarray]] = deque(maxlen=15)
        for episode in range(cli.episodes):
            _obs, info = env.reset(seed=reset_seed + episode, options=reset_options)
            terminated = truncated = False
            step_index = 0
            print(f"Episode {episode + 1}")
            while not (terminated or truncated):
                if (
                    cli.calibration_heading_injection_rad
                    and step_index == cli.calibration_heading_injection_step
                ):
                    injection = env.inject_heading_fault_for_calibration(
                        cli.calibration_heading_injection_rad
                    )
                    print(f"calibration heading injection: {injection}")
                frame_res = interface.get_camera_frame()
                frame = frame_res[0] if frame_res is not None else None
                angular, detected = controller.compute(frame)
                if not detected:
                    no_lane_steps += 1
                _obs, reward, terminated, truncated, info = env.step(np.array([angular], dtype=np.float32))
                step_index += 1
                if frame is not None:
                    frame_buffer.append((step_index, np.asarray(frame).copy()))
                records.append(_record_from_info(episode, step_index, info))
                print(f"step={step_index:04d} ey={info['ey']:.4f} epsi={info['epsi']:.4f} "
                      f"act={angular:+.3f} lane={'Y' if detected else 'n'} "
                      f"dbg={getattr(controller, 'dbg', {})} "
                      f"interv={info.get('cage_interventions', [])}")
            last_terminated, last_info = terminated, info

        summary = summarize_eval(records, perimeter=perimeter, cycle_dt_s=env.control_dt)
        summary["no_lane_steps"] = no_lane_steps
        summary["no_lane_rate_pct"] = 100.0 * no_lane_steps / max(1, len(records))
        if run_config is not None and run_config.perturbation.active:
            summary["perturbation"] = {"kind": run_config.perturbation.kind,
                                       "level": run_config.perturbation.level_label}
        _print_summary(summary)
        print(f"  no-lane (CV miss)   : {no_lane_steps} steps "
              f"({summary['no_lane_rate_pct']:.2f}%)")

        campaign = None
        if run_config is not None:
            campaign = _evaluate_scenario(
                records, run_config, completed=not last_terminated,
                control_dt=env.control_dt, road_half_m=0.5 * env.road_width,
                termination_reason=str(last_info.get("termination_reason", "")),
            )
            _print_verdict(scenario_id, campaign)

        dump_frames = (
            frame_buffer
            if str(last_info.get("termination_reason", "")) == "cage_emergency"
            else None
        )
        _write_run(cli, train_cfg, centerline_path, controller, seed,
                   records, summary, scenario_id, campaign, dump_frames)
    finally:
        if env is not None:
            env.close()
        if interface is not None:
            interface.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _dump_failure_frames(out_dir: Path, frames: Sequence[Tuple[int, np.ndarray]]) -> None:
    """Save the last buffered camera frames that preceded a cage-emergency stop.

    PNG via cv2 when available (frames are BGR, matching the estimator's input),
    else a single compressed .npz. Lets the estimator be replayed offline on the
    exact pixels that produced the misread — no Gazebo re-launch needed."""
    out_dir.mkdir(parents=True, exist_ok=True)
    try:
        import cv2  # noqa: PLC0415 — optional, only for the debug dump
        for step, frame in frames:
            cv2.imwrite(str(out_dir / f"frame_{step:04d}.png"), frame)
    except ImportError:
        np.savez_compressed(
            out_dir / "frames.npz",
            steps=np.array([s for s, _ in frames]),
            frames=np.stack([f for _, f in frames]),
        )
    print(f"Dumped {len(frames)} pre-emergency frames to {out_dir}")


def _write_run(cli, train_cfg, centerline_path, controller, seed,
               records, summary, scenario_id, campaign, dump_frames=None) -> None:
    runs_dir = resolve_runs_dir(cli.output_root)
    run_id = cli.run_id or "cv_ctrl_eval_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_dir = runs_dir / run_id
    out_dir.mkdir(parents=True, exist_ok=True)

    if dump_frames:
        _dump_failure_frames(out_dir / "failure_frames", dump_frames)

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
    if campaign is not None:
        summary_out["verdict"] = campaign["verdict"]
        summary_out["campaign"] = campaign
    with (out_dir / "summary.json").open("w", encoding="utf-8") as handle:
        json.dump(summary_out, handle, indent=2)

    metadata = {
        "run_id": run_id,
        "scenario_id": scenario_id,
        "mode": mode,
        "timestamp_iso": datetime.now(timezone.utc).isoformat(),
        "duration_s": summary["duration_s"],
        "git_commit": git_commit(out_dir),
        "cage_yaml": str(cage_yaml),
        "cage_yaml_hash": sha256_file(cage_yaml),
        # No learned checkpoint: the "policy" is the deterministic CV controller
        # (D-43 CV lane estimator + pure-pursuit look-ahead law).
        "controller": "cv_lane_controller (CvLaneEstimator D-43 + pure-pursuit look-ahead)",
        "controller_params": controller.p,
        "controller_source_hash": sha256_file(Path(__file__)),
        "centerline_yaml_hash": sha256_file(centerline_path),
        "scenario_yaml": str(cli.scenario or ""),
        "scenario_yaml_hash": sha256_file(Path(cli.scenario)) if cli.scenario else sha256_file(centerline_path),
        "rep": int(getattr(cli, "rep", 0)),
        "seed": seed,
        "platform": "sim",
        "status": "completed",
    }
    with (out_dir / "metadata.json").open("w", encoding="utf-8") as handle:
        json.dump(metadata, handle, indent=2)
    print(f"Wrote evaluation run to {out_dir}")


if __name__ == "__main__":
    main()
