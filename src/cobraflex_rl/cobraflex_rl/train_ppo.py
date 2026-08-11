"""SB3 training entry point (``ros2 launch cobraflex_rl train_lane.launch.py``).

Wires the full F3/E-track training loop: loads the centerline + training
config YAMLs, builds :class:`GazeboLaneEnv` over a live Gazebo instance,
unthrottles the sim clock for headless runs, trains the SB3 algorithm the
config selects (``algorithm: ppo`` — the default — or ``sac``; MlpPolicy on
the state obs, CnnPolicy + VecFrameStack on the camera obs), and writes the
reproducibility evidence — ``metadata.json`` (git commit, config/cage/policy
hashes, seed), the learning-curve/action CSVs and periodic checkpoints —
under ``experiments/sim/training/<run_id>/``. Supports ``--resume-from`` to
continue a saved checkpoint. Spec: Training Spec Ch.7 §7.2.

The two algorithms share everything outside the update rule — env, wrappers
(Monitor/VecFrameStack/VecNormalize), reward, cage wiring, seed, LR (+ linear
schedule) — so a config differing only in ``algorithm:`` is a like-for-like
comparison. PPO-only keys (``n_steps``, ``clip_range``, ``target_kl``, …) are
ignored under SAC; SAC's off-policy knobs live in the optional ``sac:`` block
(``buffer_size``, ``learning_starts``, ``tau``, ``train_freq``,
``gradient_steps``, ``ent_coef``) with SB3-standard defaults sized for this
env (buffer 100k ≈ 5.6 GB RAM on the 84×84×4 camera obs — set explicitly for
longer runs).
"""

from __future__ import annotations

import argparse
import csv
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional, Sequence

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
import numpy as np
import rclpy
from rclpy.utilities import remove_ros_args
from stable_baselines3 import PPO, SAC
from stable_baselines3.common.callbacks import CallbackList, CheckpointCallback
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import (
    DummyVecEnv,
    VecEnv,
    VecFrameStack,
    VecNormalize,
    VecTransposeImage,
)
import yaml

from .callbacks import (
    ActionSampleCallback,
    LearningCurveCallback,
    ProgressBarCallback,
)
from .campaign_contract import (
    bind_campaign_contract,
    campaign_contract_fingerprint,
)
from .gazebo_lane_env import GazeboLaneEnv
from .ros_interface import RosGazeboInterface
from .run_io import git_commit, sha256_file
from .training_metrics import SB3_SCALAR_COLUMNS_BY_ALGO


PACKAGE_NAME = "cobraflex_rl"
SCENARIO_ID = "SC-NOM-01"

# ``algorithm:`` config key -> SB3 class. Both are actor-critic over the same
# Box action space, so the env/wrapper stack is shared verbatim.
ALGORITHMS = {"ppo": PPO, "sac": SAC}


def resolve_share_directory() -> Path:
    """Installed package share dir, or the source tree when not colcon-built."""
    try:
        return Path(get_package_share_directory(PACKAGE_NAME))
    except PackageNotFoundError:
        return Path(__file__).resolve().parents[1]


def _resolve_repo_subdir(*parts: str) -> Path:
    """Walk up to the repo root (marked by .git/ or experiments/) and return the
    given subdirectory, creating it if needed. Falls back to a cwd-relative path."""
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / ".git").exists() or (parent / "experiments").is_dir():
            target = parent.joinpath(*parts)
            target.mkdir(parents=True, exist_ok=True)
            return target
    target = Path.cwd().joinpath(*parts)
    target.mkdir(parents=True, exist_ok=True)
    return target


def _resolve_cage_yaml(train_cfg: Dict[str, Any]) -> Path:
    """Cage config path: explicit ``cage.yaml_path`` or the repo's cage/cage.yaml."""
    explicit = train_cfg.get("cage", {}).get("yaml_path", "") or ""
    if explicit and Path(explicit).is_file():
        return Path(explicit)
    for parent in Path(__file__).resolve().parents:
        candidate = parent / "cage" / "cage.yaml"
        if candidate.is_file():
            return candidate
    return Path("cage/cage.yaml")


def load_yaml(path: Path) -> Dict[str, Any]:
    """Parse a YAML file into a dict."""
    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def parse_args(args: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """Parse CLI args, dropping the ROS-specific ones ros2 launch appends."""
    parser = argparse.ArgumentParser(
        description="Train an SB3 lane-following policy (PPO or SAC, per the "
        "training config's `algorithm` key)."
    )
    parser.add_argument("--centerline-config", type=str, default=None)
    parser.add_argument(
        "--road-centerline-config",
        type=str,
        default=None,
        help="Road-centre centerline YAML for off-road geometry (off_road = "
        "global distance to it > road_width/2). Use when the reward centerline "
        "is a lane offset and the circuit approaches itself (e.g. complex_b).",
    )
    parser.add_argument("--train-config", type=str, default=None)
    parser.add_argument("--model-path", type=str, default=None)
    parser.add_argument("--run-id", type=str, default=None)
    parser.add_argument(
        "--world-name",
        type=str,
        default=None,
        help="Gazebo SDF world name for gz service calls (set_pose/set_physics). "
        "Must match the <world name=...> of the launched .world "
        "(e.g. lane_following_complex_b for lane_following_oval_complex.world). "
        "Falls back to the config's world_name, then 'lane_following_oval'.",
    )
    parser.add_argument(
        "--resume-from",
        type=str,
        default=None,
        help="Saved SB3 .zip (matching the config's algorithm) to continue from; "
        "the run then adds the config's "
        "total_timesteps on top of the checkpoint's step count "
        "(SB3 reset_num_timesteps=False).",
    )
    parser.add_argument(
        "--resume-vecnormalize",
        type=str,
        default=None,
        help="VecNormalize .pkl paired with --resume-from. Required by the "
        "SC-PERT-03 protocol when the parent config normalizes rewards, so the "
        "one-shot continuation does not silently reset running statistics.",
    )
    parser.add_argument(
        "--resume-replay-buffer",
        type=str,
        default=None,
        help="SAC replay-buffer .pkl paired with --resume-from. SB3 excludes "
        "the buffer from the policy .zip, so a reproducible SAC continuation "
        "must restore it explicitly.",
    )
    cleaned_args = remove_ros_args(args=args)
    if cleaned_args and not cleaned_args[0].startswith("-"):
        cleaned_args = cleaned_args[1:]
    return parser.parse_args(cleaned_args)


def resolve_save_path(path: Path) -> Path:
    """The .zip path SB3 actually writes for a given model path."""
    return path if path.suffix == ".zip" else path.with_suffix(".zip")


def _linear_schedule(initial_value: float):
    """SB3-style schedule: ``initial_value`` at the start of training, linearly
    annealed to 0 at the end (``progress_remaining`` runs 1.0 -> 0.0)."""

    def schedule(progress_remaining: float) -> float:
        return float(progress_remaining) * float(initial_value)

    return schedule


def _resolve_learning_rate(train_cfg: Dict[str, Any]):
    """Return the LR (float) or a linear-anneal schedule per ``lr_schedule``."""
    base_lr = float(train_cfg.get("learning_rate", 3.0e-4))
    if str(train_cfg.get("lr_schedule", "constant")).lower() == "linear":
        return _linear_schedule(base_lr)
    return base_lr


def _resolve_target_kl(train_cfg: Dict[str, Any]) -> Optional[float]:
    """Trust-region KL early-stop (SB3 ``target_kl``); None disables it.

    Added after the ``ppo_newcam_complex_b_2024_1M`` pilot collapsed at ~105k
    steps: ``approx_kl`` ran away to ~2.7 (100x its healthy ~0.02-0.4 band) with
    no brake, destroying the policy and value function. SB3 stops the update once
    a minibatch's approx_kl exceeds ``1.5 * target_kl``, capping the runaway.
    """
    value = train_cfg.get("target_kl", None)
    return float(value) if value is not None else None


def _resolve_clip_range_vf(train_cfg: Dict[str, Any]) -> Optional[float]:
    """Value-function clipping (SB3 ``clip_range_vf``); None disables it.

    Bounds how far the critic's prediction can move per update — extra insurance
    against the value-loss spikes that drove the 1M pilot's sawtooth. Only
    meaningful when rewards are normalised (``normalize_reward``): on the raw
    reward scale (returns ~700) a 0.2 clip would freeze the critic, so set both
    together.
    """
    value = train_cfg.get("clip_range_vf", None)
    return float(value) if value is not None else None


def _resolve_algorithm(train_cfg: Dict[str, Any]) -> str:
    """The SB3 algorithm the config selects: ``algorithm: ppo`` (default) or
    ``sac``. A single-key config switch — everything else (env, wrappers,
    reward, cage, seed, LR schedule) is shared between the two."""
    algorithm = str(train_cfg.get("algorithm", "ppo")).lower()
    if algorithm not in ALGORITHMS:
        raise ValueError(
            f"Unknown algorithm '{algorithm}' (expected one of {sorted(ALGORITHMS)})"
        )
    return algorithm


def _resolve_sac_kwargs(train_cfg: Dict[str, Any]) -> Dict[str, Any]:
    """SAC's off-policy knobs from the optional ``sac:`` config block.

    Defaults are SB3-standard except ``buffer_size``: SB3's 1M default would
    hold ~56 GB of 84x84x4 uint8 transitions, so it is capped at 100k here
    (~5.6 GB) and should be set explicitly for longer runs. ``ent_coef``
    defaults to SAC's auto-tuned temperature. ``train_freq``/``gradient_steps``
    of 1/1 keep the canonical one-gradient-step-per-env-step cadence.
    """
    sac_cfg = dict(train_cfg.get("sac", {}) or {})
    ent_coef = sac_cfg.get("ent_coef", "auto")
    if not isinstance(ent_coef, str):
        ent_coef = float(ent_coef)
    return {
        "buffer_size": int(sac_cfg.get("buffer_size", 100_000)),
        "learning_starts": int(sac_cfg.get("learning_starts", 1000)),
        "tau": float(sac_cfg.get("tau", 0.005)),
        "train_freq": int(sac_cfg.get("train_freq", 1)),
        "gradient_steps": int(sac_cfg.get("gradient_steps", 1)),
        "ent_coef": ent_coef,
        "target_update_interval": int(sac_cfg.get("target_update_interval", 1)),
    }


def _write_training_metadata(
    run_dir: Path,
    run_id: str,
    seed: int,
    train_cfg: Dict[str, Any],
    train_cfg_path: Path,
    centerline_path: Path,
    cage_yaml: Path,
    model_path: Path,
    total_timesteps: int,
    status: str,
    resume_from: Optional[Path] = None,
    resume_vecnormalize: Optional[Path] = None,
    resume_replay_buffer: Optional[Path] = None,
) -> None:
    """Write the run's reproducibility metadata.json (also on failure)."""
    algorithm = _resolve_algorithm(train_cfg)
    shared_hparams = {
        "learning_rate": float(train_cfg.get("learning_rate", 3.0e-4)),
        "lr_schedule": str(train_cfg.get("lr_schedule", "constant")).lower(),
        "gamma": float(train_cfg.get("gamma", 0.99)),
        "batch_size": int(train_cfg.get("batch_size", 64)),
        "normalize_reward": bool(train_cfg.get("normalize_reward", False)),
    }
    if algorithm == "sac":
        hyperparameters = {**shared_hparams, **_resolve_sac_kwargs(train_cfg)}
    else:
        hyperparameters = {
            **shared_hparams,
            "n_steps": int(train_cfg.get("n_steps", 1024)),
            "n_epochs": int(train_cfg.get("n_epochs", 10)),
            "gae_lambda": float(train_cfg.get("gae_lambda", 0.95)),
            "clip_range": float(train_cfg.get("clip_range", 0.2)),
            "ent_coef": float(train_cfg.get("ent_coef", 0.0)),
            "vf_coef": float(train_cfg.get("vf_coef", 0.5)),
            "max_grad_norm": float(train_cfg.get("max_grad_norm", 0.5)),
            "target_kl": _resolve_target_kl(train_cfg),
            "clip_range_vf": _resolve_clip_range_vf(train_cfg),
        }
    metadata = {
        "run_id": run_id,
        "scenario_id": SCENARIO_ID,
        "mode": str(train_cfg.get("cage", {}).get("mode", "enforcement")),
        "timestamp_iso": datetime.now(timezone.utc).isoformat(),
        "git_commit": git_commit(run_dir),
        "train_config": str(train_cfg_path),
        "train_config_hash": sha256_file(train_cfg_path),
        "cage_yaml": str(cage_yaml),
        "cage_yaml_hash": sha256_file(cage_yaml),
        "scenario_yaml_hash": sha256_file(centerline_path),
        "policy_checkpoint": str(resolve_save_path(model_path)),
        "policy_checkpoint_hash": sha256_file(resolve_save_path(model_path)),
        "parent_policy_checkpoint": str(resume_from) if resume_from else None,
        "parent_policy_checkpoint_hash": sha256_file(resume_from) if resume_from else None,
        "parent_vecnormalize": str(resume_vecnormalize) if resume_vecnormalize else None,
        "parent_vecnormalize_hash": (
            sha256_file(resume_vecnormalize) if resume_vecnormalize else None
        ),
        "policy_vecnormalize": (
            str(resolve_save_path(model_path).with_suffix(".vecnormalize.pkl"))
            if bool(train_cfg.get("normalize_reward", False)) else None
        ),
        "policy_vecnormalize_hash": (
            sha256_file(resolve_save_path(model_path).with_suffix(".vecnormalize.pkl"))
            if bool(train_cfg.get("normalize_reward", False)) else None
        ),
        "parent_replay_buffer": str(resume_replay_buffer) if resume_replay_buffer else None,
        "parent_replay_buffer_hash": (
            sha256_file(resume_replay_buffer) if resume_replay_buffer else None
        ),
        "policy_replay_buffer": (
            str(resolve_save_path(model_path).with_suffix(".replay_buffer.pkl"))
            if algorithm == "sac" and bool(train_cfg.get("save_replay_buffer", False))
            else None
        ),
        "policy_replay_buffer_hash": (
            sha256_file(resolve_save_path(model_path).with_suffix(".replay_buffer.pkl"))
            if algorithm == "sac" and bool(train_cfg.get("save_replay_buffer", False))
            else None
        ),
        "seed": seed,
        "platform": "sim",
        "total_timesteps": total_timesteps,
        "algorithm": algorithm,
        "hyperparameters": hyperparameters,
        # Track 'E' provenance (inert for the state-vector track): policy
        # class, observation type, frame stack and the H-10 DR envelope.
        "policy": str(train_cfg.get("policy", "MlpPolicy")),
        "observation": dict(train_cfg.get("observation", {})),
        # Action contract + reward weights (D-50/D-59): without these a 2-D
        # steer+throttle run's metadata is indistinguishable from a frozen
        # 1-D run's ({} = config predates the block → 1-D steering-only).
        "action": dict(train_cfg.get("action", {})),
        "campaign_contract": campaign_contract_fingerprint(train_cfg),
        "reward": dict(train_cfg.get("reward", {})),
        "domain_randomization": dict(train_cfg.get("domain_randomization", {})),
        "status": status,
    }
    with (run_dir / "metadata.json").open("w", encoding="utf-8") as handle:
        json.dump(metadata, handle, indent=2)


def _append_checkpoint_registry(
    seed: int, total_timesteps: int, cage_yaml: Path, run_id: str,
    algorithm: str = "ppo",
) -> None:
    """Append the completed run to policy/checkpoints/checkpoint_registry.csv."""
    registry = _resolve_repo_subdir("policy", "checkpoints") / "checkpoint_registry.csv"
    write_header = not registry.exists()
    with registry.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        if write_header:
            writer.writerow([
                "checkpoint_id", "seed", "training_steps", "scenario_evaluated",
                "timestamp", "git_commit", "cage_yaml_hash", "notes",
            ])
        writer.writerow([
            f"cobraflex_{algorithm}_lane",
            seed,
            total_timesteps,
            "",  # scenario_evaluated — filled in by eval_policy later
            datetime.now(timezone.utc).isoformat(),
            git_commit(registry.parent),
            sha256_file(cage_yaml) or "",
            f"run_id={run_id}",
        ])


def main(args: Optional[Sequence[str]] = None) -> None:
    """Run one training session end-to-end (see module docstring)."""
    cli_args = parse_args(args)
    rclpy.init(args=args)

    share_dir = resolve_share_directory()
    centerline_path = Path(cli_args.centerline_config or share_dir / "config" / "oval_right_lane_centerline.yaml")
    train_cfg_path = Path(cli_args.train_config or share_dir / "config" / "train_ppo.yaml")

    centerline_cfg = load_yaml(centerline_path)
    train_cfg = load_yaml(train_cfg_path)
    campaign_contract_fingerprint(train_cfg)

    # Optional road-centreline for off-road geometry (see GazeboLaneEnv): the
    # reward centreline can be a lane offset, but "left the road" is judged
    # against the road centre. Needed on self-approaching circuits (complex_b).
    road_centerline_path = (
        Path(cli_args.road_centerline_config) if cli_args.road_centerline_config else None
    )
    road_centerline_points = None
    if road_centerline_path is not None:
        road_centerline_points = np.asarray(
            load_yaml(road_centerline_path)["centerline"]["points"], dtype=float
        )

    model_path = Path(cli_args.model_path or train_cfg.get("model_path", "cobraflex_ppo_lane"))
    model_path = model_path.expanduser()
    model_path.parent.mkdir(parents=True, exist_ok=True)

    seed = int(train_cfg.get("seed", 42))
    total_timesteps = int(train_cfg.get("total_timesteps", 50000))
    algorithm = _resolve_algorithm(train_cfg)
    run_id = cli_args.run_id or (
        f"{algorithm}_train_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    )
    run_dir = _resolve_repo_subdir("experiments", "sim", "training", run_id)
    checkpoints_dir = _resolve_repo_subdir("policy", "checkpoints")
    cage_yaml = _resolve_cage_yaml(train_cfg)

    interface: Optional[RosGazeboInterface] = None
    env: Optional[GazeboLaneEnv] = None
    status = "failed"
    resume_from = Path(cli_args.resume_from).resolve() if cli_args.resume_from else None
    resume_vecnormalize = (
        Path(cli_args.resume_vecnormalize).resolve()
        if cli_args.resume_vecnormalize else None
    )
    resume_replay_buffer = (
        Path(cli_args.resume_replay_buffer).resolve()
        if cli_args.resume_replay_buffer else None
    )

    obs_cfg = dict(train_cfg.get("observation", {}))
    camera_obs = str(obs_cfg.get("type", "state")) == "camera"

    world_name = (
        cli_args.world_name
        or str(train_cfg.get("world_name", "lane_following_oval"))
    )

    try:
        interface = RosGazeboInterface(
            world_name=world_name,
            camera_topic=(
                str(obs_cfg.get("camera", {}).get("topic", "/camera/image_raw_lane"))
                if camera_obs
                else ""
            ),
            service_timeout_ms=int(train_cfg.get("service_timeout_ms", 3500)),
        )
        if not interface.wait_for_initial_data(timeout_sec=10.0):
            raise RuntimeError("Timed out waiting for /odom data.")

        # Unthrottle the sim clock for headless training so episodes don't play
        # out in real time. Reversible, session-only: it touches neither the
        # (hash-tracked) .world nor max_step_size, so physics fidelity and run
        # reproducibility are unchanged. Set sim_real_time_factor: 1 to restore
        # real-time pacing. control_dt cadence stays correct via sim-time step_ros.
        rtf = float(train_cfg.get("sim_real_time_factor", 0.0))
        if rtf != 1.0:
            if interface.set_real_time_factor(rtf):
                interface.get_logger().info(
                    f"Set simulation real_time_factor to {rtf} (0 = as fast as possible)."
                )
            else:
                interface.get_logger().warning(
                    "Could not set real_time_factor via /set_physics; "
                    "training will run at the world's default pace."
                )

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

        check_env(env, warn=True, skip_render_check=True)

        # Track 'E' (docs/09 §10): camera obs trains a CnnPolicy over a frame
        # stack (k=4 fixed at E2; VecFrameStack recovers the velocity/rate
        # cues a single frame loses). SB3 adds VecTransposeImage on top
        # automatically. The state-vector track keeps the raw env + MlpPolicy.
        policy_name = str(train_cfg.get("policy", "MlpPolicy"))
        if camera_obs:
            frame_stack = int(obs_cfg.get("camera", {}).get("frame_stack", 4))
            # Monitor must wrap the raw env (SB3 only auto-wraps non-vec
            # envs); without it ep_rew_mean/ep_len_mean stay NaN in the
            # learning curve.
            train_env = VecFrameStack(
                DummyVecEnv([lambda: Monitor(env)]), n_stack=frame_stack
            )
        else:
            train_env = env

        # Optional reward normalization (VecNormalize, norm_reward only): divides
        # the reward the algorithm sees by the running std of the discounted
        # return, keeping the critic's targets ~O(1). The 1M pilot's sawtooth was
        # value-function-driven (value_loss spiking to ~470 chasing returns ~700);
        # this is the root-cause fix. norm_obs stays False — the CNN/MLP consume
        # raw obs, so eval/inference need NO normalization stats (the policy's
        # obs->action map is unchanged). ep_rew_mean stays raw (Monitor is inside).
        normalize_reward = bool(train_cfg.get("normalize_reward", False))
        if resume_vecnormalize is not None and resume_from is None:
            raise ValueError("--resume-vecnormalize requires --resume-from")
        if resume_replay_buffer is not None and resume_from is None:
            raise ValueError("--resume-replay-buffer requires --resume-from")
        if resume_replay_buffer is not None and algorithm != "sac":
            raise ValueError("--resume-replay-buffer is valid only for SAC")
        if normalize_reward:
            if not isinstance(train_env, VecEnv):
                train_env = DummyVecEnv([lambda: Monitor(env)])
            # SAC + image obs: transpose to channels-first INSIDE VecNormalize.
            # SB3 otherwise adds VecTransposeImage outside it, and the off-policy
            # replay buffer — which stores VecNormalize's *original* obs
            # (normalization is re-applied at sample time) — would receive
            # channels-last frames into a channels-first buffer and crash.
            # PPO stores the model-side (already transposed) obs, so its
            # (frozen) wrapper stack stays byte-identical.
            if algorithm == "sac" and camera_obs:
                train_env = VecTransposeImage(train_env)
            if resume_vecnormalize is not None:
                if not resume_vecnormalize.is_file():
                    raise FileNotFoundError(
                        f"VecNormalize resume state not found: {resume_vecnormalize}"
                    )
                train_env = VecNormalize.load(str(resume_vecnormalize), train_env)
                train_env.training = True
                train_env.norm_reward = True
            else:
                train_env = VecNormalize(
                    train_env,
                    norm_obs=False,
                    norm_reward=True,
                    gamma=float(train_cfg.get("gamma", 0.99)),
                    clip_reward=float(train_cfg.get("clip_reward", 10.0)),
                )
        elif resume_vecnormalize is not None:
            raise ValueError(
                "--resume-vecnormalize was supplied but normalize_reward is disabled"
            )

        algo_cls = ALGORITHMS[algorithm]
        if resume_from:
            model = algo_cls.load(
                resume_from,
                env=train_env,
                device=str(train_cfg.get("device", "cpu")),
                verbose=1,
            )
            if algorithm == "ppo":
                # PPO.load restores the checkpoint's hyperparameters, so re-apply
                # the trust-region brake from the (current) config — otherwise a
                # resume silently keeps the checkpoint's target_kl (often None).
                model.target_kl = _resolve_target_kl(train_cfg)
            elif resume_replay_buffer is not None:
                if not resume_replay_buffer.is_file():
                    raise FileNotFoundError(
                        f"SAC replay buffer not found: {resume_replay_buffer}"
                    )
                model.load_replay_buffer(str(resume_replay_buffer))
            print(
                f"Resumed {algorithm.upper()} from {resume_from} "
                f"at {model.num_timesteps} steps"
            )
        elif algorithm == "sac":
            model = SAC(
                policy=policy_name,
                env=train_env,
                # Shared with PPO: LR (+ optional linear schedule), gamma,
                # batch_size, seed, device — a config differing only in
                # ``algorithm:`` keeps everything else identical.
                learning_rate=_resolve_learning_rate(train_cfg),
                gamma=float(train_cfg.get("gamma", 0.99)),
                batch_size=int(train_cfg.get("batch_size", 64)),
                # Off-policy knobs from the ``sac:`` block (defaults in
                # _resolve_sac_kwargs; PPO-only keys are ignored here).
                **_resolve_sac_kwargs(train_cfg),
                device=str(train_cfg.get("device", "cpu")),
                seed=seed,
                verbose=1,
            )
        else:
            model = PPO(
                policy=policy_name,
                env=train_env,
                # learning_rate may be a float or a linear-anneal schedule
                # (lr_schedule: linear); target_kl is the trust-region brake.
                learning_rate=_resolve_learning_rate(train_cfg),
                gamma=float(train_cfg.get("gamma", 0.99)),
                n_steps=int(train_cfg.get("n_steps", 1024)),
                batch_size=int(train_cfg.get("batch_size", 64)),
                n_epochs=int(train_cfg.get("n_epochs", 10)),
                gae_lambda=float(train_cfg.get("gae_lambda", 0.95)),
                clip_range=float(train_cfg.get("clip_range", 0.2)),
                ent_coef=float(train_cfg.get("ent_coef", 0.0)),
                vf_coef=float(train_cfg.get("vf_coef", 0.5)),
                max_grad_norm=float(train_cfg.get("max_grad_norm", 0.5)),
                target_kl=_resolve_target_kl(train_cfg),
                clip_range_vf=_resolve_clip_range_vf(train_cfg),
                device=str(train_cfg.get("device", "cpu")),
                # §7.2.7: seeds python/numpy/torch + action space, and propagates to
                # env.reset(seed=...) so the spawn perturbation is reproducible too.
                seed=seed,
                verbose=1,
            )

        # §7.2.8: persist the learning curve + periodic checkpoints so the run
        # produces its §7.4 evidence automatically.
        # Persist the preregistered contract in fresh SB3 checkpoints. A resume
        # from a historical or differently scaled action map fails here.
        bind_campaign_contract(
            model,
            train_cfg,
            require_existing=bool(resume_from),
        )

        checkpoint_freq = int(train_cfg.get("checkpoint_freq", train_cfg.get("n_steps", 1024)))
        # SAC collects train_freq(=1) steps per "rollout", so the curve callback
        # is throttled to one row per n_steps-equivalent window; PPO keeps its
        # native one-row-per-rollout cadence (interval 0).
        curve_interval = (
            int(train_cfg.get("n_steps", 1024)) if algorithm == "sac" else 0
        )
        callback = CallbackList([
            ProgressBarCallback(
                total_timesteps=total_timesteps, desc=algorithm.upper()
            ),
            LearningCurveCallback(
                csv_path=run_dir / "learning_curve.csv",
                scalar_columns=SB3_SCALAR_COLUMNS_BY_ALGO[algorithm],
                min_row_interval=curve_interval,
            ),
            ActionSampleCallback(
                csv_path=run_dir / "action_samples.csv",
                sample_every=int(train_cfg.get("action_sample_every", 10)),
            ),
            CheckpointCallback(
                save_freq=checkpoint_freq,
                save_path=str(checkpoints_dir),
                # Periodic ckpts are named by run so concurrent/sequential runs
                # never overwrite each other in the shared policy/checkpoints
                # (mirror of the isaac_train fix, 03.07.2026).
                name_prefix=run_id,
                # Persist the VecNormalize running stats alongside each checkpoint
                # (needed only to *resume* a normalized run; deterministic eval
                # with norm_obs=False does not need them).
                save_vecnormalize=normalize_reward,
                # Periodic replay snapshots are a separate, default-off switch:
                # a camera buffer is multi-GB and duplicating it at every 25k
                # checkpoint can exhaust disk. ``save_replay_buffer`` below
                # still persists one deterministic final buffer for derivation.
                save_replay_buffer=bool(
                    train_cfg.get("checkpoint_save_replay_buffer", False)
                ),
            ),
        ])

        model.learn(
            total_timesteps=total_timesteps,
            callback=callback,
            # On resume, keep the checkpoint's step count so SB3 runs
            # total_timesteps *additional* steps on top of it.
            reset_num_timesteps=not resume_from,
        )
        model.save(str(model_path))
        if normalize_reward:
            if not isinstance(train_env, VecNormalize):
                raise RuntimeError(
                    "normalize_reward is enabled but the final env is not VecNormalize"
                )
            vecnormalize_path = resolve_save_path(model_path).with_suffix(
                ".vecnormalize.pkl"
            )
            train_env.save(str(vecnormalize_path))
        if algorithm == "sac" and bool(train_cfg.get("save_replay_buffer", False)):
            replay_path = resolve_save_path(model_path).with_suffix(".replay_buffer.pkl")
            model.save_replay_buffer(str(replay_path))
        status = "completed"
        print(f"Saved {algorithm.upper()} model to {resolve_save_path(model_path)}")
        print(f"Learning curve + metadata under {run_dir}")
    finally:
        if env is not None:
            env.close()
        if interface is not None:
            interface.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        # Always record the run (even on failure) for reproducibility.
        _write_training_metadata(
            run_dir=run_dir,
            run_id=run_id,
            seed=seed,
            train_cfg=train_cfg,
            train_cfg_path=train_cfg_path,
            centerline_path=centerline_path,
            cage_yaml=cage_yaml,
            model_path=model_path,
            total_timesteps=total_timesteps,
            status=status,
            resume_from=resume_from,
            resume_vecnormalize=resume_vecnormalize,
            resume_replay_buffer=resume_replay_buffer,
        )
        if status == "completed":
            _append_checkpoint_registry(
                seed, total_timesteps, cage_yaml, run_id, algorithm=algorithm
            )


if __name__ == "__main__":
    main()
