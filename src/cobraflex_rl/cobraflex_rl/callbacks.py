"""SB3 training callbacks: progress feedback + evidence CSVs.

Three callbacks composed by ``train_ppo``: a TTY/launch-aware progress bar,
the learning-curve recorder (reward, PPO health scalars and cage activity per
rollout — Training Spec §7.2.8) and a raw-steering subsampler for the
action-distribution figure. The CSV column schemas live in
``training_metrics``.
"""

from __future__ import annotations

import csv
import sys
import time
from collections.abc import Mapping
from pathlib import Path
from typing import Optional

from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.utils import safe_mean

from .training_metrics import (
    LEARNING_CURVE_COLUMNS,
    SB3_SCALAR_COLUMNS,
    RolloutCageStats,
)


class ProgressBarCallback(BaseCallback):
    """Training progress feedback showing steps, reward and episode length.

    Under a real terminal (stdout is a TTY) it renders a live tqdm bar.
    Under ``ros2 launch`` stdout is captured line-by-line, where tqdm's
    carriage-return redraw does not render; there it falls back to emitting a
    periodic one-line progress update that shows up cleanly in the launch log.

    ``tty_fallback`` handles the in-process Isaac trainer: ``python.sh`` + omni
    replace ``sys.stdout`` with a non-TTY log stream, so ``isatty()`` is ``False``
    even in a real terminal and no bar would show. When set, the bar is rendered
    straight to the controlling terminal (``/dev/tty``), bypassing the capture; if
    there is no controlling terminal (nohup/CI) the line-by-line fallback stands.

    Reward/length come from ``model.ep_info_buffer`` (populated by the Monitor
    wrapper SB3 adds automatically), matching SB3's ``rollout/ep_rew_mean`` and
    ``rollout/ep_len_mean``.
    """

    def __init__(
        self,
        total_timesteps: int,
        log_interval_steps: int = 1000,
        tty_fallback: bool = False,
        desc: str = "PPO",
    ) -> None:
        super().__init__()
        self.total_timesteps = int(total_timesteps)
        self.log_interval_steps = int(log_interval_steps)
        self.desc = str(desc)
        self._tty_file = None
        if tty_fallback:
            # Under Isaac, omni captures sys.stdout/stderr and re-emits them
            # line-by-line, so tqdm's carriage-return ('\r', no newline) redraws
            # never flush and the bar is invisible — even though isatty() may
            # report True. Writing straight to the controlling terminal
            # (/dev/tty) bypasses that capture, so prefer it. Falls back to the
            # plain isatty() bar (stderr) and finally to line-by-line output.
            try:
                self._tty_file = open("/dev/tty", "w")
                self._use_tqdm = True
            except OSError:
                self._use_tqdm = sys.stdout.isatty()
        else:
            self._use_tqdm = sys.stdout.isatty()
        self._pbar = None
        self._start_time = 0.0
        self._last_log_step = 0

    def _stats(self) -> tuple[float, float, int]:
        buffer = self.model.ep_info_buffer
        n_episodes = len(buffer) if buffer is not None else 0
        if n_episodes == 0:
            return 0.0, 0.0, 0
        mean_reward = safe_mean([ep["r"] for ep in buffer])
        mean_length = safe_mean([ep["l"] for ep in buffer])
        return float(mean_reward), float(mean_length), n_episodes

    def _fps(self) -> int:
        elapsed = time.time() - self._start_time
        return int(self.num_timesteps / elapsed) if elapsed > 0 else 0

    def _on_training_start(self) -> None:
        self._start_time = time.time()
        if self._use_tqdm:
            from tqdm import tqdm

            self._pbar = tqdm(
                total=self.total_timesteps,
                desc=self.desc,
                unit="step",
                dynamic_ncols=True,
                file=self._tty_file,  # None => tqdm's default (stderr)
            )
            dest = "/dev/tty" if self._tty_file is not None else "stderr"
            print(f"[progress] tqdm bar -> {dest}", flush=True)
        else:
            print("[progress] line-by-line mode (no usable TTY); progress every "
                  f"{self.log_interval_steps} steps", flush=True)

    def _on_step(self) -> bool:
        mean_reward, mean_length, n_episodes = self._stats()
        if self._use_tqdm:
            self._pbar.n = self.num_timesteps
            self._pbar.set_postfix(
                ep_rew=f"{mean_reward:.2f}",
                ep_len=f"{mean_length:.0f}",
                eps=n_episodes,
                refresh=False,
            )
            self._pbar.refresh()
        elif self.num_timesteps - self._last_log_step >= self.log_interval_steps:
            self._last_log_step = self.num_timesteps
            pct = 100.0 * self.num_timesteps / max(self.total_timesteps, 1)
            print(
                f"{self.desc} progress: {self.num_timesteps}/{self.total_timesteps} "
                f"({pct:.0f}%) | ep_rew_mean={mean_reward:.2f} | "
                f"ep_len_mean={mean_length:.0f} | episodes={n_episodes} | "
                f"fps={self._fps()}",
                flush=True,
            )
        return True

    def _on_training_end(self) -> None:
        if self._use_tqdm and self._pbar is not None:
            self._pbar.n = self.num_timesteps
            self._pbar.refresh()
            self._pbar.close()
            self._pbar = None
        if self._tty_file is not None:
            self._tty_file.close()
            self._tty_file = None


class LearningCurveCallback(BaseCallback):
    """Persist the PPO learning curve to a CSV (Training Spec §7.2.8): one row
    per rollout with the columns in ``training_metrics.LEARNING_CURVE_COLUMNS`` —
    the episode aggregates (``ep_rew_mean``/``ep_len_mean``), the PPO health
    scalars (``explained_variance``, ``value_loss``, ``entropy``, ``approx_kl``,
    ``clip_fraction``, ``std``) and the safety-cage activity (``intervention_rate``,
    ``emergency_rate`` and per-rule ``int_rate_C-0x``). The cage series is what
    the original 4-column schema lacked, so the co-adaptation figures (plan
    §11.1 Fig. 2/3) and the PPO-health figure (Fig. 5) can be drawn from a
    re-trained run.

    ``ep_rew_mean``/``ep_len_mean`` come from ``model.ep_info_buffer`` (the source
    SB3 logs as ``rollout/*``). The PPO scalars are captured opportunistically in
    ``_on_step`` from the SB3 logger — PPO records ``train/*`` after each update —
    so the value written is the most recent completed update's (NaN before the
    first update; the same one-rollout lag the original callback had for
    ``explained_variance``, kept here for every scalar). The cage rates are
    accumulated per env step from the env ``info`` dicts (``RolloutCageStats``)
    and reset every rollout.

    ``scalar_columns`` selects the logger-key map (PPO default, or the SAC map
    ``training_metrics.SB3_SCALAR_COLUMNS_SAC``); the CSV column names are the
    same for both, so downstream readers are algorithm-agnostic.

    ``min_row_interval`` throttles rows for off-policy algorithms: SB3 fires
    ``_on_rollout_end`` after every collection phase, which for SAC with
    ``train_freq=1`` is *every env step*. Rows are only written once at least
    this many timesteps have passed since the last one (0 keeps the PPO
    one-row-per-rollout behaviour); cage stats keep accumulating across the
    skipped rollouts so the rates still cover the whole window.
    """

    def __init__(
        self,
        csv_path,
        verbose: int = 0,
        scalar_columns=SB3_SCALAR_COLUMNS,
        min_row_interval: int = 0,
    ) -> None:
        super().__init__(verbose)
        self.csv_path = Path(csv_path)
        self.scalar_columns = tuple(scalar_columns)
        self.min_row_interval = int(min_row_interval)
        self._last_row_timestep: Optional[int] = None
        self._scalars = {col: float("nan") for col, _key, _sign in self.scalar_columns}
        self._cage = RolloutCageStats()
        self._header_written = False

    def _ep_stats(self) -> tuple[float, float]:
        buffer = self.model.ep_info_buffer
        if not buffer:
            return float("nan"), float("nan")
        return (
            float(safe_mean([ep["r"] for ep in buffer])),
            float(safe_mean([ep["l"] for ep in buffer])),
        )

    def _on_training_start(self) -> None:
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)

    def _on_step(self) -> bool:
        # Capture the latest PPO health scalars. name_to_value is cleared on each
        # logger dump, so only overwrite the cached value when the key is present
        # (otherwise the last completed update's value persists — see docstring).
        name_to_value = self.model.logger.name_to_value
        for col, key, sign in self.scalar_columns:
            value = name_to_value.get(key) if key else None
            if value is not None:
                self._scalars[col] = sign * float(value)
        # Accumulate this step's cage activity (SB3 passes one info per parallel env).
        self._cage.update_many(self.locals.get("infos", ()))
        return True

    def _on_rollout_end(self) -> None:
        if (
            self.min_row_interval > 0
            and self._last_row_timestep is not None
            and self.num_timesteps - self._last_row_timestep < self.min_row_interval
        ):
            return  # keep accumulating cage stats; write on a later rollout end
        mean_reward, mean_length = self._ep_stats()
        row = {
            "timestep": self.num_timesteps,
            "ep_rew_mean": f"{mean_reward:.6f}",
            "ep_len_mean": f"{mean_length:.3f}",
        }
        for col, _key, _sign in self.scalar_columns:
            row[col] = f"{self._scalars[col]:.6f}"
        for col, value in self._cage.rates().items():
            row[col] = f"{value:.6f}"

        write_header = not self.csv_path.exists() and not self._header_written
        with self.csv_path.open("a", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(LEARNING_CURVE_COLUMNS))
            if write_header:
                writer.writeheader()
            writer.writerow(row)
        self._header_written = True
        self._last_row_timestep = self.num_timesteps
        self._cage.reset()


class ActionSampleCallback(BaseCallback):
    """Subsample the policy's action during rollout collection to a CSV
    ``[timestep, raw_steer]`` (+ ``raw_throttle`` on the 2-D action path, D-50)
    for the action-distribution figure (early- vs late-training; plan §11.1
    Fig. 6). One row every ``sample_every`` env steps, reading the raw policy
    action from the env ``info`` (``raw_steer``/``raw_throttle``, set by
    ``GazeboLaneEnv`` on both the caged and debug paths). Subsampling keeps
    the file small (≈ total_timesteps / sample_every rows). The throttle
    column is appended only when the env supplies it (older envs did not);
    readers (tools/plot_f3_figures.py) select columns by name, so both
    schemas stay readable.
    """

    def __init__(self, csv_path, sample_every: int = 10, verbose: int = 0) -> None:
        super().__init__(verbose)
        self.csv_path = Path(csv_path)
        self.sample_every = max(1, int(sample_every))
        self._header_written = False
        self._step = 0

    def _on_training_start(self) -> None:
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)

    def _on_step(self) -> bool:
        self._step += 1
        if self._step % self.sample_every != 0:
            return True
        rows = [
            (
                self.num_timesteps,
                float(info["raw_steer"]),
                float(info["raw_throttle"]) if "raw_throttle" in info else None,
            )
            for info in (self.locals.get("infos") or ())
            if isinstance(info, Mapping) and "raw_steer" in info
        ]
        if not rows:
            return True
        with_throttle = rows[0][2] is not None
        write_header = not self.csv_path.exists() and not self._header_written
        with self.csv_path.open("a", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            if write_header:
                header = ["timestep", "raw_steer"]
                if with_throttle:
                    header.append("raw_throttle")
                writer.writerow(header)
            for timestep, steer, throttle in rows:
                row = [timestep, f"{steer:.6f}"]
                if with_throttle and throttle is not None:
                    row.append(f"{throttle:.6f}")
                writer.writerow(row)
        self._header_written = True
        return True
