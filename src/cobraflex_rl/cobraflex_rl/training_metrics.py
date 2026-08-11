"""Pure (ROS/SB3/numpy-free) training-metric helpers for the PPO learning curve.

This module centralises the schema and aggregation logic that ``callbacks.py``
uses to log richer training dynamics (Training Spec §7.2.8): the per-rollout
safety-cage intervention rates (plan §11.1 Fig. 2/3) and the ordered CSV column
list. It is deliberately dependency-free so the aggregation can be unit-tested
on any host without Stable-Baselines3 or ROS (``policy/tests/test_training_metrics.py``).
"""
from __future__ import annotations

from collections.abc import Mapping
from typing import Dict, Iterable

# Canonical cage rule IDs (docs/01_id_conventions.md). Fixed order so the
# learning-curve CSV schema is stable across runs even when a rule never fires.
# An intervention whose rule falls outside this set still counts toward the
# aggregate ``intervention_rate`` (see RolloutCageStats.update); it simply gets
# no dedicated per-rule column.
CAGE_RULES: tuple[str, ...] = ("C-01", "C-02", "C-03", "C-04", "C-05", "C-06")

# PPO health scalars pulled from the SB3 logger once per rollout (callbacks.py
# reads ``model.logger.name_to_value``). Each entry is (csv_column, sb3_key,
# sign): the stored value is ``sign * raw``. Entropy is stored as
# ``-train/entropy_loss`` because SB3 logs ``entropy_loss = -mean(entropy)``,
# so the column reads as the policy's mean entropy (which should decrease as the
# policy commits to the task — plan §9.1 / Fig. 5).
SB3_SCALAR_COLUMNS: tuple[tuple[str, str, float], ...] = (
    ("explained_variance", "train/explained_variance", 1.0),
    ("value_loss", "train/value_loss", 1.0),
    ("entropy", "train/entropy_loss", -1.0),
    ("approx_kl", "train/approx_kl", 1.0),
    ("clip_fraction", "train/clip_fraction", 1.0),
    ("std", "train/std", 1.0),
)

# SAC variant of the scalar map: the CSV schema (column names/order) is kept
# identical to the PPO one so every learning-curve reader works unchanged;
# columns whose concept has no SAC counterpart keep an empty key and stay NaN.
# ``value_loss`` carries SAC's critic loss (the closest health signal), and
# ``entropy`` carries the auto-tuned entropy coefficient's current value
# (train/ent_coef) — the SAC series that plays the "exploration is
# contracting" role std/entropy play for PPO.
SB3_SCALAR_COLUMNS_SAC: tuple[tuple[str, str, float], ...] = (
    ("explained_variance", "", 1.0),
    ("value_loss", "train/critic_loss", 1.0),
    ("entropy", "train/ent_coef", 1.0),
    ("approx_kl", "", 1.0),
    ("clip_fraction", "", 1.0),
    ("std", "", 1.0),
)

# Scalar map per algorithm (train_ppo.py's ``algorithm`` config key).
SB3_SCALAR_COLUMNS_BY_ALGO: Dict[str, tuple[tuple[str, str, float], ...]] = {
    "ppo": SB3_SCALAR_COLUMNS,
    "sac": SB3_SCALAR_COLUMNS_SAC,
}

# Full ordered learning_curve.csv schema (Training Spec §7.2.8). A superset of
# the legacy 4-column schema, so tools that read by column name (e.g.
# tools/plot_f3_figures.py) keep working on old runs and gain the new series on
# re-trained ones.
LEARNING_CURVE_COLUMNS: tuple[str, ...] = (
    ("timestep", "ep_rew_mean", "ep_len_mean")
    + tuple(col for col, _key, _sign in SB3_SCALAR_COLUMNS)
    + ("intervention_rate", "emergency_rate")
    + tuple(f"int_rate_{rule}" for rule in CAGE_RULES)
)


class RolloutCageStats:
    """Accumulate per-step safety-cage activity over one PPO rollout, from the
    env ``info`` dicts, and expose the per-rollout rates for the learning curve.

    Each ``info`` is expected to carry ``cage_interventions`` (an iterable of
    rule-ID strings that fired this step) and ``cage_emergency`` (bool); both are
    already populated by ``GazeboLaneEnv.step`` (see ``_apply_cage``). Missing
    keys are treated as "no activity", so the accumulator is safe against the
    cage-disabled debug loop too.
    """

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.total_steps = 0
        self.intervened_steps = 0
        self.emergency_steps = 0
        self.per_rule: Dict[str, int] = {rule: 0 for rule in CAGE_RULES}
        self.other_rule_steps = 0

    def update(self, info: Mapping) -> None:
        """Tally one env step from its ``info`` mapping."""
        self.total_steps += 1
        rules = info.get("cage_interventions") or ()
        if rules:
            self.intervened_steps += 1
        for rule in rules:
            if rule in self.per_rule:
                self.per_rule[rule] += 1
            else:
                self.other_rule_steps += 1
        if info.get("cage_emergency"):
            self.emergency_steps += 1

    def update_many(self, infos: Iterable) -> None:
        """Tally every Mapping in ``infos`` (SB3 passes one info dict per env)."""
        for info in infos:
            if isinstance(info, Mapping):
                self.update(info)

    def rates(self) -> Dict[str, float]:
        """Per-rollout fractions (of steps). An empty rollout yields all zeros."""
        n = self.total_steps or 1
        out: Dict[str, float] = {
            "intervention_rate": self.intervened_steps / n,
            "emergency_rate": self.emergency_steps / n,
        }
        for rule in CAGE_RULES:
            out[f"int_rate_{rule}"] = self.per_rule[rule] / n
        return out
