"""
scenario_perturbations — pure (ROS-free) runtime perturbation injection for the
F4 scenario campaign.

A scenario's ``perturbations:`` block (docs/05) is resolved, together with the
repetition index, into a concrete :class:`ScenarioPerturbation` that the env
applies during the run. Three *runtime* perturbation types are supported (the
ones that act during the episode, not at spawn or on the policy weights):

  * ``observation_noise``  — zero-mean Gaussian noise added to the *perceived*
    lateral offset (SC-PERT-01). It feeds the policy observation **and** the cage
    state (both are downstream of perception); the verdict is still measured on
    the true pose (Ch.8 §8.2.3, "proxy de percepción").
  * ``actuation_latency``  — a fixed delay (in control steps) between command and
    actuation (SC-PERT-02). The env buffers the ``/cmd_vel`` command by this many
    cycles.
  * ``throttle_override``  — a timed throttle pulse fed to the cage input
    (SC-EDGE-03). NB: the env's fixed-speed actuation (``target_speed_from_throttle``
    caps speed at ``fixed_speed``) limits the *speed-excess* magnitude; the
    override still reaches C-04, but exercising a real over-speed needs a
    variable-speed actuation change (documented in experiments/README).

Multi-level perturbations (``observation_noise`` σ-levels, ``actuation_latency``
ms-levels) pick their level by ``rep % n_levels`` so the N repetitions cover the
levels in equal round-robin shares — matching the YAML's "20 runs per level".

Out of scope here (different mechanisms, not runtime injection): the
``parameterised_grid`` initial conditions of SC-EDGE-05 and the
``pre_run_policy_finetune`` of SC-PERT-03.

Kept ROS-free so the ``(block, rep) -> perturbation`` mapping and the per-step
application are unit-tested without Gazebo.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional, Protocol


class _Rng(Protocol):  # minimal structural type for numpy Generator
    def normal(self, loc: float, scale: float) -> float: ...


@dataclass(frozen=True)
class ScenarioPerturbation:
    """A concrete, level-resolved runtime perturbation for one (scenario, rep)."""

    kind: str = "none"  # none | observation_noise | actuation_latency | throttle_override | visual_degradation
    # observation_noise
    obs_noise_sigma_m: float = 0.0
    obs_noise_channel: str = "lateral_offset"
    # actuation_latency
    latency_steps: int = 0
    # throttle_override
    pulse_t_start_s: float = 0.0
    pulse_duration_s: float = 0.0
    pulse_throttle: float = 0.0
    # visual_degradation (track 'E', SC-PERT-04..08): a visual_degradation
    # primitive applied to the camera frame from ``visual_onset_s`` on, before
    # BOTH consumers (policy CNN + cage CV estimator — D-43 common cause).
    # ``visual_mode`` is a cobraflex_rl.visual_degradation mode string; this
    # module stays string-only (the env binds the primitive).
    visual_mode: str = ""
    visual_level: float = 0.0
    visual_onset_s: float = 0.0
    # human-readable level tag for run logging (e.g. "sigma=0.03m", "latency=100ms")
    level_label: str = ""
    # index of this rep's level within the scenario's level list (rep % n_levels);
    # selects the arm of a labelled per-run criterion (e.g. SC-PERT-05 low/high).
    level_index: int = 0

    @property
    def active(self) -> bool:
        return self.kind != "none"

    def perceive_lateral(self, true_ey: float, rng: _Rng) -> float:
        """Perceived lateral offset = true + N(0, σ) for ``observation_noise`` on
        the ``lateral_offset`` channel; the true value otherwise. ``rng`` is the
        env's seeded Generator (so the noise stream is reproducible per rep)."""
        if (
            self.kind == "observation_noise"
            and self.obs_noise_channel == "lateral_offset"
            and self.obs_noise_sigma_m > 0.0
        ):
            return float(true_ey) + float(rng.normal(0.0, self.obs_noise_sigma_m))
        return float(true_ey)

    def throttle_override(self, t_s: float) -> Optional[float]:
        """The override throttle during the pulse window ``[t_start, t_start+dur)``,
        else ``None`` (use the nominal throttle)."""
        if self.kind == "throttle_override" and self.pulse_duration_s > 0.0:
            if self.pulse_t_start_s <= t_s < self.pulse_t_start_s + self.pulse_duration_s:
                return float(self.pulse_throttle)
        return None

    def visual_degradation_at(self, t_s: float) -> Optional[tuple]:
        """``(mode, level)`` of the visual degradation active at episode time
        ``t_s`` (onset semantics: active from ``visual_onset_s`` to episode
        end), else ``None`` (clean frame)."""
        if self.kind == "visual_degradation" and self.visual_mode:
            if t_s >= self.visual_onset_s:
                return self.visual_mode, float(self.visual_level)
        return None


NONE = ScenarioPerturbation()


def resolve_perturbation(
    block: Any, rep: int, *, control_dt: float = 0.1
) -> ScenarioPerturbation:
    """Resolve a scenario ``perturbations`` block + repetition index into a
    concrete :class:`ScenarioPerturbation`.

    Multi-level types select their level by ``rep % n_levels``. Unknown, ``none``,
    or non-runtime blocks (``parameterised_grid`` lives in initial_conditions;
    ``pre_run_policy_finetune`` is a pre-run policy swap) yield :data:`NONE`.
    """
    if not isinstance(block, dict):
        return NONE
    kind = str(block.get("type", "")).strip()

    if kind == "observation_noise":
        levels = [float(s) for s in (block.get("sigma_levels_m") or [])]
        if not levels:
            return NONE
        sigma = levels[rep % len(levels)]
        return ScenarioPerturbation(
            kind="observation_noise",
            obs_noise_sigma_m=sigma,
            obs_noise_channel=str(block.get("channel", "lateral_offset")),
            level_label=f"sigma={sigma:g}m",
            level_index=rep % len(levels),
        )

    if kind == "actuation_latency":
        levels_ms = [float(m) for m in (block.get("latency_levels_ms") or [])]
        if not levels_ms:
            return NONE
        ms = levels_ms[rep % len(levels_ms)]
        steps = max(0, int(round((ms / 1000.0) / float(control_dt))))
        return ScenarioPerturbation(
            kind="actuation_latency",
            latency_steps=steps,
            level_label=f"latency={ms:g}ms({steps}step)",
            level_index=rep % len(levels_ms),
        )

    if kind == "throttle_override":
        return ScenarioPerturbation(
            kind="throttle_override",
            pulse_t_start_s=float(block.get("at_time_s", 0.0)),
            pulse_duration_s=float(block.get("duration_s", 0.0)),
            pulse_throttle=float(block.get("throttle", 0.0)),
            level_label=f"pulse@{block.get('at_time_s', 0)}s",
        )

    if kind in ("visual_degradation", "perception_loss", "false_lane"):
        # Track 'E' camera stressors (SC-PERT-04..08). The scenario `mode`
        # maps onto a visual_degradation primitive; the YAML aliases used by
        # the scenario library resolve here so the library prose can stay
        # descriptive.
        levels = [float(s) for s in (block.get("level_levels") or [1.0])]
        level = levels[rep % len(levels)]
        mode = str(block.get("mode", "")).strip()
        mode = {
            "misleading_markings": "false_lane",
            "occlusion_or_dropout": "occlusion",
        }.get(mode, mode)
        if kind == "perception_loss" and not mode:
            mode = "occlusion"
        if kind == "false_lane":
            mode = "false_lane"
        if not mode:
            return NONE
        onset = float(block.get("at_time_s", 0.0))
        return ScenarioPerturbation(
            kind="visual_degradation",
            visual_mode=mode,
            visual_level=level,
            visual_onset_s=onset,
            level_label=f"{mode}={level:g}@{onset:g}s",
            level_index=rep % len(levels),
        )

    return NONE
