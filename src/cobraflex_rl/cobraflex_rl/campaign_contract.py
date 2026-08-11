"""Guards for preregistered posterior campaign configurations.

The frozen F/E verdict configs predate this mechanism and intentionally remain
unaffected.  A config that opts into ``campaign_contract`` must be internally
consistent, and its contract fingerprint is persisted inside every SB3
checkpoint.  Evaluation (or resume) with a checkpoint from another action map
then fails instead of silently reinterpreting its normalized throttle output.
"""

from __future__ import annotations

import math
from typing import Any, Dict, Mapping, Optional


MODEL_CONTRACT_ATTRIBUTE = "_cobraflex_campaign_contract"


def campaign_contract_fingerprint(
    train_cfg: Mapping[str, Any],
) -> Optional[Dict[str, Any]]:
    """Validate and return the checkpoint fingerprint for an opted-in config.

    Configs without ``campaign_contract`` are the historical path and return
    ``None``.  The duplicate, declared ``max_speed_mps`` is deliberate: it
    makes the preregistered value an assertion against the executable action
    map rather than an explanatory comment.
    """

    raw_contract = train_cfg.get("campaign_contract")
    if raw_contract is None:
        return None
    if not isinstance(raw_contract, Mapping):
        raise ValueError("campaign_contract must be a mapping")

    required = {
        "schema_version",
        "id",
        "status",
        "requires_fresh_training",
        "historical_checkpoint_compatible",
        "c04_curve_ceiling_mps",
        "minimum_speed_margin_mps",
        "max_speed_mps",
        "d43_preflight_required",
        "replay_buffer_required",
        "bounded_training_required",
        "planned_training_steps",
        "planned_finetune_steps",
        "replay_buffer_covers_parent_and_finetune",
    }
    missing = sorted(required - set(raw_contract))
    if missing:
        raise ValueError(f"campaign_contract missing required keys: {missing}")

    if int(raw_contract["schema_version"]) != 1:
        raise ValueError("campaign_contract.schema_version must be 1")
    contract_id = str(raw_contract["id"]).strip()
    if not contract_id:
        raise ValueError("campaign_contract.id must be non-empty")
    if str(raw_contract["status"]) != "preregistered_untrained":
        raise ValueError(
            "campaign_contract.status must be 'preregistered_untrained' "
            "until fresh training evidence exists"
        )
    if raw_contract["requires_fresh_training"] is not True:
        raise ValueError("campaign_contract must require fresh training")
    if raw_contract["historical_checkpoint_compatible"] is not False:
        raise ValueError(
            "campaign_contract must reject historical checkpoint compatibility"
        )
    if raw_contract["d43_preflight_required"] is not True:
        raise ValueError("campaign_contract must require the D-43 preflight")
    if raw_contract["replay_buffer_required"] is not True:
        raise ValueError("campaign_contract must require replay-buffer evidence")
    if raw_contract["bounded_training_required"] is not True:
        raise ValueError("campaign_contract must require bounded training")
    if raw_contract["replay_buffer_covers_parent_and_finetune"] is not True:
        raise ValueError(
            "campaign_contract must require replay coverage for parent + fine-tune"
        )
    if train_cfg.get("save_replay_buffer") is not True:
        raise ValueError(
            "campaign_contract requires save_replay_buffer=true for "
            "reproducible SAC derivations"
        )

    planned_training_steps = int(raw_contract["planned_training_steps"])
    planned_finetune_steps = int(raw_contract["planned_finetune_steps"])
    if planned_training_steps <= 0 or planned_finetune_steps <= 0:
        raise ValueError("planned training/fine-tune steps must be positive")
    protocol_cfg = train_cfg.get("protocol") or {}
    if not isinstance(protocol_cfg, Mapping):
        raise ValueError("protocol must be a mapping when present")
    protocol_arm = str(protocol_cfg.get("arm", ""))
    expected_current_steps = (
        planned_finetune_steps
        if protocol_arm == "stall_variant"
        else planned_training_steps
    )
    if int(train_cfg.get("total_timesteps", 0)) != expected_current_steps:
        raise ValueError(
            "total_timesteps must match the preregistered bounded "
            f"{'fine-tune' if protocol_arm == 'stall_variant' else 'parent'} "
            f"horizon ({expected_current_steps})"
        )
    sac_cfg = train_cfg.get("sac")
    if not isinstance(sac_cfg, Mapping):
        raise ValueError("bounded replay contract requires a sac mapping")
    replay_buffer_size = int(sac_cfg.get("buffer_size", 0))
    cumulative_steps = planned_training_steps + planned_finetune_steps
    if replay_buffer_size < cumulative_steps:
        raise ValueError(
            f"SAC replay buffer {replay_buffer_size} does not cover the "
            f"parent + fine-tune horizon {cumulative_steps}"
        )

    action = train_cfg.get("action")
    if not isinstance(action, Mapping) or action.get("type") != "steer_throttle":
        raise ValueError(
            "campaign_contract requires action.type='steer_throttle'"
        )

    action_cap = float(action["max_speed_mps"])
    declared_cap = float(raw_contract["max_speed_mps"])
    ceiling = float(raw_contract["c04_curve_ceiling_mps"])
    required_margin = float(raw_contract["minimum_speed_margin_mps"])
    actual_margin = ceiling - action_cap

    if not math.isclose(action_cap, declared_cap, rel_tol=0.0, abs_tol=1e-12):
        raise ValueError(
            "campaign_contract.max_speed_mps must match "
            "action.max_speed_mps"
        )
    if required_margin <= 0.0:
        raise ValueError("campaign speed margin must be strictly positive")
    if actual_margin + 1e-12 < required_margin:
        raise ValueError(
            f"action cap {action_cap} leaves {actual_margin} m/s below the "
            f"declared C-04 curve ceiling, less than the required "
            f"{required_margin} m/s"
        )

    return {
        "schema_version": 1,
        "id": contract_id,
        "status": "preregistered_untrained",
        "requires_fresh_training": True,
        "historical_checkpoint_compatible": False,
        "algorithm": str(train_cfg.get("algorithm", "ppo")).lower(),
        "action_type": str(action["type"]),
        "max_speed_mps": action_cap,
        "throttle_deadband": float(action.get("throttle_deadband", 0.05)),
        "c04_curve_ceiling_mps": ceiling,
        "minimum_speed_margin_mps": required_margin,
        "d43_preflight_required": True,
        "replay_buffer_required": True,
        "bounded_training_required": True,
        "planned_training_steps": planned_training_steps,
        "planned_finetune_steps": planned_finetune_steps,
        "replay_buffer_size": replay_buffer_size,
        "replay_buffer_covers_parent_and_finetune": True,
    }


def bind_campaign_contract(
    model: Any,
    train_cfg: Mapping[str, Any],
    *,
    require_existing: bool,
) -> None:
    """Bind a fresh model, or verify a resumed model, against the config."""

    expected = campaign_contract_fingerprint(train_cfg)
    if expected is None:
        return

    existing = getattr(model, MODEL_CONTRACT_ATTRIBUTE, None)
    if require_existing and existing != expected:
        found = "missing" if existing is None else repr(existing)
        raise RuntimeError(
            f"Checkpoint campaign contract mismatch: expected {expected!r}, "
            f"found {found}. This preregistered action map requires a fresh "
            "training run; do not reinterpret a historical checkpoint."
        )
    setattr(model, MODEL_CONTRACT_ATTRIBUTE, expected)


def validate_checkpoint_campaign_contract(
    model: Any, train_cfg: Mapping[str, Any]
) -> None:
    """Reject evaluation of an opted-in config with a foreign checkpoint."""

    bind_campaign_contract(model, train_cfg, require_existing=True)
