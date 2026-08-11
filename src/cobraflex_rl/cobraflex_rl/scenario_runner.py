"""
scenario_runner — pure derivation of a single scenario *run configuration* from
its YAML dict and a repetition index: the env ``reset`` options (spawn
arc-length, deterministic heading error and lateral offset, plus the per-rep
randomisation jitter that makes the N runs independent), the commanded speed,
and the step horizon from the scenario timeout.

Kept ROS-free so the ``(scenario, rep) -> run-config`` mapping is unit-tested
without Gazebo; ``eval_policy`` applies the result to ``GazeboLaneEnv.reset``.
The randomisation jitter is seeded deterministically from ``(id, rep, base_seed)``
so a given rep is byte-for-byte reproducible across processes and hosts.
"""

from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple

import numpy as np

from .scenario_perturbations import NONE as NO_PERTURBATION
from .scenario_perturbations import ScenarioPerturbation, resolve_perturbation


@dataclass
class RunConfig:
    """Everything the executor needs for one (scenario, rep) run: the env
    reset options (jittered initial conditions), the commanded speed, the
    step horizon and the per-run pass criterion."""

    scenario_id: str
    reset_options: Dict[str, float]
    fixed_speed: float
    max_steps: int
    pass_criterion_per_run: str
    # Runtime perturbation for this (scenario, rep): observation noise / actuation
    # latency / throttle pulse, level-resolved by rep. NONE for unperturbed runs.
    perturbation: ScenarioPerturbation = field(default=NO_PERTURBATION)
    # Stable per-rep env seed (drives the obs-noise stream so each rep is
    # independent yet reproducible). Same basis as the initial-condition jitter.
    env_seed: int = 0
    # parameterised_grid (SC-EDGE-05) only: the grid point this rep ran — its
    # anchor id, expected cage co-activation set, boundary-bracket factor and the
    # scaled seed. None for every non-grid scenario. Carried so the run record /
    # SR-010 aggregator can compare expected vs observed co-activation.
    grid_point: Optional[Dict[str, Any]] = None


def run_seed(scenario_id: str, rep: int, base_seed: int) -> int:
    """Stable 32-bit seed for a (scenario, rep) cell. Uses SHA-256 rather than
    Python's salted ``hash`` so the jitter is reproducible across interpreter
    runs (the reproducibility-metadata contract for campaign runs)."""
    digest = hashlib.sha256(f"{scenario_id}:{rep}:{base_seed}".encode("utf-8")).hexdigest()
    return int(digest[:8], 16)


def _jitter(randomisation: Any, rng: "np.random.Generator") -> Tuple[float, float]:
    """(lateral_offset_m, heading_deg) jitter from a scenario's ``randomisation``
    block. Missing/``none`` randomisation yields no jitter (deterministic run)."""
    dlat = 0.0
    dpsi_deg = 0.0
    if isinstance(randomisation, dict):
        lat = randomisation.get("lateral_offset_uniform_m")
        if lat and len(lat) == 2:
            dlat = float(rng.uniform(float(lat[0]), float(lat[1])))
        head = randomisation.get("heading_uniform_deg")
        if head and len(head) == 2:
            dpsi_deg = float(rng.uniform(float(head[0]), float(head[1])))
    return dlat, dpsi_deg


_GRID_N_MIN = 20  # SR-010 minimum grid points (SC-EDGE-05: ">= 20 grid points")
# C-01/C-03 lane-boundary d_max (cage.yaml cage.c03_ttlc.d_max_m / c01.d_max_m).
# Used to invert the C-03 TTLC formula for a grid anchor's ttlc_seed_s. Keep in
# sync if the cage d_max changes (SC-EDGE-05 is the only consumer).
_GRID_DMAX_M = 0.16


def expand_grid(
    anchors: Any, n_min: int = _GRID_N_MIN, *,
    factor_lo: float = 0.85, factor_hi: float = 1.30,
) -> list:
    """Expand ``parameterised_grid`` anchors into >= ``n_min`` deterministic grid
    points (SC-EDGE-05 / SR-010 cage co-activation matrix).

    Each anchor is a co-activation seed ``{seed: {...}, expected_activation: [...]}``.
    We bracket the cage-activation boundary by scaling the seed's numeric
    magnitudes across a fixed factor sweep, so for a given anchor some points sit
    just inside and some just past the rule thresholds. Order is (anchor, factor)
    — fully deterministic, no RNG (SC-EDGE-05 fixes seeds for exact
    reproducibility). Non-numeric seed values pass through unscaled."""
    anchors = list(anchors or [])
    if not anchors:
        return []
    per_anchor = max(1, -(-n_min // len(anchors)))  # ceil(n_min / n_anchors)
    if per_anchor == 1:
        factors = [1.0]
    else:
        step = (factor_hi - factor_lo) / (per_anchor - 1)
        factors = [factor_lo + i * step for i in range(per_anchor)]
        # Snap the nearest factor to exactly 1.0 so the author-specified seed
        # (the canonical co-activation point) is always evaluated as-is.
        nearest = min(range(per_anchor), key=lambda i: abs(factors[i] - 1.0))
        factors[nearest] = 1.0
    points = []
    for anchor in anchors:
        seed = dict(anchor.get("seed") or {})
        for f in factors:
            scaled = {
                k: (float(v) * f if isinstance(v, (int, float)) and not isinstance(v, bool) else v)
                for k, v in seed.items()
            }
            points.append({
                "anchor_id": str(anchor.get("id", "")),
                "expected_activation": list(anchor.get("expected_activation") or []),
                "factor": round(float(f), 4),
                "seed": scaled,
            })
    return points


def _grid_run_config(
    scenario: Dict[str, Any], rep: int, *, control_dt: float, base_seed: int,
) -> RunConfig:
    """RunConfig for one rep of a ``parameterised_grid`` scenario (SC-EDGE-05).

    The grid point is selected by ``rep % n_points`` so a full campaign (5 reps ×
    >=20 points = 100 runs) hits every point an equal number of times. The point's
    seed maps onto the env reset options the cage actually sees:
    ``d_m`` -> lateral offset (C-01), ``theta_deg`` -> heading error (C-02),
    ``v_mps`` -> commanded speed (C-04). ``ttlc_seed_s`` / ``kappa_seed_rad_m``
    are carried through for forward-compat but have **no env hook yet** (the env
    must map a curvature seed to a curve spawn and seed a TTLC closing rate —
    host-side TODO), so anchors relying solely on them under-activate until then."""
    sid = str(scenario["id"])
    track = scenario.get("track") or {}
    ic = scenario.get("initial_conditions") or {}
    points = expand_grid(ic.get("grid_anchors"))
    start_s = float(track.get("start_s_m", 0.0))
    default_speed = float(ic.get("speed_mps", scenario.get("commanded_speed_mps", 0.2)))

    if not points:  # no anchors: degrade to a plain nominal spawn (no co-activation)
        reset_options = {"start_s_m": start_s, "lateral_offset_m": 0.0, "heading_error_rad": 0.0}
        grid_point = None
        speed = default_speed
    else:
        grid_point = points[rep % len(points)]
        seed = grid_point["seed"]
        speed = float(seed.get("v_mps", default_speed))
        d_m = float(seed.get("d_m", 0.0))
        heading_rad = math.radians(float(seed.get("theta_deg", 0.0)))
        ttlc_seed = seed.get("ttlc_seed_s")
        if ttlc_seed is not None and float(ttlc_seed) > 0.0:
            # Seed C-03: invert TTLC = (d_max - |d|) / (v·sin|psi|) for |psi| so the
            # vehicle is the requested time-to-lane-crossing from the boundary,
            # drifting toward the side of its lateral offset. Supersedes theta_deg.
            margin = max(0.0, _GRID_DMAX_M - abs(d_m))
            denom = max(speed, 1e-6) * float(ttlc_seed)
            psi = math.asin(min(1.0, margin / denom)) if denom > 0.0 else 0.0
            heading_rad = math.copysign(psi, d_m) if d_m != 0.0 else psi
        reset_options = {
            "start_s_m": start_s,
            "lateral_offset_m": d_m,
            "heading_error_rad": heading_rad,
        }
        # kappa_seed has no pure mapping (it indexes track geometry): carried
        # through for the env to resolve to a curve spawn via the tracker.
        if "kappa_seed_rad_m" in seed:
            reset_options["kappa_seed_rad_m"] = float(seed["kappa_seed_rad_m"])

    timeout_s = float((scenario.get("termination") or {}).get("timeout_s", 0.0))
    max_steps = int(round(timeout_s / control_dt)) if timeout_s > 0 else 0
    return RunConfig(
        scenario_id=sid,
        reset_options=reset_options,
        fixed_speed=speed,
        max_steps=max_steps,
        pass_criterion_per_run=str(scenario.get("pass_criterion_per_run", "")),
        perturbation=NO_PERTURBATION,  # grid stress is initial-condition placement, not runtime
        env_seed=run_seed(sid, rep, base_seed),
        grid_point=grid_point,
    )


def derive_run_config(
    scenario: Dict[str, Any],
    rep: int,
    *,
    control_dt: float = 0.10,
    base_seed: int = 0,
) -> RunConfig:
    """Derive the env-facing run configuration for repetition ``rep`` of
    ``scenario``. The seed pose (``pose.y`` lateral, ``pose.theta_deg`` heading)
    is read from ``initial_conditions``; the per-rep jitter is added on top.

    ``initial_conditions.type: parameterised_grid`` (SC-EDGE-05) is dispatched to
    :func:`_grid_run_config` — the rep selects a deterministic co-activation grid
    point instead of a jittered pose."""
    sid = str(scenario["id"])
    track = scenario.get("track") or {}
    ic = scenario.get("initial_conditions") or {}
    if str(ic.get("type", "")).strip() == "parameterised_grid":
        return _grid_run_config(scenario, rep, control_dt=control_dt, base_seed=base_seed)
    pose = ic.get("pose") or {}

    start_s = float(track.get("start_s_m", 0.0))
    seed_lat = float(pose.get("y", 0.0))
    seed_theta_deg = float(pose.get("theta_deg", 0.0))

    rng = np.random.default_rng(run_seed(sid, rep, base_seed))
    dlat, dpsi_deg = _jitter(ic.get("randomisation"), rng)

    reset_options = {
        "start_s_m": start_s,
        "lateral_offset_m": seed_lat + dlat,
        "heading_error_rad": math.radians(seed_theta_deg + dpsi_deg),
    }

    speed = float(scenario.get("commanded_speed_mps", 0.2))
    timeout_s = float((scenario.get("termination") or {}).get("timeout_s", 0.0))
    max_steps = int(round(timeout_s / control_dt)) if timeout_s > 0 else 0

    perturbation = resolve_perturbation(
        scenario.get("perturbations"), rep, control_dt=control_dt
    )

    return RunConfig(
        scenario_id=sid,
        reset_options=reset_options,
        fixed_speed=speed,
        max_steps=max_steps,
        pass_criterion_per_run=str(scenario.get("pass_criterion_per_run", "")),
        perturbation=perturbation,
        env_seed=run_seed(sid, rep, base_seed),
    )
