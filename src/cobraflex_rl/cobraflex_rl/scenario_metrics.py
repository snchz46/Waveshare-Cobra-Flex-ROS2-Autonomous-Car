"""
scenario_metrics — scenario-specific verdict metrics that are *not* part of the
generic per-run catalogue (``campaign_metrics`` / docs/06). These are the custom
tokens that appear in a scenario's ``pass_criterion_per_run`` and have to be
computed from the per-step records before ``criterion_eval`` can score the run.

Currently implemented:
  * ``time_to_recovery_heading`` (SC-EDGE-01) — recovery time of the heading
    error after an initial-heading perturbation, against the run's own
    steady-state ripple envelope (``heading_recovery_band_rad``, v2 / D-68);
    the v1 fixed 2.86° band is still selectable for reproducing historical
    records.
  * Frontier / out-of-ODD verdict metrics (the cage-efficacy study): the maximum
    lateral excursion, whether the run reached the road edge (the harm proxy),
    and whether the cage's emergency rule (C-05) fired. These drive the paired
    enforcement-vs-monitoring contrast that quantifies the cage's value.

Pure: per-step records + params in, scalars out; unit-tested without ROS.
"""

from __future__ import annotations

import math
from typing import Any, Dict, Optional, Sequence


def max_excursion_m(records: Sequence[Dict[str, Any]]) -> Optional[float]:
    """Maximum absolute lateral offset ``max|ey|`` (m) reached over the run — how
    far out the vehicle got. None for an empty run."""
    if not records:
        return None
    return max(abs(float(r.get("ey", 0.0))) for r in records)


def emergency_triggered(records: Sequence[Dict[str, Any]]) -> bool:
    """Whether the cage latched a C-05 emergency at any step of the run."""
    return any(bool(r.get("emergency")) for r in records)


def road_edge_contact(records: Sequence[Dict[str, Any]], road_half_m: float) -> Optional[bool]:
    """Whether the vehicle reached the road edge (``max|ey| >= road_half_m``) — the
    harm proxy for the cage-efficacy contrast. Under enforcement the cage should
    keep this False (it stops/steers before the edge); under monitoring (no cage
    action) a frontier run is expected to reach the edge. None for an empty run."""
    mx = max_excursion_m(records)
    if mx is None:
        return None
    return mx >= road_half_m


def heading_recovery_band_rad(
    records: Sequence[Dict[str, Any]],
    *,
    floor_rad: float = 0.05,
    cap_rad: float = 0.0873,
    tail_fraction: float = 0.5,
    quantile: float = 0.95,
) -> float:
    """The band that counts as "heading recovered" for this run (rad), v2 (D-68).

    Physically, a perturbation is *recovered* when the heading error is back
    inside the envelope the vehicle occupies in steady operation — not when it
    reaches some absolute angle. The v1 metric used a fixed 0.05 rad (2.86°)
    band calibrated on the F-track PD controller on the oval, and that does not
    transfer: heading error ripples about zero with a run-dependent amplitude
    (measured p90: 4.8° on the oval, 3.0° on complex_b), so requiring a
    *sustained* window below a fixed 2.86° tests ripple amplitude rather than
    recovery. The pathology is demonstrable on runs with **nothing to recover
    from**: under v1, all 50 unperturbed SC-NOM-02 oval runs "never recover"
    (median 12.2 s).

    The band is therefore the run's own steady-state ripple — the ``quantile``
    of ``|epsi|`` over its last ``tail_fraction`` — clamped to
    ``[floor_rad, cap_rad]``. The floor keeps v1's band as the tightest possible
    bar (so the metric can never become *more* permissive than v1 on a
    well-damped run); the cap is anchored to SR-011's ``σ_θ_max`` = 5°, so a
    policy cannot buy itself a wider band by oscillating more — beyond that bar
    the run is an SR-011 finding, which is where such behaviour belongs.

    Returns ``floor_rad`` for an empty/degenerate run.
    """
    n = len(records)
    if n == 0:
        return float(floor_rad)
    start = int(n * (1.0 - float(tail_fraction)))
    tail = [abs(float(r.get("epsi", 0.0))) for r in records[start:]] or [0.0]
    tail.sort()
    idx = min(len(tail) - 1, max(0, int(round(float(quantile) * (len(tail) - 1)))))
    return float(min(float(cap_rad), max(float(floor_rad), tail[idx])))


def time_to_recovery_heading(
    records: Sequence[Dict[str, Any]],
    *,
    threshold_rad: Optional[float] = None,
    settle_s: float = 0.5,
    control_dt: float = 0.10,
    ripple_reference: bool = True,
) -> Optional[float]:
    """Seconds from run start until ``|epsi|`` first drops into the recovery band
    *and stays* there for ``settle_s`` — i.e. the heading is recovered and held,
    not just momentarily crossing. Returns the time at the start of that
    sustained interval.

    The band is :func:`heading_recovery_band_rad` (v2, D-68) unless
    ``ripple_reference=False`` or an explicit ``threshold_rad`` is given, either
    of which restores the v1 fixed-band behaviour — kept so the historical
    campaign records (scored under v1, and immutable) stay reproducible.

    ``math.inf`` if the heading never recovers for a full settle window: a clause
    like ``time_to_recovery_heading < 2.0`` then evaluates to a real *fail*
    (``inf < 2.0`` is False) rather than the indeterminate that ``None`` would
    produce in ``criterion_eval``'s three-valued logic. ``None`` only for an
    empty run (no data at all).
    """
    n = len(records)
    if n == 0:
        return None
    legacy = threshold_rad is not None or not ripple_reference
    if legacy:
        # v1 path, bit-exact: fixed band, strict inequality.
        band = float(threshold_rad) if threshold_rad is not None else 0.05
    else:
        band = heading_recovery_band_rad(records)
    settle_steps = max(1, int(round(settle_s / control_dt)))
    below = [
        (abs(float(r.get("epsi", 0.0))) < band) if legacy
        else (abs(float(r.get("epsi", 0.0))) <= band)
        for r in records
    ]
    for i in range(n - settle_steps + 1):
        if all(below[i : i + settle_steps]):
            return i * control_dt
    return math.inf
