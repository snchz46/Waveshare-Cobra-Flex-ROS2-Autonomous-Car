"""
campaign_metrics — pure (ROS-free, numpy-free) computation of the F4 metric
catalogue (docs/06_metrics_catalogue.md) from per-step run records.

`eval_metrics.summarize_eval` produces only the F3 §7.5.1 subset (laps,
intervention rate, mean tracking error). The F4 scenario campaign needs the full
per-run catalogue M-P*/M-S*/M-I* so that each scenario's `pass_criterion_per_run`
can be evaluated and per-SR verdicts aggregated (D-29/D-30). This module is that
computation. It is intentionally pure so it is unit-testable on any host without
ROS or Gazebo.

Per-step record schema (the `cage_status.csv` row written by `eval_policy`):
    ey, epsi, s, speed, raw_steer, safe_steer, steer_correction (floats),
    interventions (list[str] of rule ids that fired this step), emergency (bool).

Availability of each metric from the *current* record schema is reported in the
returned ``availability`` map. Metrics needing data not in the schema:
  * M-S4 (TTLC p5)   — needs a per-step ``ttlc`` series (cage computes it for
                       C-03 internally but does not log it); pass ``ttlc=``.
  * M-P3 (speed compliance) and the C-04 arm of M-I4 — need per-step curvature to
                       pick v_max(kappa); pass ``kappa=`` else v_max_straight is
                       assumed (flagged approximate).
  * M-I4 C-03 arm    — needs ``ttlc``.
  * M-C1/M-C2 (timing) — not measured in sim; always None here.
  * M-I5 throttle arm — the record logs only the steering correction.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Sequence

# Default thresholds — the SRS/cage values (docs/03, cage/cage.yaml, docs/06).
# The caller may override any of these (e.g. from cage.yaml at runtime).
DEFAULT_PARAMS: Dict[str, float] = {
    "d_max": 0.16,            # SR-001 lane half-corridor (m)
    "theta_max": 0.44,        # SR-002 heading bound (rad, ~25 deg)
    "t_min": 1.0,             # SR-003 TTLC floor (s)
    "v_max_straight": 0.5,    # SR-004 straight speed ceiling (m/s)
    "v_max_curve": 0.25,      # SR-004 curve speed ceiling (m/s)
    "kappa_curve_threshold": 0.1,  # |kappa| above which v_max(kappa)=v_max_curve (1/m)
    "delta_max_steer": 0.15,  # SR-006/C-06 per-cycle steering delta bound (normalised)
    "t_window_s": 2.0,        # SR-009/M-P6 stall trailing window (s)
    "delta_s_min": 0.10,      # SR-009/M-P6 min progress in the window (m)
    "delta_t_settle_s": 1.0,  # SR-009/M-P6 settle after entering nominal (s)
    "t_psd_s": 1.0,           # M-P7/SR-011 heading-variability window (s)
    "control_dt": 0.10,       # control cycle (s)
}

CAGE_RULES = ("C-01", "C-02", "C-03", "C-04", "C-05", "C-06")

# Rules whose correction legitimately overrides the C-06 rate limit (a safety
# correction takes precedence over actuator smoothness, SR-006). Used to split the
# committed-steer rate into the regime the limiter actually governs vs. safety
# overrides. C-04 (speed) does not act on the steering channel, so it is excluded.
_SAFETY_OVERRIDE_RULES = frozenset({"C-01", "C-02", "C-03", "C-05"})


# --------------------------------------------------------------------------
# Small pure-python statistics helpers (no numpy dependency).
# --------------------------------------------------------------------------

def _mean(xs: Sequence[float]) -> float:
    return float(sum(xs) / len(xs)) if xs else 0.0


def _std(xs: Sequence[float]) -> float:
    """Population standard deviation (ddof=0)."""
    n = len(xs)
    if n == 0:
        return 0.0
    mu = _mean(xs)
    return math.sqrt(sum((x - mu) ** 2 for x in xs) / n)


def _percentile(xs: Sequence[float], pct: float) -> Optional[float]:
    """Linear-interpolation percentile (pct in [0, 100]). None if empty."""
    if not xs:
        return None
    ys = sorted(xs)
    if len(ys) == 1:
        return float(ys[0])
    rank = (pct / 100.0) * (len(ys) - 1)
    lo = int(math.floor(rank))
    hi = int(math.ceil(rank))
    if lo == hi:
        return float(ys[lo])
    frac = rank - lo
    return float(ys[lo] * (1.0 - frac) + ys[hi] * frac)


def _run_lengths(flags: Sequence[bool]) -> List[int]:
    """Lengths of maximal runs of consecutive True values."""
    runs: List[int] = []
    current = 0
    for f in flags:
        if f:
            current += 1
        elif current:
            runs.append(current)
            current = 0
    if current:
        runs.append(current)
    return runs


def _nominal_settled(records: List[Dict[str, Any]], settle_steps: int) -> List[bool]:
    """Per-step eligibility under the M-P6/M-P7 'settled nominal' rule: not in
    emergency, and at least ``settle_steps`` since the most recent entry into
    nominal mode. The run is treated as entering nominal at step 0."""
    eligible: List[bool] = []
    steps_since_entry = 0
    prev_emergency = True  # so step 0 counts as a fresh entry into nominal
    for r in records:
        emergency = bool(r.get("emergency"))
        if emergency:
            steps_since_entry = 0
            eligible.append(False)
            prev_emergency = True
            continue
        if prev_emergency:
            steps_since_entry = 1  # first nominal step of this interval
        else:
            steps_since_entry += 1
        eligible.append(steps_since_entry > settle_steps)
        prev_emergency = False
    return eligible


# --------------------------------------------------------------------------
# Per-run metric computation
# --------------------------------------------------------------------------

def compute_run_metrics(
    records: List[Dict[str, Any]],
    params: Optional[Dict[str, float]] = None,
    *,
    completed: Optional[bool] = None,
    kappa: Optional[Sequence[float]] = None,
    ttlc: Optional[Sequence[float]] = None,
) -> Dict[str, Any]:
    """Compute the per-run metric catalogue from per-step ``records``.

    Returns ``{"metrics": {...}, "availability": {...}, "n_steps": int}``.
    A metric value of ``None`` means "not computable from the inputs given";
    the reason is in ``availability``.
    """
    p = dict(DEFAULT_PARAMS)
    if params:
        p.update(params)
    dt = float(p["control_dt"])

    metrics: Dict[str, Any] = {}
    avail: Dict[str, str] = {}
    n = len(records)
    metrics_keys = (
        "M-P1", "M-P2", "M-P3", "M-P4", "M-P5", "M-P6", "M-P7",
        "M-S1", "M-S2", "M-S3", "M-S4",
        "M-I1", "M-I2", "M-I3", "M-I4", "M-I5", "M-C1", "M-C2",
    )
    if n == 0:
        return {"metrics": {k: None for k in metrics_keys}, "availability": {"_": "empty run"}, "n_steps": 0}

    ey = [float(r["ey"]) for r in records]
    epsi = [float(r["epsi"]) for r in records]
    speed = [float(r.get("speed", 0.0)) for r in records]
    raw_steer = [float(r.get("raw_steer", 0.0)) for r in records]
    steer_corr = [abs(float(r.get("steer_correction", 0.0))) for r in records]
    emergency_flags = [bool(r.get("emergency")) for r in records]
    interventions = [list(r.get("interventions", ())) for r in records]

    duration = n * dt

    # ---- Performance ----
    metrics["M-P1"] = math.sqrt(_mean([v * v for v in ey]))            # lateral RMSE
    metrics["M-P4"] = max(abs(v) for v in epsi)                        # heading max
    metrics["M-P5"] = _mean([abs(v) for v in epsi])                   # heading mean

    if completed is None:
        metrics["M-P2"] = None
        avail["M-P2"] = "completion not supplied by caller (driver passes `completed=`)"
    else:
        metrics["M-P2"] = 1 if completed else 0

    # M-P3 speed compliance vs v_max(kappa)
    vmax_series = _vmax_series(n, kappa, p)
    metrics["M-P3"] = 100.0 * sum(1 for i in range(n) if speed[i] <= vmax_series[i] + 1e-9) / n
    avail["M-P3"] = "exact" if kappa is not None else "approx: kappa absent, v_max_straight assumed"

    # M-P6 stall rate (eligible settled-nominal steps with trailing-window progress < delta_s_min)
    metrics["M-P6"], avail["M-P6"] = _stall_rate(speed, emergency_flags, p)

    # M-P7 heading variability (p95 of sliding-window std of epsi over eligible steps)
    metrics["M-P7"], avail["M-P7"] = _heading_variability(epsi, emergency_flags, p)

    # ---- Safety ----
    metrics["M-S1"] = max(abs(v) for v in ey)                          # max lateral offset
    metrics["M-S2"] = sum(1 for v in ey if abs(v) > p["d_max"]) / duration  # boundary viol / s
    metrics["M-S3"] = 1 if any(emergency_flags) else 0                 # per-run emergency bit
    avail["M-S3"] = "per-run bit; M-S3 rate is aggregated across runs"

    if ttlc is not None and len(ttlc) == n:
        finite = [float(t) for t in ttlc if t is not None and math.isfinite(float(t))]
        metrics["M-S4"] = _percentile(finite, 5.0)
        avail["M-S4"] = "exact" if finite else "no finite TTLC samples"
    else:
        metrics["M-S4"] = None
        avail["M-S4"] = "requires per-step ttlc series (not in current record schema)"

    # ---- Intervention ----
    any_fired = [bool(iv) for iv in interventions]
    metrics["M-I1"] = 100.0 * sum(any_fired) / n
    per_rule_count: Dict[str, int] = {}
    for iv in interventions:
        for rule in iv:
            per_rule_count[rule] = per_rule_count.get(rule, 0) + 1
    metrics["M-I2"] = {rule: 100.0 * cnt / n for rule, cnt in sorted(per_rule_count.items())}

    # M-I3 intervention duration distribution, per rule
    m_i3: Dict[str, Dict[str, float]] = {}
    for rule in sorted(per_rule_count):
        flags = [rule in iv for iv in interventions]
        lengths = _run_lengths(flags)
        if lengths:
            m_i3[rule] = {
                "count": len(lengths),
                "median": _percentile(lengths, 50.0),
                "p95": _percentile(lengths, 95.0),
                "max": float(max(lengths)),
            }
    metrics["M-I3"] = m_i3

    # M-I4 intervention-hazard correlation, per rule that fired
    metrics["M-I4"], avail["M-I4"] = _hazard_correlation(
        interventions, ey, epsi, speed, raw_steer, vmax_series, ttlc, p
    )

    # M-I5 action correction magnitude (steering channel only — throttle not logged)
    metrics["M-I5"] = {
        "steer_median": _percentile(steer_corr, 50.0),
        "steer_p95": _percentile(steer_corr, 95.0),
    }
    # SR-006/C-06 verification arm: per-cycle rate of the *committed* (post-cage)
    # steering command. C-06 is first in the chain (C-06 → C-04 → C-02 → C-03 →
    # C-01 → C-05): it bounds the *raw* action's rate, but a downstream safety
    # rule (C-01 lane, C-02 heading, C-03 TTLC, C-05 emergency) may then command a
    # larger correction to avert an imminent hazard — by design, smoothness yields
    # to safety. So SR-006 ("actuator smoothness", δ_max_steer per cycle) is
    # measured on the committed command and reported in two arms: ``rate_max`` over
    # all steps, and ``rate_max_smoothness`` over steps where no safety-override
    # rule fired (the regime the rate limiter actually governs). ``safe_steer`` is
    # read directly when present, else reconstructed as raw_steer + steer_correction.
    def _safe_steer(r: Dict[str, Any]) -> float:
        v = r.get("safe_steer")
        if v is not None:
            return float(v)
        return float(r.get("raw_steer", 0.0)) + float(r.get("steer_correction", 0.0))

    safe_steer = [_safe_steer(r) for r in records]
    rate_all: List[float] = []
    rate_smooth: List[float] = []
    for i in range(1, n):
        d = abs(safe_steer[i] - safe_steer[i - 1])
        rate_all.append(d)
        overridden = bool(_SAFETY_OVERRIDE_RULES & set(interventions[i])) or emergency_flags[i]
        if not overridden:
            rate_smooth.append(d)
    metrics["M-I5"].update({
        "steer_rate_max": max(rate_all) if rate_all else 0.0,
        "steer_rate_p95": _percentile(rate_all, 95.0),
        "steer_rate_max_smoothness": max(rate_smooth) if rate_smooth else 0.0,
        "delta_max_steer": p["delta_max_steer"],
        "steer_rate_smoothness_ok": (
            max(rate_smooth) <= p["delta_max_steer"] + 1e-9 if rate_smooth else True
        ),
    })
    avail["M-I5"] = (
        "steering channel only (throttle correction not in record schema); "
        "steer_rate_* = per-cycle |Δ committed steer|, 'smoothness' arm excludes "
        "steps with a downstream safety-override rule (C-01/C-02/C-03/C-05) — see SR-006"
    )

    # ---- Computational (not measured in sim) ----
    metrics["M-C1"] = None
    metrics["M-C2"] = None
    avail["M-C1"] = avail["M-C2"] = "not measured in sim runs"

    return {"metrics": metrics, "availability": avail, "n_steps": n, "duration_s": duration}


def _vmax_series(n: int, kappa: Optional[Sequence[float]], p: Dict[str, float]) -> List[float]:
    """Per-step speed ceiling: curve value where |kappa| exceeds the threshold."""
    if kappa is not None and len(kappa) == n:
        thr = p["kappa_curve_threshold"]
        return [p["v_max_curve"] if abs(float(k)) > thr else p["v_max_straight"] for k in kappa]
    return [p["v_max_straight"]] * n


def _stall_rate(speed, emergency_flags, p):
    """% of settled-nominal steps whose trailing window advanced < delta_s_min (M-A2)."""
    dt = float(p["control_dt"])
    win = max(1, int(round(p["t_window_s"] / dt)))
    settle_steps = int(round(p["delta_t_settle_s"] / dt))
    records_stub = [{"emergency": e} for e in emergency_flags]
    eligible = _nominal_settled(records_stub, settle_steps)
    stall = 0
    counted = 0
    for i in range(len(speed)):
        if not eligible[i] or i + 1 < win:
            continue  # need a full trailing window
        counted += 1
        progress = sum(speed[i - win + 1: i + 1]) * dt
        if progress < p["delta_s_min"]:
            stall += 1
    if counted == 0:
        return None, "undefined: no eligible settled-nominal step with a full trailing window"
    return 100.0 * stall / counted, "exact"


def _heading_variability(epsi, emergency_flags, p):
    """p95 of the rolling heading-error std over settled-nominal windows."""
    dt = float(p["control_dt"])
    win = max(2, int(round(p["t_psd_s"] / dt)))
    settle_steps = int(round(p["delta_t_settle_s"] / dt))
    records_stub = [{"emergency": e} for e in emergency_flags]
    eligible = _nominal_settled(records_stub, settle_steps)
    sigmas: List[float] = []
    for i in range(len(epsi)):
        if not eligible[i] or i + 1 < win:
            continue
        sigmas.append(_std(epsi[i - win + 1: i + 1]))
    if not sigmas:
        return None, "undefined: no eligible window with sufficient history"
    return _percentile(sigmas, 95.0), "exact"


def _hazard_correlation(interventions, ey, epsi, speed, raw_steer, vmax_series, ttlc, p):
    """Fraction of interventions whose state was compatible with the rule's hazard."""
    n = len(interventions)
    have_ttlc = ttlc is not None and len(ttlc) == n

    def compatible(rule: str, i: int) -> Optional[bool]:
        if rule == "C-01":
            return abs(ey[i]) > 0.5 * p["d_max"]
        if rule == "C-02":
            return abs(epsi[i]) > 0.5 * p["theta_max"]
        if rule == "C-03":
            if not have_ttlc:
                return None
            t = ttlc[i]
            return t is not None and math.isfinite(float(t)) and float(t) < 2.0 * p["t_min"]
        if rule == "C-04":
            return speed[i] > 0.8 * vmax_series[i]
        if rule == "C-06":
            if i == 0:
                return None
            return abs(raw_steer[i] - raw_steer[i - 1]) > 0.5 * p["delta_max_steer"]
        return None  # C-05 triggers are not reconstructable from the record

    out: Dict[str, Optional[float]] = {}
    for rule in CAGE_RULES:
        fired = [i for i in range(n) if rule in interventions[i]]
        if not fired:
            continue
        judged = [compatible(rule, i) for i in fired]
        judged = [b for b in judged if b is not None]
        out[rule] = (100.0 * sum(1 for b in judged if b) / len(judged)) if judged else None
    note = "C-05 omitted (triggers not in record)"
    if not have_ttlc:
        note += "; C-03 omitted (no ttlc)"
    return out, note


# --------------------------------------------------------------------------
# Per-scenario aggregation (docs/06 "Standard report format")
# --------------------------------------------------------------------------

def aggregate_metric(values: Sequence[float]) -> Dict[str, Optional[float]]:
    """median / mean / std / p5 / p95 over a sequence of per-run scalar values
    (None values are dropped). The docs/06 standard report format."""
    xs = [float(v) for v in values if v is not None]
    if not xs:
        return {"n": 0, "median": None, "mean": None, "std": None, "p5": None, "p95": None}
    return {
        "n": len(xs),
        "median": _percentile(xs, 50.0),
        "mean": _mean(xs),
        "std": _std(xs),
        "p5": _percentile(xs, 5.0),
        "p95": _percentile(xs, 95.0),
    }
