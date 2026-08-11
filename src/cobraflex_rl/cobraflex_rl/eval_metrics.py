"""
eval_metrics — pure (ROS-free) aggregation of RL evaluation runs (Training
Specification §7.5).

`eval_policy` collects one record per control step; `summarize_eval` turns those
records into the §7.5.1 comparison metrics (completed laps, cage intervention
rate, emergencies, mean tracking error). This module performs no I/O — the
caller owns CSV/JSON writing — so it is unit-testable without ROS or Gazebo.
"""

from __future__ import annotations

from typing import Dict, List, Sequence


def forward_delta(prev_s: float, s: float, perimeter: float) -> float:
    """Signed arc-length advance between two centerline positions on a closed
    loop, correcting the wrap at the start/finish line: a large backward jump
    (more than half the loop) is interpreted as a forward crossing of s=0.
    """
    ds = s - prev_s
    if perimeter > 0 and ds < -perimeter / 2.0:
        ds += perimeter
    return ds


def _mean(values: Sequence[float]) -> float:
    return float(sum(values) / len(values)) if values else 0.0


def summarize_eval(
    records: List[Dict],
    perimeter: float,
    cycle_dt_s: float,
) -> Dict:
    """Aggregate per-step eval records into the §7.5.1 metrics.

    Each record is a dict with at least:
        episode (int), ey (float), epsi (float), s (float),
        interventions (list[str]), emergency (bool).

    Laps are accumulated as the per-episode sum of forward arc-length progress
    (the reference resets at each new episode, so episode boundaries do not add
    a spurious lap).
    """
    total_steps = len(records)
    if total_steps == 0:
        return {
            "total_steps": 0,
            "episodes": 0,
            "completed_laps": 0.0,
            "mean_ey_m": 0.0,
            "mean_abs_ey_m": 0.0,
            "max_abs_ey_m": 0.0,
            "mean_epsi_rad": 0.0,
            "mean_abs_epsi_rad": 0.0,
            "emergency_steps": 0,
            "intervention_steps": 0,
            "intervention_rate_pct": 0.0,
            "interventions_per_rule": {},
            "cycle_dt_s": float(cycle_dt_s),
            "duration_s": 0.0,
        }

    eys = [float(r["ey"]) for r in records]
    epsis = [float(r["epsi"]) for r in records]

    emergency_steps = sum(1 for r in records if r.get("emergency"))
    intervention_steps = sum(1 for r in records if r.get("interventions"))
    per_rule: Dict[str, int] = {}
    for r in records:
        for rule in r.get("interventions", ()):
            per_rule[rule] = per_rule.get(rule, 0) + 1

    progress = 0.0
    prev_episode = None
    prev_s = None
    episodes = set()
    for r in records:
        ep = r.get("episode", 0)
        episodes.add(ep)
        s = float(r["s"])
        if ep != prev_episode:
            prev_episode = ep
            prev_s = s
            continue
        ds = forward_delta(prev_s, s, perimeter)
        if ds > 0:
            progress += ds
        prev_s = s
    completed_laps = progress / perimeter if perimeter > 0 else 0.0

    return {
        "total_steps": total_steps,
        "episodes": len(episodes),
        "completed_laps": completed_laps,
        "mean_ey_m": _mean(eys),
        "mean_abs_ey_m": _mean([abs(v) for v in eys]),
        "max_abs_ey_m": max(abs(v) for v in eys),
        "mean_epsi_rad": _mean(epsis),
        "mean_abs_epsi_rad": _mean([abs(v) for v in epsis]),
        "emergency_steps": emergency_steps,
        "intervention_steps": intervention_steps,
        "intervention_rate_pct": 100.0 * intervention_steps / total_steps,
        "interventions_per_rule": per_rule,
        "cycle_dt_s": float(cycle_dt_s),
        "duration_s": total_steps * float(cycle_dt_s),
    }
