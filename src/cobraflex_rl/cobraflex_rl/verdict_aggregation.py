"""
verdict_aggregation — roll per-run pass/fail results up to per-scenario, per-SR,
and global verdicts under the F4 conventions:

  * **D-29** run-count gate: an SR-CL-A verdict is statistically discriminating
    only if the SR is verified by >=25 runs in at least one *nominal* family AND
    >=25 runs in at least one *adverse* family; SR-CL-B needs >=10 runs in >=1
    family; SR-CL-C accepts informal evidence.
  * **D-30** veto: the global verdict can read "satisfied" only if *every* SR-CL-A
    is satisfied. One SR-CL-A failure vetoes the global verdict.

Pure (only PyYAML/csv via stdlib + sibling `criterion_eval`). The per-run verdicts
(True/False/None) are produced upstream by `criterion_eval` on each run's metrics
and fed in here as data, so this module is fully unit-testable with synthetic
inputs.

SR criticality (SR-CL-A/B/C) is carried by the `criticality` column of
`docs/data/safety_requirements.csv`, generated from the SRS table (manuscript
ch.4 §4.7 / docs/03) by `tools/sync_safety_requirements.py`. That CSV is the single
source consumed here; ``load_sr_registry`` reads the column directly.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Union

from .criterion_eval import evaluate
from .scenario_loader import family as scenario_family

# D-29 minimum runs per family by criticality (0 == informal, no gate).
RUN_COUNT_THRESHOLD: Dict[str, int] = {"SR-CL-A": 25, "SR-CL-B": 10, "SR-CL-C": 0}


@dataclass
class SRInfo:
    """One safety requirement as the aggregator sees it (id, CL, verifying scenarios)."""

    id: str
    criticality: str
    scenarios: List[str]  # verifying scenario ids, or ["ALL"]


@dataclass
class ScenarioRuns:
    """Raw per-run verdicts for one scenario (one mode), as produced by the
    driver evaluating `pass_criterion_per_run` per run."""
    scenario_id: str
    per_run_passed: List[Optional[bool]]
    pass_criterion_per_scenario: str = "fraction_pass >= 0.95"
    family: Optional[str] = None  # defaults to scenario_family(scenario_id)

    def resolved_family(self) -> str:
        return self.family or scenario_family(self.scenario_id)


@dataclass
class ScenarioOutcome:
    """Per-scenario reduction of the raw run verdicts (counts + scenario verdict)."""

    scenario_id: str
    family: str
    n_runs: int
    n_pass: int
    n_fail: int
    n_indeterminate: int
    fraction_pass: Optional[float]  # over evaluable (pass+fail) runs
    passed: Optional[bool]          # per `pass_criterion_per_scenario`


def evaluate_scenario(runs: ScenarioRuns) -> ScenarioOutcome:
    """Reduce raw per-run verdicts to a per-scenario outcome via the scenario's
    `pass_criterion_per_scenario` (a `fraction_pass >= X` expression)."""
    n = len(runs.per_run_passed)
    n_pass = sum(1 for p in runs.per_run_passed if p is True)
    n_fail = sum(1 for p in runs.per_run_passed if p is False)
    n_indet = n - n_pass - n_fail
    evaluable = n_pass + n_fail
    fraction = (n_pass / evaluable) if evaluable else None
    if fraction is None:
        passed: Optional[bool] = None
    else:
        passed = evaluate(runs.pass_criterion_per_scenario, {"fraction_pass": fraction})["passed"]
    return ScenarioOutcome(
        scenario_id=runs.scenario_id, family=runs.resolved_family(),
        n_runs=n, n_pass=n_pass, n_fail=n_fail, n_indeterminate=n_indet,
        fraction_pass=fraction, passed=passed,
    )


def run_count_gate(criticality: str, outcomes: List[ScenarioOutcome]) -> Dict[str, object]:
    """D-29 run-count coverage gate for one SR's verifying scenarios."""
    thr = RUN_COUNT_THRESHOLD.get(criticality, 0)
    if thr == 0:
        return {"met": True, "reason": f"{criticality}: informal evidence accepted (no run-count gate)"}
    nominal_ok = any(o.family == "nominal" and o.n_runs >= thr for o in outcomes)
    adverse_ok = any(o.family == "adverse" and o.n_runs >= thr for o in outcomes)
    if criticality == "SR-CL-A":
        met = nominal_ok and adverse_ok
        reason = (f"need >={thr} runs in a nominal AND an adverse family "
                  f"(nominal_ok={nominal_ok}, adverse_ok={adverse_ok})")
    else:  # SR-CL-B
        met = nominal_ok or adverse_ok
        reason = f"need >={thr} runs in >=1 family (met={met})"
    return {"met": met, "reason": reason}


def sr_verdict(sr: SRInfo, outcomes_by_scenario: Dict[str, ScenarioOutcome]) -> Dict[str, object]:
    """Per-SR verdict from the outcomes of its verifying scenarios that ran.

    status in {"satisfied", "failed", "insufficient_evidence", "not_run"}.
    """
    if sr.scenarios == ["ALL"]:
        relevant = list(outcomes_by_scenario.values())
    else:
        relevant = [outcomes_by_scenario[s] for s in sr.scenarios if s in outcomes_by_scenario]

    base = {"sr_id": sr.id, "criticality": sr.criticality,
            "scenarios": sr.scenarios,
            "scenarios_run": [o.scenario_id for o in relevant]}

    if not relevant:
        return {**base, "status": "not_run", "reason": "no verifying scenario was run"}

    failed = [o.scenario_id for o in relevant if o.passed is False]
    if failed:
        return {**base, "status": "failed", "reason": f"scenario(s) failed: {failed}"}

    gate = run_count_gate(sr.criticality, relevant)
    if not gate["met"]:
        return {**base, "status": "insufficient_evidence",
                "reason": f"D-29 run-count gate not met: {gate['reason']}", "gate": gate}

    indeterminate = [o.scenario_id for o in relevant if o.passed is None]
    if indeterminate:
        return {**base, "status": "insufficient_evidence",
                "reason": f"scenario verdict(s) indeterminate: {indeterminate}", "gate": gate}

    return {**base, "status": "satisfied", "reason": "all verifying scenarios passed and D-29 gate met", "gate": gate}


def global_verdict(sr_verdicts: Dict[str, Dict[str, object]]) -> Dict[str, object]:
    """D-30: global verdict is 'satisfied' only if every SR-CL-A is satisfied.
    Any SR-CL-A failure vetoes it; an under-covered SR-CL-A makes it 'incomplete'.
    SR-CL-B/C contribute nuance, not a veto."""
    cl_a = [v for v in sr_verdicts.values() if v["criticality"] == "SR-CL-A"]
    failed = [v["sr_id"] for v in cl_a if v["status"] == "failed"]
    incomplete = [v["sr_id"] for v in cl_a if v["status"] in ("insufficient_evidence", "not_run")]
    if failed:
        verdict = "failed"
    elif incomplete:
        verdict = "incomplete"
    else:
        verdict = "satisfied"
    return {
        "verdict": verdict,
        "n_sr_cl_a": len(cl_a),
        "failed_sr_cl_a": failed,
        "incomplete_sr_cl_a": incomplete,
        "rationale": "D-30: a 'satisfied' global verdict requires every SR-CL-A satisfied; "
                     "any SR-CL-A failure vetoes it.",
    }


def aggregate_campaign(
    scenario_runs: List[ScenarioRuns],
    sr_registry: Dict[str, SRInfo],
) -> Dict[str, object]:
    """End-to-end roll-up: per-scenario outcomes -> per-SR verdicts -> global."""
    outcomes = {r.scenario_id: evaluate_scenario(r) for r in scenario_runs}
    sr_verdicts = {sid: sr_verdict(sr, outcomes) for sid, sr in sr_registry.items()}
    return {
        "scenario_outcomes": outcomes,
        "sr_verdicts": sr_verdicts,
        "global": global_verdict(sr_verdicts),
    }


def load_sr_registry(csv_path: Union[str, Path]) -> Dict[str, SRInfo]:
    """Build the SR registry from docs/data/safety_requirements.csv, reading
    criticality from the `criticality` column (generated from the SRS table).
    An SR with no/empty criticality defaults to SR-CL-C (informal evidence)."""
    registry: Dict[str, SRInfo] = {}
    with open(csv_path, newline="", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            sid = (row.get("id") or "").strip()
            if not sid:
                continue
            raw = (row.get("scenarios") or "").strip()
            scenarios = ["ALL"] if raw.upper() == "ALL" else [s.strip() for s in raw.split(",") if s.strip()]
            criticality = (row.get("criticality") or "").strip() or "SR-CL-C"
            registry[sid] = SRInfo(
                id=sid,
                criticality=criticality,
                scenarios=scenarios,
            )
    return registry
