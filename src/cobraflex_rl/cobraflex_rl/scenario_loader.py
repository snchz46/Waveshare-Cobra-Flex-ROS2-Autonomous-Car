"""
scenario_loader — parse the executable scenario YAMLs (scenarios/<cat>/*.yaml)
into typed ``RunSpec`` objects for the F4 campaign runner. Pure (only PyYAML).

Stubs (``status: stub``) are not runnable and raise ``ScenarioStub``; the bulk
loader skips them. ``RunSpec.family`` maps the scenario to the nominal/adverse
distinction the D-29 run-count gate needs (SC-NOM-* = nominal, everything else
= adverse).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Union

import yaml

REQUIRED_KEYS = {
    "id", "category", "description", "initial_conditions", "perturbations",
    "termination", "metrics_primary", "metrics_secondary",
    "pass_criterion_per_run", "pass_criterion_per_scenario",
    "references_SR", "n_runs_recommended",
}


class ScenarioError(ValueError):
    """A scenario YAML is malformed or incomplete."""


class ScenarioStub(Exception):
    """The scenario is an explicit stub (not yet a runnable full spec)."""


def family(scenario_id: str) -> str:
    """nominal/adverse family of a scenario id (SC-NOM-* nominal, else adverse)."""
    parts = scenario_id.split("-")
    return "nominal" if len(parts) >= 2 and parts[1] == "NOM" else "adverse"


@dataclass
class RunSpec:
    """A validated scenario YAML (one SC-* file) in object form."""

    id: str
    category: str
    description: str
    initial_conditions: Any
    perturbations: Any
    termination: Dict[str, Any]
    metrics_primary: List[str]
    metrics_secondary: List[str]
    pass_criterion_per_run: str
    pass_criterion_per_scenario: str
    references_SR: List[str]
    n_runs_recommended: Dict[str, int]

    @property
    def family(self) -> str:
        return family(self.id)

    def runs_for_mode(self, mode: str) -> int:
        """Recommended repetition count for a cage mode (0 when unspecified)."""
        return int(self.n_runs_recommended.get(mode, 0))


def load_scenario(path: Union[str, Path]) -> RunSpec:
    """Load and validate a single full scenario YAML into a ``RunSpec``.

    Raises ``ScenarioStub`` for explicit stubs and ``ScenarioError`` for
    malformed/incomplete full scenarios.
    """
    path = Path(path)
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ScenarioError(f"{path}: top-level YAML is not a mapping")
    sid = data.get("id")
    if not isinstance(sid, str) or not sid:
        raise ScenarioError(f"{path}: missing/invalid 'id'")
    if data.get("status") == "stub":
        raise ScenarioStub(sid)
    missing = REQUIRED_KEYS - set(data)
    if missing:
        raise ScenarioError(f"{sid}: missing required keys: {', '.join(sorted(missing))}")
    n_runs = data["n_runs_recommended"]
    if not isinstance(n_runs, dict):
        raise ScenarioError(f"{sid}: n_runs_recommended must map mode -> count")
    return RunSpec(
        id=sid,
        category=str(data["category"]),
        description=str(data["description"]),
        initial_conditions=data["initial_conditions"],
        perturbations=data["perturbations"],
        termination=dict(data["termination"]),
        metrics_primary=list(data["metrics_primary"]),
        metrics_secondary=list(data["metrics_secondary"]),
        pass_criterion_per_run=str(data["pass_criterion_per_run"]),
        pass_criterion_per_scenario=str(data["pass_criterion_per_scenario"]),
        references_SR=list(data["references_SR"]),
        n_runs_recommended={str(k): int(v) for k, v in n_runs.items()},
    )


def load_scenarios(scenarios_dir: Union[str, Path]) -> Dict[str, RunSpec]:
    """Load every runnable (non-stub) scenario under ``scenarios_dir``,
    returning ``{scenario_id: RunSpec}``. Stubs are skipped."""
    out: Dict[str, RunSpec] = {}
    for path in sorted(Path(scenarios_dir).glob("*/*.yaml")):
        try:
            spec = load_scenario(path)
        except ScenarioStub:
            continue
        out[spec.id] = spec
    return out
