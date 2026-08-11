"""
criterion_eval — safe evaluator for scenario pass-criterion expressions
(the `pass_criterion_per_run` / `pass_criterion_per_scenario` strings in the
scenario YAMLs). Pure, no `eval()`, no ROS.

Grammar (matches the scenario library):
  * A criterion is a conjunction of comparisons joined by `AND`:
        "M-P1 < 0.05 AND M-P2 == 1 AND emergency == False"
  * A comparison is `<operand> <op> <value>` with op in {<, <=, ==, !=, >, >=}.
  * <operand> is any token used as a key in the supplied `values` dict —
    a metric id (M-P1, M-S2, ...) or a scenario-specific token (emergency,
    fraction_pass, time_to_recovery_heading, joint_envelope_assertion_failures,
    inter_cycle_oscillations, ...).
  * <value> is a number or a bool literal (True/False).
  * A *labelled multi-arm* criterion (SC-PERT-03) is `;`-separated, each segment
    `label: <conjunction>`; evaluate each arm against that label's values.

Evaluation is **three-valued**. A clause whose operand is missing/None (the
metric was not computable for this run, e.g. M-S4 without a ttlc series) is
*indeterminate*, not false. A criterion is:
    False  if any clause is False (a real failure dominates),
    None   if no clause is False but some clause is indeterminate,
    True   if every clause is True.
This prevents an unavailable metric from being silently scored as a pass or a
fail; the campaign driver surfaces `None` as "not evaluable".
"""

from __future__ import annotations

import math
import re
from typing import Any, Dict, List, Optional, Tuple

_OP_RE = re.compile(r"^(.*?)(<=|>=|==|!=|<|>)(.*)$")
_AND_RE = re.compile(r"\s+AND\s+")
_LABEL_SEG_RE = re.compile(r"^\s*[A-Za-z_]\w*\s*:\s*.+$")


def _coerce(token: str) -> Any:
    """Parse an RHS token into bool/int/float, falling back to the bare string."""
    s = token.strip()
    low = s.lower()
    if low == "true":
        return True
    if low == "false":
        return False
    try:
        if "." in s or "e" in low:
            return float(s)
        return int(s)
    except ValueError:
        return s  # bare string literal (uncommon)


def _compare(lhs: Any, op: str, rhs: Any) -> bool:
    """Apply one comparison operator (== on floats uses an absolute tolerance)."""
    if op in ("==", "!="):
        if isinstance(lhs, bool) or isinstance(rhs, bool):
            eq = bool(lhs) == bool(rhs)
        elif isinstance(lhs, (int, float)) and isinstance(rhs, (int, float)):
            eq = math.isclose(float(lhs), float(rhs), abs_tol=1e-9)
        else:
            eq = lhs == rhs
        return eq if op == "==" else not eq
    left, right = float(lhs), float(rhs)
    if op == "<":
        return left < right
    if op == "<=":
        return left <= right
    if op == ">":
        return left > right
    if op == ">=":
        return left >= right
    raise ValueError(f"unknown operator {op!r}")


def _combine(flags: List[Optional[bool]]) -> Optional[bool]:
    """Three-valued conjunction: False dominates, then None, else True."""
    if any(f is False for f in flags):
        return False
    if any(f is None for f in flags):
        return None
    return True


def evaluate(expression: str, values: Dict[str, Any]) -> Dict[str, Any]:
    """Evaluate a conjunction-of-comparisons criterion against ``values``.

    Returns ``{"passed": True|False|None, "clauses": [...], "error": str|None}``.
    """
    clauses: List[Dict[str, Any]] = []
    error: Optional[str] = None
    for raw in _AND_RE.split(expression.strip()):
        clause = raw.strip()
        if not clause:
            continue
        m = _OP_RE.match(clause)
        if not m:
            error = f"unparseable clause: {clause!r}"
            clauses.append({"clause": clause, "passed": None, "reason": "unparseable"})
            continue
        operand = m.group(1).strip()
        op = m.group(2)
        rhs = _coerce(m.group(3))
        if operand not in values or values[operand] is None:
            clauses.append({"clause": clause, "operand": operand, "op": op,
                            "rhs": rhs, "lhs": None, "passed": None,
                            "reason": "operand unavailable"})
            continue
        try:
            ok = _compare(values[operand], op, rhs)
        except (TypeError, ValueError) as exc:
            clauses.append({"clause": clause, "passed": None, "reason": f"compare error: {exc}"})
            continue
        clauses.append({"clause": clause, "operand": operand, "op": op,
                        "rhs": rhs, "lhs": values[operand], "passed": ok})
    passed = _combine([c["passed"] for c in clauses]) if clauses else None
    return {"passed": passed, "clauses": clauses, "error": error}


def is_labelled(expression: str) -> bool:
    """True if the expression is a `;`-separated set of `label: <expr>` arms."""
    if ";" not in expression:
        return False
    segments = [seg for seg in expression.split(";") if seg.strip()]
    return bool(segments) and all(_LABEL_SEG_RE.match(seg) for seg in segments)


def labelled_arms(expression: str) -> List[Tuple[str, str]]:
    """Ordered ``(label, sub-expression)`` arms of a labelled criterion
    (``"low: ...; high: ..."``). Order matches the YAML, which by convention
    matches the scenario's ``level_levels`` order (low arm first). A single-run
    evaluator picks the arm at the rep's level index; the scenario aggregator
    pools across arms via :func:`evaluate_labelled`."""
    arms: List[Tuple[str, str]] = []
    for seg in expression.split(";"):
        seg = seg.strip()
        if not seg:
            continue
        label, _, sub = seg.partition(":")
        arms.append((label.strip(), sub.strip()))
    return arms


def _split_labelled(expression: str) -> Dict[str, str]:
    """Split `label: expr; label: expr` into {label: expr}."""
    arms: Dict[str, str] = {}
    for seg in expression.split(";"):
        seg = seg.strip()
        if not seg:
            continue
        label, _, sub = seg.partition(":")
        arms[label.strip()] = sub.strip()
    return arms


def evaluate_labelled(expression: str, values_by_label: Dict[str, Dict[str, Any]]) -> Dict[str, Any]:
    """Evaluate a labelled multi-arm criterion; each arm against its own values.

    ``values_by_label`` maps each arm label to its metric values. Overall verdict
    is the three-valued conjunction across arms.
    """
    per_label: Dict[str, Any] = {}
    for label, sub in _split_labelled(expression).items():
        per_label[label] = evaluate(sub, values_by_label.get(label, {}))
    passed = _combine([r["passed"] for r in per_label.values()]) if per_label else None
    return {"passed": passed, "per_label": per_label}


def evaluate_labelled_arm(
    expression: str, label: str, values: Dict[str, Any]
) -> Dict[str, Any]:
    """Evaluate exactly one preregistered arm of a labelled criterion.

    Unlike :func:`evaluate_labelled`, this is the per-run operation used when
    the campaign has already selected the policy arm. An unknown label remains
    indeterminate and explicit; it can never fall back to another arm.
    """
    arms = _split_labelled(expression)
    if label not in arms:
        return {
            "passed": None,
            "clauses": [],
            "error": f"unknown labelled arm {label!r}; expected {sorted(arms)}",
        }
    result = evaluate(arms[label], values)
    result["label"] = label
    return result
