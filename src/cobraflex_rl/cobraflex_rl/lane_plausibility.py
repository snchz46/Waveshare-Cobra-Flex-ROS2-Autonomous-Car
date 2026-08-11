"""
lane_plausibility — pure-logic plausibility / temporal-consistency check for the cage's
CV lane estimate (track 'E'; SR-014, hazard H-12, decision D-43).

Under D-43 the cage's state comes from a deterministic CV lane-estimator that can be
confidently *wrong* (a false but plausible-looking lane — a fork, an old marking, a
shadow read as an edge). SR-014 requires the cage to **not** enforce C-01..C-06 on a
suspect estimate: when the estimate is geometrically implausible (outside the ODD ranges)
or temporally inconsistent (an inter-frame jump incompatible with the vehicle's motion)
beyond tolerance for more than ``min_implausible_cycles`` cycles, the cage shall fall back
to the C-05 controlled stop rather than steer toward the suspect lane.

This module is the **pure decision logic** (stdlib only, no ROS, host-testable). The CV
estimator itself and the wiring of the ``reject`` flag to the cage's C-05 trigger live on
the Ubuntu host; in simulation the estimator's error is validated against the ground-truth
oracle (D-43). Parameters are **provisional**, aligned with the ODD / cage state-validity
ranges at E-integration. Mirrors the persistence pattern of :mod:`cobraflex_rl.perception_health`.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional


@dataclass(frozen=True)
class LaneEstimate:
    """One cycle's CV lane estimate.

    ey: lateral offset to the lane centre (m, + left); heading: heading error (rad);
    lane_width: detected lane width (m); curvature: signed lane curvature (1/m).
    """

    ey: float
    heading: float
    lane_width: float
    curvature: float
    timestamp_s: float


@dataclass(frozen=True)
class PlausibilityResult:
    plausible: bool
    reject: bool  # raise the C-05 controlled-stop trigger this cycle
    reason: str


class LanePlausibilityCheck:
    """Decide, per cycle, whether the cage's CV lane estimate is trustworthy (SR-014).

    Geometric plausibility: ``ey``, ``heading``, ``lane_width`` and ``|curvature|`` must lie
    within the ODD ranges. Temporal consistency: the inter-frame change in ``ey`` / ``heading``
    must not exceed what the vehicle's motion allows over the elapsed ``dt`` (plus a tolerance).
    The ``reject`` flag (→ C-05 controlled stop) is raised only after ``min_implausible_cycles``
    consecutive failures, so a single suspect frame does not trip it (cf. C-05 persistence,
    SR-005 ``Δt_max``). The check reports per-cycle; the latch / explicit reset is C-05's job.
    """

    def __init__(self, *, ey_max: float = 0.30, heading_max: float = 0.79,
                 lane_width_range=(0.20, 0.80), curvature_max: float = 1.5,
                 jump_tol_m: float = 0.10, max_heading_rate: float = 6.0,
                 jump_tol_rad: float = 0.30, min_implausible_cycles: int = 2) -> None:
        lo, hi = lane_width_range
        if ey_max <= 0 or heading_max <= 0 or curvature_max <= 0:
            raise ValueError("ey_max, heading_max, curvature_max must be > 0")
        if not (0.0 < lo < hi):
            raise ValueError("lane_width_range must satisfy 0 < low < high")
        if jump_tol_m < 0 or jump_tol_rad < 0 or max_heading_rate <= 0:
            raise ValueError("tolerances must be >= 0 and max_heading_rate > 0")
        if min_implausible_cycles < 1:
            raise ValueError("min_implausible_cycles must be >= 1")
        self.ey_max = float(ey_max)
        self.heading_max = float(heading_max)
        self.lane_width_lo, self.lane_width_hi = float(lo), float(hi)
        self.curvature_max = float(curvature_max)
        self.jump_tol_m = float(jump_tol_m)
        self.max_heading_rate = float(max_heading_rate)
        self.jump_tol_rad = float(jump_tol_rad)
        self.min_implausible_cycles = int(min_implausible_cycles)
        self._consecutive_bad = 0
        self._prev: Optional[LaneEstimate] = None

    def reset(self) -> None:
        """Clear the persistence counter and the temporal reference (episode reset)."""
        self._consecutive_bad = 0
        self._prev = None

    def update(self, est: LaneEstimate, speed_mps: float, now_s: float) -> PlausibilityResult:
        """Judge one estimate: geometric ranges + temporal jump vs the last trusted one."""
        reasons: List[str] = []

        # Geometric plausibility (ODD ranges).
        if abs(est.ey) > self.ey_max:
            reasons.append("ey_out_of_range")
        if abs(est.heading) > self.heading_max:
            reasons.append("heading_out_of_range")
        if not (self.lane_width_lo <= est.lane_width <= self.lane_width_hi):
            reasons.append("lane_width_implausible")
        if abs(est.curvature) > self.curvature_max:
            reasons.append("curvature_out_of_range")

        # Temporal consistency (vs the previous accepted estimate).
        if self._prev is not None:
            dt = max(now_s - self._prev.timestamp_s, 1e-3)
            # The vehicle cannot shift laterally faster than it moves forward (+ tolerance);
            # heading cannot change faster than max_heading_rate (+ tolerance).
            allowed_dey = abs(speed_mps) * dt + self.jump_tol_m
            allowed_dheading = self.max_heading_rate * dt + self.jump_tol_rad
            if abs(est.ey - self._prev.ey) > allowed_dey:
                reasons.append("ey_jump")
            if abs(est.heading - self._prev.heading) > allowed_dheading:
                reasons.append("heading_jump")

        plausible = not reasons
        self._consecutive_bad = 0 if plausible else self._consecutive_bad + 1
        reject = self._consecutive_bad >= self.min_implausible_cycles
        # Only a plausible estimate updates the temporal reference, so a suspect frame
        # is compared against the last trusted one (not against another suspect frame).
        if plausible:
            self._prev = est
        reason = "ok" if plausible else "+".join(reasons)
        return PlausibilityResult(plausible=plausible, reject=reject, reason=reason)
