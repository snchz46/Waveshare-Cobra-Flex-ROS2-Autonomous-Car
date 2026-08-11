"""
cage_perception — the cage's perception supervisor for track 'E' (D-43).

Composes, per control cycle, the three host-tested kernels around the CV
lane-estimator into the cage-facing contract:

* :class:`~cobraflex_rl.cv_lane_estimator.CvLaneEstimator` produces the lane
  estimate from the (possibly degraded) camera frame;
* :class:`~cobraflex_rl.perception_health.PerceptionHealthMonitor` (SR-013,
  H-11) judges loss — stale/dropped frame, low confidence, missing features;
* :class:`~cobraflex_rl.lane_plausibility.LanePlausibilityCheck` (SR-014,
  H-12) judges a *suspect* estimate — out-of-ODD geometry or a temporal jump.

The output is exactly what `SafetyCageNode.step` consumes: the cage `state`
built from the CV estimate (when there is a trustworthy one) and the
``perception_invalid`` boolean for C-05 Trigger 8. The cage itself never sees
the camera (camera-agnostic, docs/04 §C-05).

Pure logic (no ROS) — host-testable; used in-process by `gazebo_lane_env` and
by the ROS estimator node alike.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from .cv_lane_estimator import (
    CvLaneEstimate,
    CvLaneEstimator,
)
from .lane_plausibility import LaneEstimate, LanePlausibilityCheck
from .perception_health import PerceptionHealthMonitor, PerceptionSample


@dataclass(frozen=True)
class CagePerceptionResult:
    """Per-cycle supervisor verdict.

    estimate:
        The CV lane estimate this cycle (``ok`` may be False).
    state_available:
        True when the estimate is usable as the cage state this cycle.
    perception_invalid:
        The C-05 Trigger 8 flag (latched persistence handled by the monitors'
        consecutive-cycle counters; the emergency latch itself is C-05's).
    health_reason / plausibility_reason:
        Diagnostic strings for the cycle log.
    """

    estimate: CvLaneEstimate
    state_available: bool
    perception_invalid: bool
    health_reason: str
    plausibility_reason: str


class CagePerceptionSupervisor:
    """Estimator + health (SR-013) + plausibility (SR-014) composed per cycle.

    Defaults are the live-tuned E2 values (see inline rationale); pass explicit
    kernels to override in tests.
    """

    def __init__(
        self,
        estimator: Optional[CvLaneEstimator] = None,
        health: Optional[PerceptionHealthMonitor] = None,
        plausibility: Optional[LanePlausibilityCheck] = None,
    ) -> None:
        self.estimator = estimator or CvLaneEstimator()
        if health is None:
            # min_confidence must admit the estimator's single-line fallback
            # (confidence = 0.5 × row coverage, floor ≈ 0.08 at the minimum
            # cluster size): it is a *degraded-but-valid* mode, not a loss.
            # Genuine loss still gates — a failed estimate carries
            # confidence 0.0. min_invalid_cycles=4 (0.4 s at 10 Hz) bridges
            # the ~2-cycle dash-gap blind stretches measured live at the oval
            # curve apex while staying inside the cage's own 5-cycle
            # missing-state budget (n_missing_max_cycles, C-05 Trigger 5) —
            # a real loss still stops within ~0.5 s.
            health = PerceptionHealthMonitor(
                min_confidence=0.10, min_invalid_cycles=4
            )
        self.health = health
        if plausibility is None:
            # The SR-014 lane-width window must equal the estimator's own
            # pair-acceptance window. With the checker's generic default
            # (0.20–0.80 m) every pair the estimator accepts in
            # [nominal−tol, 0.20) was permanently rejected — a dead zone that
            # deadlocked the cage into its no-state path (found live at E2).
            cfg = self.estimator.config
            plausibility = LanePlausibilityCheck(
                lane_width_range=(
                    cfg.lane_width_nominal_m - cfg.lane_width_tol_m,
                    cfg.lane_width_nominal_m + cfg.lane_width_tol_m,
                ),
                # ODD KAPPA_MAX is 1.25 1/m (the oval's U-turns); the
                # estimator's quadratic-fit curvature carries noise on top, so
                # the generic 1.5 default rejected real curve entries (found
                # live at E2). 3.0 still rejects absurd geometry while
                # admitting KAPPA_MAX plus the measured estimator noise.
                curvature_max=3.0,
            )
        self.plausibility = plausibility

    def reset(self) -> None:
        """Per-episode reset (fresh persistence counters, no stale reference)."""
        self.health.reset()
        self.plausibility.reset()
        # Clear the estimator's temporal heading-consistency window (T3, D-62)
        # so a new rollout cannot inherit the previous episode's confirmation.
        estimator_reset = getattr(self.estimator, "reset", None)
        if callable(estimator_reset):
            estimator_reset()

    def update(
        self,
        frame_bgr: Optional[np.ndarray],
        *,
        frame_timestamp_s: float,
        now_s: float,
        speed_mps: float,
    ) -> CagePerceptionResult:
        """Run one cycle: estimate from ``frame_bgr`` (None = no frame arrived),
        then judge health (SR-013) and plausibility (SR-014)."""
        if frame_bgr is not None:
            estimate = self.estimator.estimate(frame_bgr)
        else:
            estimate = CvLaneEstimate(ok=False, reason="no_frame")

        sample = PerceptionSample(
            timestamp_s=frame_timestamp_s,
            frame_received=frame_bgr is not None,
            confidence=estimate.confidence,
            feature_count=estimate.feature_count if estimate.ok else 0,
        )
        health = self.health.update(sample, now_s=now_s)

        plaus_reason = "not_evaluated"
        plaus_reject = False
        state_available = False
        if estimate.ok:
            plaus = self.plausibility.update(
                LaneEstimate(
                    ey=estimate.ey,
                    heading=estimate.epsi,
                    lane_width=estimate.lane_width,
                    curvature=estimate.curvature,
                    timestamp_s=frame_timestamp_s,
                ),
                speed_mps=speed_mps,
                now_s=now_s,
            )
            plaus_reason = plaus.reason
            plaus_reject = plaus.reject
            state_available = plaus.plausible and health.valid

        return CagePerceptionResult(
            estimate=estimate,
            state_available=state_available,
            perception_invalid=bool(health.emergency or plaus_reject),
            health_reason=health.reason,
            plausibility_reason=plaus_reason,
        )
