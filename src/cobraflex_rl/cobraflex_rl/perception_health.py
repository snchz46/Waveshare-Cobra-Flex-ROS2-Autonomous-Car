"""
perception_health — pure-logic perception-health monitor for track 'E'.

Track 'E' (end-to-end front-camera, decisions D-41/D-42): the camera *policy* can
lose a valid lane percept (hazard H-11) through occlusion, absent lane features, or
a stale/dropped camera frame. SR-013 requires the system to degrade to a controlled
safe state in that case.

Per D-42 the cage stays **camera-agnostic**, so this monitor is an *external
supervisor*: it consumes the perception signal and, when perception is invalid,
raises the cage's C-05 perception-health trigger (Trigger 8, ``docs/04`` §C-05),
exactly as ``/external_stop`` raises C-05's Trigger 6. The cage never reads the
camera; it only receives this boolean trigger.

This module is the supervisor's **pure decision logic** — no ROS, host-testable. The
ROS node that subscribes to the camera / health topic and forwards the trigger to
the cage lives on the Ubuntu+Jazzy host (deferred, cf. D-42 GE2).
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class PerceptionSample:
    """One control cycle's perception-health inputs.

    Attributes
    ----------
    timestamp_s:
        Acquisition time of the most recent camera frame (seconds).
    frame_received:
        Whether a new frame arrived this cycle (``False`` = dropout/freeze).
    confidence:
        Lane-detection confidence in ``[0, 1]`` for the most recent frame
        (ignored when ``frame_received`` is ``False``).
    feature_count:
        Number of lane features detected this cycle (ignored when no frame).
    """

    timestamp_s: float
    frame_received: bool
    confidence: float
    feature_count: int


@dataclass(frozen=True)
class HealthResult:
    """Per-cycle monitor output."""

    valid: bool
    emergency: bool  # raise the C-05 perception-health trigger this cycle
    reason: str


class PerceptionHealthMonitor:
    """Decide, per cycle, whether camera perception is valid and whether to raise the
    C-05 perception-health emergency trigger (SR-013, H-11).

    A cycle's perception is *invalid* if any of:

    * the most recent frame is **stale** — ``now - timestamp_s > staleness_max_s`` —
      or no frame has been received for that long (dropout);
    * ``confidence < min_confidence`` (occlusion / wash-out);
    * ``feature_count < min_features`` (absent lane features).

    The emergency trigger is raised only after ``min_invalid_cycles`` *consecutive*
    invalid cycles, so a single-frame glitch does not trip it — mirroring C-05's
    persistence requirement (cf. SR-005 ``Δt_max``). The monitor reports
    instantaneous health; the latch and the explicit reset are C-05's responsibility
    (``docs/04`` §C-05, "On deactivation").
    """

    def __init__(self, *, staleness_max_s: float = 0.2, min_confidence: float = 0.3,
                 min_features: int = 1, min_invalid_cycles: int = 2) -> None:
        if staleness_max_s <= 0:
            raise ValueError("staleness_max_s must be > 0")
        if not 0.0 <= min_confidence <= 1.0:
            raise ValueError("min_confidence must be in [0, 1]")
        if min_features < 0:
            raise ValueError("min_features must be >= 0")
        if min_invalid_cycles < 1:
            raise ValueError("min_invalid_cycles must be >= 1")
        self.staleness_max_s = float(staleness_max_s)
        self.min_confidence = float(min_confidence)
        self.min_features = int(min_features)
        self.min_invalid_cycles = int(min_invalid_cycles)
        self._consecutive_invalid = 0
        self._last_frame_time: float | None = None

    def reset(self) -> None:
        """Drop accumulated state (e.g. between episodes / after an explicit reset)."""
        self._consecutive_invalid = 0
        self._last_frame_time = None

    def update(self, sample: PerceptionSample, now_s: float) -> HealthResult:
        """Advance the monitor by one cycle and return the health verdict."""
        if sample.frame_received:
            self._last_frame_time = sample.timestamp_s

        if self._last_frame_time is None:
            stale = True  # no frame ever received
        else:
            stale = (now_s - self._last_frame_time) > self.staleness_max_s

        reasons = []
        if stale:
            reasons.append("stale_or_dropped_frame")
        # confidence / feature checks only apply to a frame that actually arrived;
        # a single missing frame within the staleness budget is not yet a fault.
        if sample.frame_received and not stale:
            if sample.confidence < self.min_confidence:
                reasons.append("low_confidence")
            if sample.feature_count < self.min_features:
                reasons.append("insufficient_features")

        valid = not reasons
        self._consecutive_invalid = 0 if valid else self._consecutive_invalid + 1
        emergency = self._consecutive_invalid >= self.min_invalid_cycles
        reason = "ok" if valid else "+".join(reasons)
        return HealthResult(valid=valid, emergency=emergency, reason=reason)
