"""
visual_domain_randomization — per-episode visual-degradation sampler for E-training.

Track 'E' (D-41/D-42): SR-012 mitigates H-10 partly through a *training constraint* —
the camera policy is trained under randomised visual degradations so it is robust
across the SC-PERT-04..06 envelope (glare, low-light, motion blur). This module is
the **sampler**: given a numpy ``Generator`` it draws a per-episode degradation spec
(mode + level), which the training env then applies to each camera frame via
:func:`cobraflex_rl.visual_degradation.degrade`.

Pure (numpy ``Generator`` + stdlib), deterministic given the RNG, host-testable. The
Gazebo camera env that draws a spec at ``reset()`` and applies it in the observation
bridge is the Ubuntu part.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from cobraflex_rl.visual_degradation import MODES, degrade


@dataclass(frozen=True)
class DegradationSpec:
    """A drawn per-episode degradation. ``mode is None`` means a clean frame."""

    mode: Optional[str]
    level: float

    @property
    def is_clean(self) -> bool:
        return self.mode is None


@dataclass(frozen=True)
class DomainRandomizationConfig:
    """Sampler configuration.

    Attributes
    ----------
    p_degrade:
        Probability that an episode receives a degradation at all (otherwise clean).
    modes:
        Degradation modes to draw from (a subset of
        :data:`cobraflex_rl.visual_degradation.MODES`).
    level_range:
        ``(low, high)`` within ``[0, 1]`` for the sampled intensity.
    """

    p_degrade: float = 0.5
    modes: Tuple[str, ...] = MODES
    level_range: Tuple[float, float] = (0.2, 1.0)

    def __post_init__(self) -> None:
        if not 0.0 <= self.p_degrade <= 1.0:
            raise ValueError("p_degrade must be in [0, 1]")
        if not self.modes:
            raise ValueError("modes must be non-empty")
        unknown = [m for m in self.modes if m not in MODES]
        if unknown:
            raise ValueError(f"unknown modes {unknown}; valid modes are {MODES}")
        lo, hi = self.level_range
        if not (0.0 <= lo <= hi <= 1.0):
            raise ValueError("level_range must satisfy 0 <= low <= high <= 1")


class VisualDomainRandomizer:
    """Draws per-episode :class:`DegradationSpec`s and applies them to camera frames."""

    def __init__(self, config: Optional[DomainRandomizationConfig] = None) -> None:
        self.config = config or DomainRandomizationConfig()

    def sample(self, rng: np.random.Generator) -> DegradationSpec:
        """Draw a per-episode degradation spec using the numpy ``Generator`` ``rng``.

        Deterministic given the generator's state, so a seeded training run is
        reproducible (cf. the F3 seed/reproducibility policy, docs/09 §7.2.7).
        """
        if rng.random() >= self.config.p_degrade:
            return DegradationSpec(mode=None, level=0.0)
        mode = self.config.modes[int(rng.integers(0, len(self.config.modes)))]
        lo, hi = self.config.level_range
        level = float(rng.uniform(lo, hi))
        return DegradationSpec(mode=mode, level=level)

    def apply(self, img: np.ndarray, spec: DegradationSpec) -> np.ndarray:
        """Apply a spec to a frame; a clean spec returns an unchanged copy."""
        if spec.is_clean:
            return img.copy()
        return degrade(img, spec.mode, spec.level)
