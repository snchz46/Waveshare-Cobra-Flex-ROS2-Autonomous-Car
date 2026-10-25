"""Unit test of the minimum-range rule applied to the simulated scan."""

import math
from types import SimpleNamespace

from cobraflex.scan_min_range_node import apply_min_range


def test_short_returns_become_inf():
    msg = SimpleNamespace(range_min=0.015, ranges=[0.05, 0.149, 0.15, 2.0, math.inf, math.nan],
                          intensities=[1.0, 1.0, 1.0, 1.0, 0.0, 0.0])
    n = apply_min_range(msg, 0.15)
    assert n == 2
    assert math.isinf(msg.ranges[0]) and math.isinf(msg.ranges[1])
    # LaserScan ranges are float32: 0.15 is kept, as 0.15000000596
    assert math.isclose(msg.ranges[2], 0.15, rel_tol=1e-6) and msg.ranges[3] == 2.0
    assert math.isinf(msg.ranges[4]) and math.isnan(msg.ranges[5])
    assert msg.intensities[:4] == [0.0, 0.0, 1.0, 1.0]
    assert msg.range_min == 0.15


def test_scan_without_intensities_and_larger_range_min():
    msg = SimpleNamespace(range_min=0.2, ranges=[0.1, 1.0], intensities=[])
    assert apply_min_range(msg, 0.15) == 1
    assert math.isinf(msg.ranges[0]) and msg.intensities == []
    assert msg.range_min == 0.2
