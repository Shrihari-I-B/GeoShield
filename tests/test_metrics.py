"""
GeoShield -- Tests for geometric metrics (frechet_analysis.py).
"""

import math
import pytest
from frechet_analysis import (
    frechet,
    resample,
    arc_lengths,
    truncate_common,
    max_pointwise,
    tail_offsets,
)


class TestFrechetMetrics:
    def test_identical_paths_return_zero(self):
        """Discrete Frechet distance between identical paths must be 0.0."""
        P = [(float(x), 0.0) for x in range(20)]
        assert frechet(P, P) == pytest.approx(0.0, abs=1e-9)

    def test_parallel_offset_returns_exact_distance(self):
        """Two parallel straight lines offset by d must have Frechet distance == d."""
        d = 3.5
        P = [(float(x), 0.0) for x in range(50)]
        Q = [(float(x), d) for x in range(50)]
        assert frechet(P, Q) == pytest.approx(d, abs=1e-6)

    def test_empty_paths_return_nan(self):
        """Empty paths must return NaN."""
        assert math.isnan(frechet([], [(0.0, 0.0)]))
        assert math.isnan(frechet([(0.0, 0.0)], []))

    def test_single_point_paths(self):
        """Frechet distance between single-point paths is their Euclidean distance."""
        p1 = [(0.0, 0.0)]
        p2 = [(3.0, 4.0)]
        assert frechet(p1, p2) == pytest.approx(5.0, abs=1e-9)

    def test_symmetry(self):
        """Frechet(P, Q) must equal Frechet(Q, P)."""
        P = [(0.0, 0.0), (1.0, 2.0), (3.0, 5.0)]
        Q = [(0.0, 0.5), (1.5, 1.8), (3.2, 4.9)]
        assert frechet(P, Q) == pytest.approx(frechet(Q, P), abs=1e-9)


class TestArcLengthAndTruncation:
    def test_arc_lengths_monotonic(self):
        """Cumulative arc length must start at 0.0 and be non-decreasing."""
        pts = [(0.0, 0.0), (3.0, 4.0), (3.0, 10.0)]
        lengths = arc_lengths(pts)
        assert len(lengths) == len(pts)
        assert lengths[0] == 0.0
        assert lengths[1] == pytest.approx(5.0)
        assert lengths[2] == pytest.approx(11.0)
        assert all(lengths[i] <= lengths[i + 1] for i in range(len(lengths) - 1))

    def test_truncate_common_eliminates_endpoint_gap(self):
        """
        If path A is shorter than path B along the same line,
        untruncated Frechet shows the endpoint gap, while truncate_common
        cuts both to the shorter length, eliminating the gap.
        """
        # Path A has length 10, Path B continues to length 20
        path_a = [(float(x), 0.0) for x in range(11)]  # 0 to 10
        path_b = [(float(x), 0.0) for x in range(21)]  # 0 to 20

        # Without truncation, discrete Frechet reflects trailing endpoint difference (10.0)
        raw_df = frechet(path_a, path_b)
        assert raw_df == pytest.approx(10.0, abs=1e-6)

        # With truncation to common length
        ta, tb, limit = truncate_common(path_a, path_b)
        assert limit == pytest.approx(10.0)
        assert ta[-1] == pytest.approx((10.0, 0.0))
        assert tb[-1] == pytest.approx((10.0, 0.0))
        assert frechet(ta, tb) == pytest.approx(0.0, abs=1e-6)


class TestResampling:
    def test_resample_count_and_endpoints(self):
        """Resample must return exactly n points and preserve start and end."""
        pts = [(0.0, 0.0), (10.0, 0.0)]
        n = 5
        sampled = resample(pts, n)
        assert len(sampled) == n
        assert sampled[0] == pytest.approx((0.0, 0.0))
        assert sampled[-1] == pytest.approx((10.0, 0.0))
        assert sampled[2] == pytest.approx((5.0, 0.0))

    def test_resample_uniform_spacing(self):
        """Points from resample on a straight line must be uniformly spaced."""
        pts = [(0.0, 0.0), (0.0, 100.0)]
        n = 11
        sampled = resample(pts, n)
        step = 100.0 / (n - 1)
        for i, (x, y) in enumerate(sampled):
            assert x == pytest.approx(0.0)
            assert y == pytest.approx(i * step)


class TestPointwiseMetrics:
    def test_max_pointwise_parallel(self):
        d = 2.0
        P = [(float(x), 0.0) for x in range(10)]
        Q = [(float(x), d) for x in range(10)]
        assert max_pointwise(P, Q) == pytest.approx(d, abs=1e-6)

    def test_tail_offsets(self):
        P = [(float(x), 0.0) for x in range(20)]
        Q = [(float(x), 0.0) for x in range(15)]
        # Last 5 points of P (15..19) are beyond Q's end (14)
        offsets = tail_offsets(P, Q, k=5)
        assert len(offsets) == 5
        # The offset from (15..19) to Q's nearest point (14) grows monotonically
        assert offsets == [1.0, 2.0, 3.0, 4.0, 5.0]
