#!/usr/bin/env python3
"""
GeoShield -- Phase 4a: feature extraction.

Turns RoadSegments into fixed-length numeric vectors for Tier 2.

WHY THIS FILE MATTERS MORE THAN THE MODEL
-----------------------------------------
A model can only find patterns in what you show it. Phase 3 measured the
ceiling for detecting ramp attacks with ONE statistic: max achievable F1 was
0.22, because attacked chains fall inside the 90th percentile of clean chains
on the trend statistic.

The ML layer's only hope of beating that ceiling is the JOINT distribution --
a lanelet that is unremarkable on trend, unremarkable on taper, and
unremarkable on asymmetry, but unusual in all three *together*. That joint
structure exists only if we compute the right per-axis features here.

FEATURE FAMILIES
----------------
(a) intrinsic   -- the lanelet alone: width, length, curvature
(b) relational  -- how it compares to its neighbours   <-- where tampering lives
(c) chain       -- statistics of the paths it belongs to
(d) missingness -- what is absent is informative

Family (b) dominates. A 4.5 m lane is normal; a 4.5 m lane between two 3.0 m
lanes is not. Tampering is a CONTEXT violation, not a VALUE violation, so a
model shown only intrinsic features will barely beat random.
"""

from __future__ import annotations

import math
import statistics
from typing import Optional

from road_segment import RoadSegment


FEATURE_NAMES = [
    # (a) intrinsic
    "width", "length", "curvature", "speed",
    "width_taper", "width_std_rel",
    # (b) relational -- successor
    "d_width_succ", "rel_width_succ", "grad_width_succ",
    "d_speed_succ", "centreline_gap_succ",
    # (b) relational -- predecessor
    "d_width_pred", "rel_width_pred", "grad_width_pred",
    # (b) relational -- local neighbourhood
    "width_vs_nbr_median", "width_z_local", "speed_vs_nbr_median",
    "n_succ", "n_pred", "degree_imbalance",
    # (c) chain context
    "chain_trend_max", "chain_consistency_max", "chain_span_max",
    "chain_count",
    # (d) flags / missingness
    "is_junction", "is_road", "missing_speed", "missing_width",
]


def _safe(x, default=0.0):
    return default if x is None or (isinstance(x, float) and math.isnan(x)) else x


def _haversine(a, b) -> float:
    R = 6_371_000.0
    la1, lo1, la2, lo2 = map(math.radians, (a[0], a[1], b[0], b[1]))
    h = (math.sin((la2 - la1) / 2) ** 2
         + math.cos(la1) * math.cos(la2) * math.sin((lo2 - lo1) / 2) ** 2)
    return 2 * R * math.asin(math.sqrt(h))


def _chain_stats(segs, by, length=5, max_branch=3, cap=20000):
    """
    Per-segment summary of every chain it participates in.

    Phase 3 showed a single chain's trend is not separable on its own, but as
    ONE AXIS among many it still carries signal -- so we attach the strongest
    chain evidence touching each lanelet rather than thresholding it.
    """
    def usable(x):
        return x.width_m is not None and x.highway_class in ("road", None)

    stats = {s.segment_id: {"trend": [], "consistency": [], "span": [], "n": 0}
             for s in segs}
    out = []

    def extend(chain):
        if len(out) >= cap:
            return
        if len(chain) == length:
            out.append(list(chain))
            return
        cur = chain[-1]
        nxt = [by[t] for t in cur.successors
               if t in by and usable(by[t])
               and by[t].segment_id not in {c.segment_id for c in chain}]
        if not nxt:
            return
        if len(chain) >= 2:
            drift = chain[-1].width_m - chain[-2].width_m
            nxt.sort(key=lambda t: -((t.width_m - cur.width_m) * drift))
        for t in nxt[:max_branch]:
            chain.append(t)
            extend(chain)
            chain.pop()

    for s in segs:
        if usable(s):
            extend([s])

    for chain in out:
        w = [x.width_m for x in chain]
        steps = [w[i + 1] - w[i] for i in range(len(w) - 1)]
        if not steps:
            continue
        span = w[-1] - w[0]
        sign = 1 if span >= 0 else -1
        consistency = sum(1 for d in steps if d * sign > 0) / len(steps)
        mean_w = statistics.fmean(w) or 1.0
        trend = abs(span) / mean_w * consistency
        for x in chain:
            st = stats[x.segment_id]
            st["trend"].append(trend)
            st["consistency"].append(consistency)
            st["span"].append(abs(span))
            st["n"] += 1

    return stats


def extract(segs: list[RoadSegment], chain_length: int = 5) -> tuple[list[str], list[list[float]]]:
    """Return (segment_ids, feature_matrix) aligned row-for-row."""
    by = {s.segment_id: s for s in segs}
    chains = _chain_stats(segs, by, chain_length)

    ids, rows = [], []
    for s in segs:
        f = {}
        w = s.width_m
        f["width"] = _safe(w)
        f["length"] = _safe(s.length_m)
        f["curvature"] = _safe(s.mean_curvature)
        f["speed"] = _safe(s.speed_limit_kph)

        st = s.raw_tags.get("_width_start")
        en = s.raw_tags.get("_width_end")
        sd = s.raw_tags.get("_width_std")
        f["width_taper"] = (abs(float(en) - float(st)) / w
                            if st and en and w else 0.0)
        f["width_std_rel"] = float(sd) / w if sd and w else 0.0

        # ---- successor relations
        succ = [by[t] for t in s.successors if t in by]
        if succ and w:
            ws = [t.width_m for t in succ if t.width_m]
            if ws:
                nearest = min(ws, key=lambda x: abs(x - w))
                f["d_width_succ"] = nearest - w
                f["rel_width_succ"] = (nearest - w) / w
                f["grad_width_succ"] = (nearest - w) / max(s.length_m, 1.0)
            sp = [t.speed_limit_kph for t in succ if t.speed_limit_kph]
            f["d_speed_succ"] = ((statistics.fmean(sp) - s.speed_limit_kph)
                                 if sp and s.speed_limit_kph else 0.0)
            if s.geometry:
                gaps = [_haversine(s.geometry[-1], t.geometry[0])
                        for t in succ if t.geometry]
                f["centreline_gap_succ"] = min(gaps) if gaps else 0.0

        # ---- predecessor relations
        pred = [by[t] for t in s.predecessors if t in by]
        if pred and w:
            wp = [t.width_m for t in pred if t.width_m]
            if wp:
                nearest = min(wp, key=lambda x: abs(x - w))
                f["d_width_pred"] = w - nearest
                f["rel_width_pred"] = (w - nearest) / w
                f["grad_width_pred"] = (w - nearest) / max(s.length_m, 1.0)

        # ---- local neighbourhood (both directions, 1 hop)
        nbr = succ + pred
        nw = [t.width_m for t in nbr if t.width_m]
        if nw and w:
            med = statistics.median(nw)
            f["width_vs_nbr_median"] = w - med
            sd_n = statistics.pstdev(nw) if len(nw) > 1 else 0.0
            f["width_z_local"] = (w - med) / sd_n if sd_n > 1e-6 else 0.0
        ns = [t.speed_limit_kph for t in nbr if t.speed_limit_kph]
        if ns and s.speed_limit_kph:
            f["speed_vs_nbr_median"] = s.speed_limit_kph - statistics.median(ns)

        f["n_succ"] = len(s.successors)
        f["n_pred"] = len(s.predecessors)
        f["degree_imbalance"] = len(s.successors) - len(s.predecessors)

        # ---- chain context
        c = chains.get(s.segment_id, {})
        f["chain_trend_max"] = max(c.get("trend") or [0.0])
        f["chain_consistency_max"] = max(c.get("consistency") or [0.0])
        f["chain_span_max"] = max(c.get("span") or [0.0])
        f["chain_count"] = c.get("n", 0)

        # ---- flags
        f["is_junction"] = 1.0 if "turn_direction" in s.raw_tags else 0.0
        f["is_road"] = 1.0 if s.highway_class == "road" else 0.0
        f["missing_speed"] = 1.0 if s.speed_limit_kph is None else 0.0
        f["missing_width"] = 1.0 if s.width_m is None else 0.0

        ids.append(s.segment_id)
        rows.append([float(_safe(f.get(k, 0.0))) for k in FEATURE_NAMES])

    return ids, rows


def standardise(rows: list[list[float]],
                stats: Optional[tuple[list[float], list[float]]] = None):
    """
    Zero mean, unit variance. Statistics come from the CLEAN set only --
    computing them on the tampered set would leak attack information into
    the normalisation itself.
    """
    n = len(FEATURE_NAMES)
    if stats is None:
        cols = [[r[i] for r in rows] for i in range(n)]
        mean = [statistics.fmean(c) if c else 0.0 for c in cols]
        sd = [statistics.pstdev(c) if len(c) > 1 else 1.0 for c in cols]
        sd = [x if x > 1e-9 else 1.0 for x in sd]
    else:
        mean, sd = stats
    out = [[(r[i] - mean[i]) / sd[i] for i in range(n)] for r in rows]
    return out, (mean, sd)


if __name__ == "__main__":
    import sys
    from lanelet2_adapter import load
    segs = load(sys.argv[1])
    ids, rows = extract(segs)
    print(f"{len(rows)} segments x {len(FEATURE_NAMES)} features")
    print("\nfeature ranges (raw):")
    for i, name in enumerate(FEATURE_NAMES):
        col = [r[i] for r in rows]
        print(f"  {name:<24}{min(col):>10.3f}{statistics.fmean(col):>10.3f}{max(col):>10.3f}")