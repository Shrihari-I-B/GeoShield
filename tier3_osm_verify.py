#!/usr/bin/env python3
"""GeoShield Tier 3 -- cross-verification against OpenStreetMap.

OSM is the independent witness; the HD map is the artefact under test. Never
both. This module matches HD lanelets to OSM ways geometrically, then reports
which attributes OSM is actually able to witness for them.

Matching is point-to-polyline, not midpoint-to-midpoint: an OSM way may be
300 m long where a lanelet is 30 m, so comparing midpoints penalises a lanelet
that sits perfectly on a long way. Headings are compared modulo 180 degrees --
a two-way road's digitisation direction is arbitrary, and an HD map carries one
lanelet per direction.

Tier 3's recall on any attribute is bounded above by that attribute's coverage
in OSM. You cannot witness what is not tagged. Reporting that ceiling is part
of the tier's job.
"""

import argparse
import json
import math
import os
from typing import Optional

LAT0 = 35.693          # reference latitude for the equirectangular projection
M_PER_DEG_LAT = 111320.0


def _projector(lat0: float):
    mlon = M_PER_DEG_LAT * math.cos(math.radians(lat0))
    def xy(p):
        return (p[1] * mlon, p[0] * M_PER_DEG_LAT)
    return xy


def _pt_seg(p, a, b) -> float:
    px, py = p; ax, ay = a; bx, by = b
    dx, dy = bx - ax, by - ay
    if dx == 0.0 and dy == 0.0:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _poly_dist(p, poly) -> float:
    if len(poly) == 1:
        return math.hypot(p[0] - poly[0][0], p[1] - poly[0][1])
    return min(_pt_seg(p, poly[i], poly[i + 1]) for i in range(len(poly) - 1))


def _heading(poly) -> float:
    a, b = poly[0], poly[-1]
    return math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])) % 180.0


def _hdiff(a: float, b: float) -> float:
    d = abs(a - b) % 180.0
    return min(d, 180.0 - d)


def match(hd_segs, osm_segs, max_dist=25.0, max_heading=35.0, lat0=LAT0):
    """{hd_segment_id: (osm_segment, distance_m, heading_diff_deg)}"""
    xy = _projector(lat0)
    osm = []
    for s in osm_segs:
        if not s.geometry or len(s.geometry) < 2:
            continue
        poly = [xy(p) for p in s.geometry]
        xs = [q[0] for q in poly]; ys = [q[1] for q in poly]
        osm.append((poly, _heading(poly), min(xs), max(xs), min(ys), max(ys), s))

    out = {}
    for h in hd_segs:
        if not h.geometry or len(h.geometry) < 2:
            continue
        hp = [xy(p) for p in h.geometry]
        mid = hp[len(hp) // 2]
        hh = _heading(hp)
        best = None
        for poly, oh, x0, x1, y0, y1, s in osm:
            if mid[0] < x0 - max_dist or mid[0] > x1 + max_dist:
                continue
            if mid[1] < y0 - max_dist or mid[1] > y1 + max_dist:
                continue
            hd_ = _hdiff(hh, oh)
            if hd_ > max_heading:
                continue
            d = _poly_dist(mid, poly)
            if d <= max_dist and (best is None or d < best[1]):
                best = (s, d, hd_)
        if best:
            out[h.segment_id] = best
    return out


ATTRS = ("speed_limit_kph", "oneway", "lanes", "width_m", "tunnel", "bridge")


def coverage(hd_segs, pairs) -> dict:
    """What OSM can actually witness for the lanelets we matched."""
    rep = {"hd_total": len(hd_segs), "matched": len(pairs), "attrs": {}}
    for a in ATTRS:
        have = sum(1 for (s, _, _) in pairs.values() if getattr(s, a, None) is not None)
        rep["attrs"][a] = {
            "witnessed": have,
            "of_matched": round(have / len(pairs), 4) if pairs else 0.0,
            "of_map": round(have / len(hd_segs), 4) if hd_segs else 0.0,
        }
    return rep


def disagreements(hd_segs, pairs, speed_tol_kph: float = 1.0) -> dict:
    """Where the HD map contradicts the witness, for attributes OSM carries.

    Run this on a CLEAN map first. Whatever it reports there is Tier 3's
    false-positive floor: honest disagreement between two independently
    produced datasets, not tampering.
    """
    by = {h.segment_id: h for h in hd_segs}
    out = {"checked": {}, "disagree": {}, "ids": {}}
    for a in ("oneway", "lanes", "speed_limit_kph", "tunnel", "bridge"):
        checked, bad = 0, []
        for sid, (osm_seg, _d, _h) in pairs.items():
            hv = getattr(by[sid], a, None)
            ov = getattr(osm_seg, a, None)
            if hv is None or ov is None:
                continue
            checked += 1
            if a == "speed_limit_kph":
                if abs(float(hv) - float(ov)) > speed_tol_kph:
                    bad.append((sid, hv, ov))
            elif hv != ov:
                bad.append((sid, hv, ov))
        out["checked"][a] = checked
        out["disagree"][a] = len(bad)
        out["ids"][a] = bad
    return out


def main():
    ap = argparse.ArgumentParser(description="GeoShield Tier 3 -- OSM matching")
    ap.add_argument("--map", required=True, help="HD Lanelet2 .osm under test")
    ap.add_argument("--area", default="nishi-shinjuku", help="built-in OSM box")
    ap.add_argument("--max-dist", type=float, default=25.0)
    ap.add_argument("--max-heading", type=float, default=35.0)
    ap.add_argument("--sweep", action="store_true", help="try several tolerances")
    ap.add_argument("--out")
    a = ap.parse_args()

    from lanelet2_adapter import load as hd_load
    from osm_adapter import load as osm_load, AREAS

    hd = hd_load(a.map)
    box = AREAS[a.area]
    osm = osm_load(box)
    lat0 = (box[0] + box[2]) / 2.0
    print(f"HD lanelets {len(hd)}   OSM ways {len(osm)}   lat0 {lat0:.4f}")

    if a.sweep:
        print(f"\n{'max_dist':>9}{'max_hdg':>9}{'matched':>10}{'rate':>8}")
        for d in (10.0, 15.0, 20.0, 25.0, 35.0):
            for hdg in (20.0, 35.0, 90.0):
                m = match(hd, osm, d, hdg, lat0)
                print(f"{d:>9.0f}{hdg:>9.0f}{len(m):>10}{len(m)/len(hd):>8.3f}")

    pairs = match(hd, osm, a.max_dist, a.max_heading, lat0)
    rep = coverage(hd, pairs)
    print(f"\nmatched {rep['matched']} of {rep['hd_total']} lanelets "
          f"({rep['matched']/rep['hd_total']:.1%}) at <={a.max_dist:.0f} m, "
          f"<={a.max_heading:.0f} deg")
    if pairs:
        ds = sorted(d for (_, d, _) in pairs.values())
        print(f"  match distance: median {ds[len(ds)//2]:.1f} m  "
              f"p90 {ds[int(.9*len(ds))]:.1f} m  max {ds[-1]:.1f} m")

    print(f"\n{'attribute':<18}{'witnessed':>11}{'of matched':>12}{'of map':>9}"
          f"   <- Tier 3 recall ceiling")
    for k, v in rep["attrs"].items():
        print(f"  {k:<16}{v['witnessed']:>11}{v['of_matched']:>12.3f}"
              f"{v['of_map']:>9.3f}")

    if a.out:
        json.dump(rep, open(a.out, "w"), indent=2)
        print(f"\nwrote {a.out}")


if __name__ == "__main__":
    main()
