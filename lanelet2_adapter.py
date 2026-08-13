#!/usr/bin/env python3
"""
GeoShield -- Track B adapter: Lanelet2 .osm HD map -> RoadSegment.

    python3 lanelet2_adapter.py map.osm --summary
    python3 lanelet2_adapter.py map.osm --widths        # the critical measurement
    python3 lanelet2_adapter.py map.osm --out lanelets.json

WHY WIDTH IS THE POINT OF THIS FILE
-----------------------------------
The Phase 0 density check measured OSM `width` coverage at 0.3% (Nishi-Shinjuku)
and 5.9% (Munich). Lane width is therefore effectively unwitnessed by OSM, which
means Sato et al.'s lane-width expansion attack cannot be caught by Tier-3
cross-verification anywhere. It has to be caught by Tier-1 self-consistency --
by noticing that a lanelet's width disagrees with its own neighbours.

That makes `width_profile()` below the load-bearing measurement of Track B.

FORMAT NOTES (Lanelet2 / Autoware)
----------------------------------
* Points        -> OSM <node>.  Autoware adds `local_x` / `local_y`, a metric
                   MGRS-frame coordinate. We prefer these over lat/lon: width is
                   a sub-metre quantity and metres beat degrees for it.
* LineStrings   -> OSM <way>.
* Lanelets      -> OSM <relation> with tag type=lanelet, holding exactly two
                   member ways with roles `left` and `right` (plus an optional
                   `centerline`).
* Direction tag -> Autoware uses `one_way`, NOT OSM's `oneway`. Different key.

Standard library only -- no lanelet2, no ROS.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
from typing import Optional

from road_segment import RoadSegment, Provenance


# ----------------------------------------------------------------------
# raw XML -> primitives
# ----------------------------------------------------------------------

class Lanelet2Map:
    """Parsed primitives from a Lanelet2 .osm file."""

    def __init__(self) -> None:
        self.nodes: dict[int, dict] = {}        # id -> {lat, lon, x, y, ele}
        self.ways: dict[int, list[int]] = {}    # id -> [node ids]
        self.way_tags: dict[int, dict] = {}
        self.lanelets: dict[int, dict] = {}     # id -> {left, right, centerline, tags}
        self.regulatory: dict[int, dict] = {}

    # -- geometry helpers ------------------------------------------------

    def use_metric(self) -> bool:
        """True when local_x/local_y are present on (most) nodes."""
        if not self.nodes:
            return False
        got = sum(1 for n in self.nodes.values() if n["x"] is not None)
        return got > 0.9 * len(self.nodes)

    def point(self, nid: int) -> Optional[tuple[float, float]]:
        """Planar coordinate for width maths: metric if available, else lat/lon."""
        n = self.nodes.get(nid)
        if n is None:
            return None
        if n["x"] is not None and n["y"] is not None:
            return (n["x"], n["y"])
        return (n["lat"], n["lon"]) if n["lat"] is not None else None

    def latlon(self, nid: int) -> Optional[tuple[float, float]]:
        n = self.nodes.get(nid)
        if n is None or n["lat"] is None:
            return None
        return (n["lat"], n["lon"])

    def way_points(self, wid: int) -> list[tuple[float, float]]:
        return [p for nid in self.ways.get(wid, [])
                if (p := self.point(nid)) is not None]

    def way_latlon(self, wid: int) -> list[tuple[float, float]]:
        return [p for nid in self.ways.get(wid, [])
                if (p := self.latlon(nid)) is not None]


def parse_file(path: str) -> Lanelet2Map:
    m = Lanelet2Map()
    root = ET.parse(path).getroot()

    for el in root.findall("node"):
        nid = int(el.get("id"))
        tags = {t.get("k"): t.get("v") for t in el.findall("tag")}
        m.nodes[nid] = {
            "lat": _f(el.get("lat")),
            "lon": _f(el.get("lon")),
            "x": _f(tags.get("local_x")),
            "y": _f(tags.get("local_y")),
            "ele": _f(tags.get("ele")),
        }

    for el in root.findall("way"):
        wid = int(el.get("id"))
        m.ways[wid] = [int(nd.get("ref")) for nd in el.findall("nd")]
        m.way_tags[wid] = {t.get("k"): t.get("v") for t in el.findall("tag")}

    for el in root.findall("relation"):
        rid = int(el.get("id"))
        tags = {t.get("k"): t.get("v") for t in el.findall("tag")}
        rtype = tags.get("type")
        roles: dict[str, list[int]] = defaultdict(list)
        for mem in el.findall("member"):
            roles[mem.get("role")].append(int(mem.get("ref")))

        if rtype == "lanelet":
            m.lanelets[rid] = {
                "left": roles["left"][0] if roles["left"] else None,
                "right": roles["right"][0] if roles["right"] else None,
                "centerline": roles["centerline"][0] if roles["centerline"] else None,
                "regulatory": roles.get("regulatory_element", []),
                "tags": tags,
            }
        elif rtype == "regulatory_element":
            m.regulatory[rid] = {"tags": tags, "roles": dict(roles)}

    return m


def _f(v) -> Optional[float]:
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


# ----------------------------------------------------------------------
# width -- the measurement everything in Track B rests on
# ----------------------------------------------------------------------

def _dist(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _resample(pts: list[tuple[float, float]], n: int) -> list[tuple[float, float]]:
    """n points spaced evenly by arc length along a polyline."""
    if len(pts) < 2:
        return pts * n
    cum, total = [0.0], 0.0
    for i in range(len(pts) - 1):
        total += _dist(pts[i], pts[i + 1])
        cum.append(total)
    if total == 0:
        return [pts[0]] * n

    out, j = [], 0
    for i in range(n):
        target = total * i / (n - 1)
        while j < len(cum) - 2 and cum[j + 1] < target:
            j += 1
        span = cum[j + 1] - cum[j]
        t = 0.0 if span == 0 else (target - cum[j]) / span
        out.append((pts[j][0] + t * (pts[j + 1][0] - pts[j][0]),
                    pts[j][1] + t * (pts[j + 1][1] - pts[j][1])))
    return out


def width_profile(left: list, right: list, samples: int = 12) -> Optional[dict]:
    """
    Width sampled along the lanelet.

    Left and right boundaries usually have different node counts and different
    node spacing, so we cannot pair them index-by-index. Instead both are
    resampled to the same number of arc-length-proportional stations, and width
    is the distance between corresponding stations.

    This is an approximation of true perpendicular width -- it is exact for
    parallel boundaries and degrades gracefully on curves. That is fine here:
    Tier-1 compares a lanelet's width against its *neighbours'* widths computed
    the same way, so a consistent bias cancels out. What matters is detecting
    the discontinuity, not certifying an absolute value.

    Returns mean / std / min / max / start / end, or None if unusable.
    """
    if len(left) < 2 or len(right) < 2:
        return None

    l = _resample(left, samples)
    r = _resample(right, samples)
    w = [_dist(a, b) for a, b in zip(l, r)]
    if not w:
        return None

    return {
        "mean": statistics.fmean(w),
        "std": statistics.pstdev(w) if len(w) > 1 else 0.0,
        "min": min(w),
        "max": max(w),
        "start": w[0],
        "end": w[-1],
        "samples": w,
    }


def centerline(left: list, right: list, samples: int = 12) -> list[tuple[float, float]]:
    l, r = _resample(left, samples), _resample(right, samples)
    return [((a[0] + b[0]) / 2, (a[1] + b[1]) / 2) for a, b in zip(l, r)]


# ----------------------------------------------------------------------
# topology: shared boundary endpoints imply connectivity
# ----------------------------------------------------------------------

def build_topology(m: Lanelet2Map) -> dict[int, dict[str, list[int]]]:
    """
    Successor = a lanelet whose boundaries *start* where ours *end*.

    Lanelet2 encodes connectivity implicitly through shared linestring
    endpoints rather than explicit links, so we index by the (left_end,
    right_end) node pair and look for lanelets starting at that same pair.
    """
    starts: dict[tuple, list[int]] = defaultdict(list)
    ends: dict[int, tuple] = {}

    for lid, ll in m.lanelets.items():
        lw, rw = ll["left"], ll["right"]
        if lw not in m.ways or rw not in m.ways:
            continue
        ln, rn = m.ways[lw], m.ways[rw]
        if not ln or not rn:
            continue
        starts[(ln[0], rn[0])].append(lid)
        ends[lid] = (ln[-1], rn[-1])

    topo = {lid: {"pred": [], "succ": []} for lid in m.lanelets}
    for lid, endkey in ends.items():
        for nxt in starts.get(endkey, []):
            if nxt != lid:
                topo[lid]["succ"].append(nxt)
                topo[nxt]["pred"].append(lid)
    return topo


# ----------------------------------------------------------------------
# lanelet -> RoadSegment
# ----------------------------------------------------------------------

def parse_speed(tags: dict) -> Optional[float]:
    v = tags.get("speed_limit")
    if v is None:
        return None
    try:
        s = float(str(v).replace("km/h", "").replace("kmh", "").strip())
    except ValueError:
        return None
    # Some maps store m/s. Anything under 20 is implausible as km/h for a road.
    return round(s * 3.6, 1) if s < 20 else s


def to_segments(m: Lanelet2Map, samples: int = 12) -> list[RoadSegment]:
    topo = build_topology(m)
    metric = m.use_metric()
    out: list[RoadSegment] = []

    for lid, ll in m.lanelets.items():
        lw, rw = ll["left"], ll["right"]
        if lw is None or rw is None:
            continue

        lpts, rpts = m.way_points(lw), m.way_points(rw)
        prof = width_profile(lpts, rpts, samples) if metric else None

        # Geometry for the RoadSegment stays lat/lon so both adapters agree.
        ll_l, ll_r = m.way_latlon(lw), m.way_latlon(rw)
        geom = (centerline(ll_l, ll_r, samples)
                if len(ll_l) >= 2 and len(ll_r) >= 2 else [])

        tags = ll["tags"]
        seg = RoadSegment(
            segment_id=f"lanelet:{lid}",
            source="lanelet2",
            geometry=geom,
            speed_limit_kph=parse_speed(tags),
            # Autoware uses `one_way`, not OSM's `oneway`.
            oneway=(tags.get("one_way") not in ("no", "false")
                    if "one_way" in tags else None),
            tunnel=None,          # not a Lanelet2 concept; comes from OSM side
            bridge=None,
            lanes=1,              # a lanelet is by definition a single lane
            width_m=round(prof["mean"], 3) if prof else None,
            highway_class=tags.get("subtype"),
            predecessors=[f"lanelet:{p}" for p in topo[lid]["pred"]],
            successors=[f"lanelet:{s}" for s in topo[lid]["succ"]],
            provenance=Provenance(),
            raw_tags=tags,
        )

        # Width detail beyond the single mean -- Tier 1 needs the shape, not
        # just the average, to catch tapered or ramped tampering.
        if prof:
            seg.raw_tags = dict(tags)
            seg.raw_tags["_width_std"] = round(prof["std"], 3)
            seg.raw_tags["_width_min"] = round(prof["min"], 3)
            seg.raw_tags["_width_max"] = round(prof["max"], 3)
            seg.raw_tags["_width_start"] = round(prof["start"], 3)
            seg.raw_tags["_width_end"] = round(prof["end"], 3)

        # Metric-frame centreline and boundary endpoints.
        #
        # WHY: geometry[] is lat/lon, where a 0.5 m lateral shift is ~5e-6
        # degrees -- below the precision written into the file, so it rounds
        # to zero. That is why centreline_gap_succ measured 0.000 on every
        # segment. Displacing one boundary (what our injector does, and what
        # a Vector Map Builder edit does) moves the centreline sideways while
        # the road "widens"; in metres that shift is plainly visible.
        if metric and len(lpts) >= 2 and len(rpts) >= 2:
            cl = centerline(lpts, rpts, samples)
            seg.raw_tags["_cl_start_x"] = round(cl[0][0], 4)
            seg.raw_tags["_cl_start_y"] = round(cl[0][1], 4)
            seg.raw_tags["_cl_end_x"] = round(cl[-1][0], 4)
            seg.raw_tags["_cl_end_y"] = round(cl[-1][1], 4)
            seg.raw_tags["_lb_end_x"] = round(lpts[-1][0], 4)
            seg.raw_tags["_lb_end_y"] = round(lpts[-1][1], 4)
            seg.raw_tags["_rb_end_x"] = round(rpts[-1][0], 4)
            seg.raw_tags["_rb_end_y"] = round(rpts[-1][1], 4)
            seg.raw_tags["_lb_start_x"] = round(lpts[0][0], 4)
            seg.raw_tags["_lb_start_y"] = round(lpts[0][1], 4)
            seg.raw_tags["_rb_start_x"] = round(rpts[0][0], 4)
            seg.raw_tags["_rb_start_y"] = round(rpts[0][1], 4)

        out.append(seg)

    return out


def load(path: str, samples: int = 12) -> list[RoadSegment]:
    return to_segments(parse_file(path), samples)


# ----------------------------------------------------------------------
# reports
# ----------------------------------------------------------------------

def report_summary(m: Lanelet2Map, segs: list[RoadSegment], path: str) -> None:
    print(f"\n=== Lanelet2 map: {path} ===")
    print(f"nodes            : {len(m.nodes)}")
    print(f"ways             : {len(m.ways)}")
    print(f"lanelets         : {len(m.lanelets)}")
    print(f"regulatory elems : {len(m.regulatory)}")
    print(f"coordinate frame : {'metric (local_x/local_y)' if m.use_metric() else 'lat/lon only'}")

    if not m.use_metric():
        print("\n  ! No local_x/local_y found. Widths would be in degrees, not")
        print("    metres, and unusable. Check this is an Autoware-exported map.")

    print(f"\nparsed segments  : {len(segs)}")
    for k in ("width_m", "speed_limit_kph", "oneway", "highway_class"):
        got = sum(1 for s in segs if getattr(s, k) is not None)
        pct = 100 * got / len(segs) if segs else 0
        print(f"  {k:<17}{got:>6} {pct:>6.1f}%")

    linked = sum(1 for s in segs if s.successors)
    print(f"  with successors  {linked:>6} {100*linked/len(segs) if segs else 0:>6.1f}%")

    subs = defaultdict(int)
    for s in segs:
        subs[s.highway_class or "(none)"] += 1
    print("\nsubtypes:")
    for k, v in sorted(subs.items(), key=lambda kv: -kv[1])[:8]:
        print(f"  {k:<24}{v:>6}")


def report_widths(segs: list[RoadSegment]) -> None:
    w = [s.width_m for s in segs if s.width_m is not None]
    if not w:
        print("\nNo widths computed -- map has no metric coordinates.")
        return

    w.sort()
    def pct(p): return w[min(int(p / 100 * len(w)), len(w) - 1)]

    print(f"\n=== Lane width distribution ({len(w)} lanelets) ===")
    print(f"  min     {min(w):6.2f} m")
    print(f"  p05     {pct(5):6.2f} m")
    print(f"  median  {pct(50):6.2f} m")
    print(f"  mean    {statistics.fmean(w):6.2f} m")
    print(f"  p95     {pct(95):6.2f} m")
    print(f"  max     {max(w):6.2f} m")
    print(f"  stdev   {statistics.pstdev(w):6.2f} m")

    print("\n  histogram")
    lo, hi = math.floor(min(w)), math.ceil(max(w))
    for b in range(lo, min(hi, lo + 12)):
        c = sum(1 for x in w if b <= x < b + 1)
        if c:
            print(f"  {b:>2}-{b+1:<2} m {'#' * min(int(50 * c / len(w)), 50):<50} {c}")

    # Neighbour deltas: the Tier-1 signal that has to carry Sato's attack.
    by_id = {s.segment_id: s for s in segs}
    deltas = [abs(s.width_m - by_id[succ].width_m)
              for s in segs if s.width_m is not None
              for succ in s.successors
              if succ in by_id and by_id[succ].width_m is not None]

    if deltas:
        deltas.sort()
        def dp(p): return deltas[min(int(p / 100 * len(deltas)), len(deltas) - 1)]
        print(f"\n=== Width delta to successor ({len(deltas)} pairs) ===")
        print(f"  median  {dp(50):6.3f} m")
        print(f"  p90     {dp(90):6.3f} m")
        print(f"  p99     {dp(99):6.3f} m   <- natural variation ceiling")
        print(f"  max     {max(deltas):6.3f} m")
        print(f"\n  Sato's smallest effective attack was +0.5 m (3.0 -> 3.5 m).")
        print(f"  A Tier-1 threshold sits between p99 and 0.5 m; the wider that")
        print(f"  gap, the more separable tampering is from honest variation.")


def main() -> None:
    ap = argparse.ArgumentParser(description="GeoShield Lanelet2 adapter")
    ap.add_argument("path", help="Lanelet2 .osm file")
    ap.add_argument("--summary", action="store_true")
    ap.add_argument("--widths", action="store_true")
    ap.add_argument("--samples", type=int, default=12)
    ap.add_argument("--out", help="write RoadSegments as JSON")
    a = ap.parse_args()

    try:
        m = parse_file(a.path)
    except ET.ParseError as exc:
        sys.exit(f"XML parse failed: {exc}")
    except FileNotFoundError:
        sys.exit(f"not found: {a.path}")

    if not m.lanelets:
        sys.exit("No type=lanelet relations found -- is this a Lanelet2 map?")

    segs = to_segments(m, a.samples)

    if a.summary or not (a.widths or a.out):
        report_summary(m, segs, a.path)
    if a.widths:
        report_widths(segs)
    if a.out:
        with open(a.out, "w") as fh:
            json.dump([s.to_dict() for s in segs], fh, indent=2)
        print(f"\nwrote {a.out} ({len(segs)} segments)", file=sys.stderr)


if __name__ == "__main__":
    main()