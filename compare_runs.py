#!/usr/bin/env python3
"""
GeoShield -- side-by-side demo: clean map vs tampered map, each with the
trajectory the vehicle actually drove on it.

    python3 compare_runs.py \
        --clean-map ~/autoware_map/nishishinjuku_autoware_map/lanelet2_map.osm \
        --tampered-map data/route_g3.0.osm \
        --clean-bag data/bags/clean_run_good \
        --attack-bag data/bags/route_g3.0 \
        --out results/compare.png

Laptop only. No ROS, no Foxglove, no simulator.


ENDPOINT ARTEFACT -- WHY THIS SCRIPT TRUNCATES
----------------------------------------------
The first version of this analysis reported a driven deviation of 1.414 m.
That number was wrong, and the way it was wrong is worth stating plainly:

    offsets over the last 10 samples : 0.474 0.579 0.683 ... 1.309 1.414
    max offset excluding last 20     : 0.051
    gap between the two END points   : 1.414

The two runs each recorded for a fixed 90 s. The vehicle on the tampered map
happened to be 1.4 m further along the route when recording stopped. With no
clean-path sample beyond that point, every trailing attacked sample matched
against the clean path's final point, and the offset grew monotonically to
exactly the endpoint gap. Frechet distance then reported that gap as the
result. It measured where the recording stopped, not where the vehicle went.

Fix: truncate both paths to their common arc length before comparing. The
corrected figure is what this script produces.
"""

from __future__ import annotations

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


# ----------------------------------------------------------------------
# map
# ----------------------------------------------------------------------

def load_map(path: str):
    """Boundary polylines and lanelet id -> boundary mapping, metric frame."""
    root = ET.parse(path).getroot()

    nodes = {}
    for n in root.findall("node"):
        tags = {t.get("k"): t.get("v") for t in n.findall("tag")}
        try:
            nodes[int(n.get("id"))] = (float(tags["local_x"]), float(tags["local_y"]))
        except (KeyError, TypeError, ValueError):
            pass

    ways = {int(w.get("id")): [int(nd.get("ref")) for nd in w.findall("nd")]
            for w in root.findall("way")}

    all_lines, by_lanelet = [], {}
    for rel in root.findall("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        if tags.get("type") != "lanelet":
            continue
        lid = int(rel.get("id"))
        lines = []
        for mem in rel.findall("member"):
            if mem.get("role") not in ("left", "right"):
                continue
            pts = [nodes[n] for n in ways.get(int(mem.get("ref")), []) if n in nodes]
            if len(pts) >= 2:
                lines.append(pts)
                all_lines.append(pts)
        if lines:
            by_lanelet[lid] = lines
    return all_lines, by_lanelet


# ----------------------------------------------------------------------
# trajectories
# ----------------------------------------------------------------------

def read_bag_path(bag: str) -> list[tuple[float, float]]:
    try:
        from mcap.reader import make_reader
        from mcap_ros2.decoder import DecoderFactory
    except ImportError:
        sys.exit("pip install mcap mcap-ros2-support --break-system-packages")

    files = list(Path(bag).glob("*.mcap"))
    if not files:
        sys.exit(f"no .mcap inside {bag}")

    pts = []
    with open(files[0], "rb") as fh:
        reader = make_reader(fh, decoder_factories=[DecoderFactory()])
        for _, _, _, msg in reader.iter_decoded_messages(
                topics=["/localization/kinematic_state"]):
            p = msg.pose.pose.position
            if not pts or math.dist(pts[-1], (p.x, p.y)) > 0.05:
                pts.append((p.x, p.y))
    return pts


def arc_lengths(pts) -> list[float]:
    out = [0.0]
    for i in range(1, len(pts)):
        out.append(out[-1] + math.dist(pts[i - 1], pts[i]))
    return out


def truncate_common(a, b):
    """
    Cut both paths at the shorter one's arc length.

    Without this the trailing samples of the longer path have nothing to match
    against and the reported deviation collapses to the endpoint gap.
    """
    la, lb = arc_lengths(a), arc_lengths(b)
    limit = min(la[-1], lb[-1])
    ta = [p for p, d in zip(a, la) if d <= limit]
    tb = [p for p, d in zip(b, lb) if d <= limit]
    return ta, tb, limit


def lateral_offset(a, b):
    return [min(math.dist(p, q) for q in b) for p in a]


def frechet(P, Q) -> float:
    n, m = len(P), len(Q)
    if n == 0 or m == 0:
        return float("nan")
    prev = [0.0] * m
    cur = [0.0] * m
    for i in range(n):
        for j in range(m):
            d = math.dist(P[i], Q[j])
            if i == 0 and j == 0:
                cur[j] = d
            elif i == 0:
                cur[j] = max(cur[j - 1], d)
            elif j == 0:
                cur[j] = max(prev[j], d)
            else:
                cur[j] = max(min(prev[j], prev[j - 1], cur[j - 1]), d)
        prev, cur = cur, prev
    return prev[m - 1]


def resample(pts, n):
    if len(pts) < 2:
        return pts * n
    cum = arc_lengths(pts)
    total = cum[-1]
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


# ----------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="clean vs tampered, side by side")
    ap.add_argument("--clean-map", required=True, dest="clean_map")
    ap.add_argument("--tampered-map", required=True, dest="tampered_map")
    ap.add_argument("--clean-bag", required=True, dest="clean_bag")
    ap.add_argument("--attack-bag", required=True, dest="attack_bag")
    ap.add_argument("--attacked-lanelets", dest="attacked",
                    default="3012234,3013054,3013093,3012977,3002017,"
                            "3013032,3002007,3002013")
    ap.add_argument("--threshold", type=float, default=0.5)
    ap.add_argument("--out", default="compare.png")
    a = ap.parse_args()

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        sys.exit("pip install matplotlib --break-system-packages")

    print("loading maps ...")
    clean_lines, clean_by = load_map(a.clean_map)
    tam_lines, tam_by = load_map(a.tampered_map)
    ids = [int(x) for x in a.attacked.split(",") if x.strip()]
    print(f"  clean {len(clean_lines)} boundaries, tampered {len(tam_lines)}")

    print("loading trajectories ...")
    cpath = read_bag_path(a.clean_bag)
    apath = read_bag_path(a.attack_bag)
    print(f"  clean {len(cpath)} pts ({arc_lengths(cpath)[-1]:.1f} m), "
          f"attacked {len(apath)} pts ({arc_lengths(apath)[-1]:.1f} m)")

    raw_gap = math.dist(cpath[-1], apath[-1])
    cpath, apath, common = truncate_common(cpath, apath)
    print(f"  truncated to common {common:.1f} m "
          f"(endpoint gap before truncation was {raw_gap:.3f} m)")

    offs = lateral_offset(apath, cpath)
    peak = max(offs)
    peak_i = offs.index(peak)
    dfe = frechet(resample(cpath, 300), resample(apath, 300))

    print(f"\n  peak lateral deviation : {peak:.3f} m")
    print(f"  Frechet distance d_Fe  : {dfe:.3f} m")
    print(f"  safety threshold       : {a.threshold} m")

    # ---------------- figure ----------------
    fig = plt.figure(figsize=(16, 9))
    gs = fig.add_gridspec(2, 2, height_ratios=[2, 1], hspace=0.3, wspace=0.15)

    xs = [p[0] for p in cpath] + [p[0] for p in apath]
    ys = [p[1] for p in cpath] + [p[1] for p in apath]
    pad = 40
    xlim = (min(xs) - pad, max(xs) + pad)
    ylim = (min(ys) - pad, max(ys) + pad)

    def draw(ax, lines, by, path, colour, title, highlight):
        for pts in lines:
            gx, gy = zip(*pts)
            ax.plot(gx, gy, color="#ccd1d6", lw=0.6, zorder=1)
        if highlight:
            for lid in ids:
                for pts in by.get(lid, []):
                    hx, hy = zip(*pts)
                    ax.plot(hx, hy, color="#e08a3c", lw=2.0, zorder=2)
        px, py = zip(*path)
        ax.plot(px, py, color=colour, lw=2.6, zorder=4)
        ax.plot(px[0], py[0], "o", color=colour, ms=10, zorder=5)
        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)
        ax.set_aspect("equal")
        ax.set_title(title, fontsize=13, fontweight="bold", loc="left")
        ax.set_xlabel("x [m]")

    axc = fig.add_subplot(gs[0, 0])
    draw(axc, clean_lines, clean_by, cpath, "#2f4858",
         "(a) CLEAN map — vehicle follows the lane", False)
    axc.set_ylabel("y [m]")

    axt = fig.add_subplot(gs[0, 1])
    draw(axt, tam_lines, tam_by, apath, "#b3223b",
         "(b) TAMPERED map — 8 lanelets widened (orange)", True)

    # deviation
    axd = fig.add_subplot(gs[1, :])
    dist = arc_lengths(apath)
    axd.plot(dist, offs, color="#b3223b", lw=2, label="lateral offset")
    axd.axhline(a.threshold, color="#2f4858", ls="--", lw=1.5,
                label=f"safety threshold {a.threshold} m")
    axd.plot(dist[peak_i], peak, "o", color="#b3223b", ms=8)
    axd.annotate(f"peak {peak:.3f} m", (dist[peak_i], peak),
                 textcoords="offset points", xytext=(10, 6), fontsize=11)
    axd.set_xlabel("distance along route [m]")
    axd.set_ylabel("lateral offset [m]")
    axd.set_title("(c) deviation of the tampered run from the clean run",
                  fontsize=12, fontweight="bold", loc="left")
    axd.set_ylim(0, max(a.threshold * 1.3, peak * 1.3))
    axd.legend(loc="upper left")
    axd.grid(alpha=0.3)

    verdict = ("EXCEEDS the safety threshold" if peak > a.threshold
               else "stays within the safety threshold")
    fig.suptitle(
        f"HD map tampering, 3.0 m cumulative width ramp over 8 lanelets  |  "
        f"peak deviation {peak:.3f} m  |  {verdict}",
        fontsize=13, y=0.97)

    fig.savefig(a.out, dpi=150, bbox_inches="tight", facecolor="white")
    print(f"\nwrote {a.out}")


if __name__ == "__main__":
    main()