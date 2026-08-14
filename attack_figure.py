#!/usr/bin/env python3
"""
GeoShield -- Sato-style paired figure: clean vs tampered, zoomed on the attack.

    python3 attack_figure.py \
        --clean-map ~/autoware_map/nishishinjuku_autoware_map/lanelet2_map.osm \
        --tampered-map data/route_g3.0.osm \
        --clean-bag data/bags/clean_run_good \
        --attack-bag data/bags/route_g3.0 \
        --out results/attack_figure.png

Styled after Figure 6 of Sato et al. (VehicleSec 2025): dark background,
lanelet boundaries in grey, drivable area shaded, trajectory in cyan.

WHAT THIS FIGURE SHOWS, AND WHAT IT DOES NOT
--------------------------------------------
It shows the MAP being tampered. On the attacked run the drivable corridor
visibly widens -- lanelet 3002013 goes from 3.057 m to 6.358 m and its
centreline displaces by 2.79 m.

It does NOT show a large trajectory deviation, because there wasn't one:
the measured driven deviation is 0.051 m. Autoware's motion planner optimises
a smooth path inside the drivable area rather than tracking the geometric
centre, so widening a straight lane PERMITS lateral movement without CAUSING
it. Sato et al. reached the same conclusion from the other direction -- they
needed to add a centerline before the vehicle would reliably follow the
altered path.

The honest claim this figure supports: substantial map tampering does not
imply substantial behavioural impact. Attack magnitude in the map is not
attack magnitude in behaviour.
"""

from __future__ import annotations

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def load_map(path: str):
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

    lines, lanelets = [], {}
    for rel in root.findall("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        if tags.get("type") != "lanelet":
            continue
        left = right = None
        for mem in rel.findall("member"):
            pts = [nodes[n] for n in ways.get(int(mem.get("ref")), []) if n in nodes]
            if len(pts) < 2:
                continue
            if mem.get("role") == "left":
                left = pts
            elif mem.get("role") == "right":
                right = pts
            if mem.get("role") in ("left", "right"):
                lines.append(pts)
        if left and right:
            lanelets[int(rel.get("id"))] = (left, right)
    return lines, lanelets


def read_bag_path(bag: str):
    try:
        from mcap.reader import make_reader
        from mcap_ros2.decoder import DecoderFactory
    except ImportError:
        sys.exit("pip install mcap mcap-ros2-support --break-system-packages")
    files = list(Path(bag).glob("*.mcap"))
    if not files:
        sys.exit(f"no .mcap in {bag}")
    pts = []
    with open(files[0], "rb") as fh:
        for _, _, _, msg in make_reader(
                fh, decoder_factories=[DecoderFactory()]).iter_decoded_messages(
                topics=["/localization/kinematic_state"]):
            p = msg.pose.pose.position
            if not pts or math.dist(pts[-1], (p.x, p.y)) > 0.05:
                pts.append((p.x, p.y))
    return pts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--clean-map", required=True, dest="clean_map")
    ap.add_argument("--tampered-map", required=True, dest="tampered_map")
    ap.add_argument("--clean-bag", required=True, dest="clean_bag")
    ap.add_argument("--attack-bag", required=True, dest="attack_bag")
    ap.add_argument("--attacked", default="3002007,3002013,3013032,3002017",
                    help="lanelet ids to centre the view on")
    ap.add_argument("--span", type=float, default=90.0,
                    help="half-width of the view in metres")
    ap.add_argument("--out", default="attack_figure.png")
    a = ap.parse_args()

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Polygon
    except ImportError:
        sys.exit("pip install matplotlib --break-system-packages")

    ids = [int(x) for x in a.attacked.split(",") if x.strip()]

    print("loading ...")
    clean_lines, clean_ll = load_map(a.clean_map)
    tam_lines, tam_ll = load_map(a.tampered_map)
    cpath = read_bag_path(a.clean_bag)
    apath = read_bag_path(a.attack_bag)

    # centre the view on the attacked lanelets
    cx = cy = 0.0
    n = 0
    for lid in ids:
        if lid in tam_ll:
            for pts in tam_ll[lid]:
                for x, y in pts:
                    cx += x
                    cy += y
                    n += 1
    if n:
        cx, cy = cx / n, cy / n
    else:
        cx, cy = apath[len(apath) // 2]

    span = a.span
    xlim = (cx - span, cx + span)
    ylim = (cy - span * 0.62, cy + span * 0.62)

    BG = "#1c1f24"
    BOUND = "#8d949c"
    AREA = "#5f7a68"
    TRAJ = "#31e0d8"
    HOT = "#ff8a3d"

    fig, axes = plt.subplots(1, 2, figsize=(17, 6.4), facecolor=BG)

    def panel(ax, lines, lanelets, path, title, mark):
        ax.set_facecolor(BG)
        # drivable area of every lanelet in view
        for lid, (l, r) in lanelets.items():
            poly = l + list(reversed(r))
            if any(xlim[0] < px < xlim[1] and ylim[0] < py < ylim[1]
                   for px, py in poly):
                hot = mark and lid in ids
                ax.add_patch(Polygon(poly, closed=True,
                                     facecolor=HOT if hot else AREA,
                                     alpha=0.42 if hot else 0.20,
                                     edgecolor="none", zorder=1))
        for pts in lines:
            gx, gy = zip(*pts)
            ax.plot(gx, gy, color=BOUND, lw=0.7, alpha=0.8, zorder=2)
        px, py = zip(*path)
        ax.plot(px, py, color=TRAJ, lw=2.4, zorder=4)

        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)
        ax.set_aspect("equal")
        ax.set_xticks([])
        ax.set_yticks([])
        for sp in ax.spines.values():
            sp.set_color("#3a3f47")
        ax.set_title(title, color="white", fontsize=13,
                     fontweight="bold", pad=10)

    panel(axes[0], clean_lines, clean_ll, cpath,
          "(a) clean HD map", False)
    panel(axes[1], tam_lines, tam_ll, apath,
          "(b) tampered HD map — widened lanelets in orange", True)

    # label the trajectory, Sato-style
    for ax, path in zip(axes, (cpath, apath)):
        inview = [p for p in path if xlim[0] < p[0] < xlim[1]
                  and ylim[0] < p[1] < ylim[1]]
        if inview:
            tx, ty = inview[len(inview) // 2]
            ax.annotate("trajectory", xy=(tx, ty),
                        xytext=(tx - span * 0.45, ty + span * 0.32),
                        color="white", fontsize=11,
                        bbox=dict(boxstyle="round,pad=0.35", fc="white",
                                  ec="none", alpha=0.92),
                        arrowprops=dict(arrowstyle="-", color=TRAJ, lw=1.6))
            axes_txt = ax

    fig.suptitle(
        "HD map tampering: 8 lanelets widened, cumulative +3.0 m   |   "
        "lane 3002013: 3.06 m → 6.36 m, centreline displaced 2.79 m   |   "
        "driven deviation 0.051 m",
        color="white", fontsize=12, y=0.035)

    fig.savefig(a.out, dpi=170, bbox_inches="tight", facecolor=BG)
    print(f"wrote {a.out}")
    print(f"  view centred on {cx:.0f}, {cy:.0f}  span {span} m")


if __name__ == "__main__":
    main()