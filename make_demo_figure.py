#!/usr/bin/env python3
"""
GeoShield -- demo figure: map + clean trajectory + attacked trajectory.

    python3 make_demo_figure.py \
        --map ~/autoware_map/nishishinjuku_autoware_map/lanelet2_map.osm \
        --tampered-map data/route_g3.0.osm \
        --clean-bag data/bags/clean_run_good \
        --attack-bag data/bags/route_g3.0 \
        --out results/demo_figure.png

Runs entirely on the laptop. No ROS, no Foxglove, no simulator.

WHY A FIGURE AND NOT A 3D VIEW
------------------------------
The measured deviation is 1.414 m over a 393 m route. At map scale that is
roughly a third of a percent of the frame -- invisible. This figure solves it
by drawing the overview and an inset zoomed on the divergence, side by side
with the lateral-offset plot. The guide sees the road, the two paths, and the
magnitude, in one image.

Trajectories are read from the .mcap bags if the ROS python packages are
available; otherwise pass --clean-csv / --attack-csv with x,y columns.
"""

from __future__ import annotations

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


# ----------------------------------------------------------------------
# map geometry
# ----------------------------------------------------------------------

def load_lanelet_boundaries(path: str) -> list[list[tuple[float, float]]]:
    """Every lanelet's left and right boundary, in the metric local frame."""
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

    out = []
    for rel in root.findall("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        if tags.get("type") != "lanelet":
            continue
        for mem in rel.findall("member"):
            if mem.get("role") not in ("left", "right"):
                continue
            wid = int(mem.get("ref"))
            pts = [nodes[n] for n in ways.get(wid, []) if n in nodes]
            if len(pts) >= 2:
                out.append(pts)
    return out


def load_route_lanelets(path: str, ids: list[int]) -> list[list[tuple[float, float]]]:
    """Boundaries of specific lanelets -- used to highlight the attacked run."""
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

    want = set(ids)
    out = []
    for rel in root.findall("relation"):
        if int(rel.get("id")) not in want:
            continue
        for mem in rel.findall("member"):
            if mem.get("role") not in ("left", "right"):
                continue
            pts = [nodes[n] for n in ways.get(int(mem.get("ref")), []) if n in nodes]
            if len(pts) >= 2:
                out.append(pts)
    return out


# ----------------------------------------------------------------------
# trajectories
# ----------------------------------------------------------------------

def read_bag_path(bag: str) -> list[tuple[float, float]]:
    """Driven path from /localization/kinematic_state in an mcap bag."""
    try:
        from mcap.reader import make_reader
    except ImportError:
        sys.exit("pip install mcap mcap-ros2-support\n"
                 "(or export the trajectories to CSV and use --clean-csv/--attack-csv)")
    try:
        from mcap_ros2.decoder import DecoderFactory
    except ImportError:
        sys.exit("pip install mcap-ros2-support")

    files = list(Path(bag).glob("*.mcap"))
    if not files:
        sys.exit(f"no .mcap inside {bag}")

    pts = []
    with open(files[0], "rb") as fh:
        reader = make_reader(fh, decoder_factories=[DecoderFactory()])
        for _, ch, _, msg in reader.iter_decoded_messages(
                topics=["/localization/kinematic_state"]):
            p = msg.pose.pose.position
            if not pts or math.dist(pts[-1], (p.x, p.y)) > 0.05:
                pts.append((p.x, p.y))
    return pts


def read_csv_path(path: str) -> list[tuple[float, float]]:
    import csv
    pts = []
    with open(path) as fh:
        for row in csv.reader(fh):
            try:
                pts.append((float(row[0]), float(row[1])))
            except (ValueError, IndexError):
                continue
    return pts


def lateral_offset(a: list, b: list) -> list[float]:
    """Distance from each point of `a` to the nearest point of `b`."""
    return [min(math.dist(p, q) for q in b) for p in a]


# ----------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="GeoShield demo figure")
    ap.add_argument("--map", required=True, help="clean Lanelet2 .osm")
    ap.add_argument("--tampered-map", dest="tampered_map",
                    help="tampered .osm, drawn as a dashed overlay")
    ap.add_argument("--clean-bag", dest="clean_bag")
    ap.add_argument("--attack-bag", dest="attack_bag")
    ap.add_argument("--clean-csv", dest="clean_csv")
    ap.add_argument("--attack-csv", dest="attack_csv")
    ap.add_argument("--attacked-lanelets", dest="attacked",
                    default="3012234,3013054,3013093,3012977,3002017,"
                            "3013032,3002007,3002013")
    ap.add_argument("--threshold", type=float, default=0.5)
    ap.add_argument("--out", default="demo_figure.png")
    a = ap.parse_args()

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
    except ImportError:
        sys.exit("pip install matplotlib")

    print("loading map ...")
    bounds = load_lanelet_boundaries(a.map)
    print(f"  {len(bounds)} boundary lines")

    attacked_ids = [int(x) for x in a.attacked.split(",") if x.strip()]
    attacked = load_route_lanelets(a.map, attacked_ids)
    print(f"  {len(attacked)} boundaries on the attacked route")

    print("loading trajectories ...")
    clean = read_csv_path(a.clean_csv) if a.clean_csv else read_bag_path(a.clean_bag)
    attack = read_csv_path(a.attack_csv) if a.attack_csv else read_bag_path(a.attack_bag)
    print(f"  clean {len(clean)} points, attacked {len(attack)} points")

    if not clean or not attack:
        sys.exit("empty trajectory -- check the bag paths")

    offs = lateral_offset(attack, clean)
    peak = max(offs)
    peak_i = offs.index(peak)

    # ---------------- figure ----------------
    fig = plt.figure(figsize=(15, 8))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.4, 1], height_ratios=[1, 1],
                          hspace=0.28, wspace=0.22)

    # (a) overview
    ax = fig.add_subplot(gs[:, 0])
    for pts in bounds:
        xs, ys = zip(*pts)
        ax.plot(xs, ys, color="#c8ccd0", lw=0.5, zorder=1)
    for pts in attacked:
        xs, ys = zip(*pts)
        ax.plot(xs, ys, color="#e08a3c", lw=1.6, zorder=2)

    cx, cy = zip(*clean)
    ax_, ay = zip(*attack)
    ax.plot(cx, cy, color="#2f4858", lw=2.2, label="clean map", zorder=4)
    ax.plot(ax_, ay, color="#b3223b", lw=2.2, label="tampered map", zorder=5)
    ax.plot(cx[0], cy[0], "o", color="#2f4858", ms=9, zorder=6)
    ax.annotate("start", (cx[0], cy[0]), textcoords="offset points",
                xytext=(10, 8), fontsize=10)

    pad = 60
    ax.set_xlim(min(min(cx), min(ax_)) - pad, max(max(cx), max(ax_)) + pad)
    ax.set_ylim(min(min(cy), min(ay)) - pad, max(max(cy), max(ay)) + pad)
    ax.set_aspect("equal")
    ax.set_title("(a) Nishi-Shinjuku route: clean vs tampered HD map",
                 fontsize=13, fontweight="bold", loc="left")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.legend(loc="upper right", framealpha=0.95)

    # marker for the zoom region
    zx, zy = attack[peak_i]
    ax.add_patch(Rectangle((zx - 25, zy - 25), 50, 50, fill=False,
                           edgecolor="#b3223b", lw=1.2, ls="--", zorder=7))

    # (b) zoom on the divergence
    az = fig.add_subplot(gs[0, 1])
    for pts in bounds:
        xs, ys = zip(*pts)
        az.plot(xs, ys, color="#c8ccd0", lw=0.7)
    az.plot(cx, cy, color="#2f4858", lw=2.6, label="clean")
    az.plot(ax_, ay, color="#b3223b", lw=2.6, label="tampered")
    az.set_xlim(zx - 25, zx + 25)
    az.set_ylim(zy - 25, zy + 25)
    az.set_aspect("equal")
    az.set_title(f"(b) maximum divergence: {peak:.2f} m",
                 fontsize=12, fontweight="bold", loc="left")
    az.set_xlabel("x [m]")
    az.legend(loc="best", fontsize=9)

    # (c) lateral offset along the route
    ao = fig.add_subplot(gs[1, 1])
    dist = [0.0]
    for i in range(1, len(attack)):
        dist.append(dist[-1] + math.dist(attack[i - 1], attack[i]))
    ao.plot(dist, offs, color="#b3223b", lw=2)
    ao.axhline(a.threshold, color="#2f4858", ls="--", lw=1.4,
               label=f"safety threshold {a.threshold} m")
    ao.fill_between(dist, a.threshold, offs,
                    where=[o > a.threshold for o in offs],
                    color="#b3223b", alpha=0.18)
    ao.set_xlabel("distance along route [m]")
    ao.set_ylabel("lateral offset [m]")
    ao.set_title("(c) deviation from the clean trajectory",
                 fontsize=12, fontweight="bold", loc="left")
    ao.legend(loc="upper left", fontsize=9)
    ao.grid(alpha=0.3)

    over = sum(1 for o in offs if o > a.threshold) / len(offs) * 100
    fig.suptitle(
        f"HD map tampering: 8 lanelets widened by 3.0 m cumulative  |  "
        f"peak deviation {peak:.2f} m  |  "
        f"{over:.0f}% of route beyond the {a.threshold} m safety threshold",
        fontsize=12, y=0.975)

    fig.savefig(a.out, dpi=160, bbox_inches="tight", facecolor="white")
    print(f"\nwrote {a.out}")
    print(f"  peak lateral deviation : {peak:.3f} m")
    print(f"  route beyond threshold : {over:.1f}%")


if __name__ == "__main__":
    main()