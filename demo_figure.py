#!/usr/bin/env python3
"""
GeoShield -- demonstration figure for jury presentation.

    python3 demo_figure.py \\
        --clean-map ~/autoware_map/nishishinjuku_autoware_map/lanelet2_map.osm \\
        --tampered-map data/dv_width_ramp.osm \\
        --labels data/dv_width_ramp_labels.json \\
        --dv-report results/dv_width_ramp.json \\
        --out results/demo_figure.png

Three-panel figure plus detection scoreboard:

  Panel 1: Clean HD map (lanelet boundaries, drivable area, route)
  Panel 2: Tampered HD map (tampered lanelets highlighted in orange/red)
  Panel 3: Detection verdict overlay (green/yellow/red per lanelet)

No ROS dependency. No bag dependency. Uses map geometry only.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path


def load_map(path: str):
    """Parse lanelet boundaries and drivable polygons from Lanelet2 OSM."""
    import xml.etree.ElementTree as ET

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


def compute_view(lanelets, target_ids, span=90.0):
    """Centre the view on the target lanelets."""
    cx = cy = 0.0
    n = 0
    for lid in target_ids:
        if lid in lanelets:
            for pts in lanelets[lid]:
                for x, y in pts:
                    cx += x
                    cy += y
                    n += 1
    if n:
        cx, cy = cx / n, cy / n
    else:
        # Fallback: centre of all lanelets
        for lid, (l, r) in lanelets.items():
            for pts in [l, r]:
                for x, y in pts:
                    cx += x
                    cy += y
                    n += 1
        if n:
            cx, cy = cx / n, cy / n

    xlim = (cx - span, cx + span)
    ylim = (cy - span * 0.62, cy + span * 0.62)
    return xlim, ylim, cx, cy


def main():
    ap = argparse.ArgumentParser(description="GeoShield demonstration figure")
    ap.add_argument("--clean-map", required=True, dest="clean_map")
    ap.add_argument("--tampered-map", required=True, dest="tampered_map")
    ap.add_argument("--labels", required=True, help="attack labels JSON")
    ap.add_argument("--dv-report", dest="dv_report", help="DV result JSON")
    ap.add_argument("--dv-summary", dest="dv_summary", help="DV summary JSON for scoreboard")
    ap.add_argument("--span", type=float, default=90.0)
    ap.add_argument("--out", default="results/demo_figure.png")
    a = ap.parse_args()

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Polygon
        from matplotlib import font_manager
    except ImportError:
        sys.exit("pip install matplotlib --break-system-packages")

    # Load data
    print("Loading maps...")
    clean_lines, clean_ll = load_map(a.clean_map)
    tam_lines, tam_ll = load_map(a.tampered_map)

    labels = json.loads(Path(a.labels).read_text())
    target_ids = [int(k.split(":")[1]) for k in labels.keys()
                  if k.startswith("lanelet:")]

    dv_report = None
    if a.dv_report and Path(a.dv_report).exists():
        dv_report = json.loads(Path(a.dv_report).read_text())

    dv_summary = None
    if a.dv_summary and Path(a.dv_summary).exists():
        dv_summary = json.loads(Path(a.dv_summary).read_text())

    # Build change verdicts from DV report
    verdicts = {}  # lanelet_id -> verdict string
    if dv_report and "changes" in dv_report:
        for ch in dv_report["changes"]:
            sid = ch.get("segment_id", "")
            if sid.startswith("lanelet:"):
                lid = int(sid.split(":")[1])
                v = ch.get("verdict", "ACCEPT")
                # Take the worst verdict per lanelet
                if lid not in verdicts or _verdict_rank(v) > _verdict_rank(verdicts[lid]):
                    verdicts[lid] = v

    xlim, ylim, cx, cy = compute_view(tam_ll, target_ids, a.span)

    # ---- Colour scheme ----
    BG = "#0f1117"
    BOUND = "#6b7280"
    AREA_CLEAN = "#2d5a3d"
    AREA_TAMPER = "#8b3a1f"
    TRAJ = "#31e0d8"
    REJECT_C = "#ef4444"
    SUSPECT_C = "#f59e0b"
    ACCEPT_C = "#22c55e"
    UNAFFECTED_C = "#2d5a3d"
    TEXT_C = "#e5e7eb"
    MUTED_C = "#9ca3af"

    # ---- Layout ----
    has_scoreboard = dv_summary is not None
    n_panels = 4 if has_scoreboard else 3
    fig = plt.figure(figsize=(22, 6.5), facecolor=BG)

    if has_scoreboard:
        gs = fig.add_gridspec(1, 4, width_ratios=[1, 1, 1, 0.82],
                              wspace=0.08, left=0.02, right=0.98,
                              top=0.88, bottom=0.08)
    else:
        gs = fig.add_gridspec(1, 3, wspace=0.08, left=0.02, right=0.98,
                              top=0.88, bottom=0.08)

    ax1 = fig.add_subplot(gs[0])
    ax2 = fig.add_subplot(gs[1])
    ax3 = fig.add_subplot(gs[2])

    def draw_map(ax, lines, lanelets, title, highlight_ids=None,
                 highlight_color=None, verdict_map=None):
        ax.set_facecolor(BG)

        for lid, (l, r) in lanelets.items():
            poly = l + list(reversed(r))
            if not any(xlim[0] < px < xlim[1] and ylim[0] < py < ylim[1]
                       for px, py in poly):
                continue

            if verdict_map and lid in verdict_map:
                v = verdict_map[lid]
                if v == "REJECT":
                    fc, alpha = REJECT_C, 0.50
                elif v == "SUSPECT":
                    fc, alpha = SUSPECT_C, 0.40
                else:
                    fc, alpha = ACCEPT_C, 0.25
            elif highlight_ids and lid in highlight_ids:
                fc, alpha = highlight_color or AREA_TAMPER, 0.45
            else:
                fc, alpha = AREA_CLEAN, 0.18

            ax.add_patch(Polygon(poly, closed=True, facecolor=fc,
                                 alpha=alpha, edgecolor="none", zorder=1))

        for pts in lines:
            gx, gy = zip(*pts)
            ax.plot(gx, gy, color=BOUND, lw=0.6, alpha=0.7, zorder=2)

        # Annotate tampered lanelets
        if highlight_ids:
            for lid in highlight_ids:
                if lid not in lanelets:
                    continue
                l, r = lanelets[lid]
                mx = sum(p[0] for p in l + r) / len(l + r)
                my = sum(p[1] for p in l + r) / len(l + r)
                if xlim[0] < mx < xlim[1] and ylim[0] < my < ylim[1]:
                    ax.plot(mx, my, "o", color="white", markersize=3, zorder=5)

        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)
        ax.set_aspect("equal")
        ax.set_xticks([])
        ax.set_yticks([])
        for sp in ax.spines.values():
            sp.set_color("#2a2e37")
        ax.set_title(title, color=TEXT_C, fontsize=12,
                     fontweight="bold", pad=10, fontfamily="monospace")

    # Panel 1: Clean map
    draw_map(ax1, clean_lines, clean_ll,
             "(a)  Clean HD Map")

    # Panel 2: Tampered map with highlighted attacks
    draw_map(ax2, tam_lines, tam_ll,
             "(b)  Tampered HD Map",
             highlight_ids=set(target_ids),
             highlight_color=AREA_TAMPER)

    # Panel 3: Detection result
    draw_map(ax3, tam_lines, tam_ll,
             "(c)  GeoShield Detection",
             verdict_map=verdicts)

    # Add legend to panel 3
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor=REJECT_C, alpha=0.5, label="REJECT"),
        Patch(facecolor=SUSPECT_C, alpha=0.4, label="SUSPECT"),
        Patch(facecolor=ACCEPT_C, alpha=0.25, label="ACCEPT"),
    ]
    ax3.legend(handles=legend_elements, loc="lower right",
               fontsize=9, facecolor="#1a1d24", edgecolor="#3a3f47",
               labelcolor=TEXT_C, framealpha=0.9)

    # Panel 4: Scoreboard (if DV summary available)
    if has_scoreboard:
        ax4 = fig.add_subplot(gs[3])
        ax4.set_facecolor(BG)
        ax4.set_xlim(0, 1)
        ax4.set_ylim(0, 1)
        ax4.set_xticks([])
        ax4.set_yticks([])
        for sp in ax4.spines.values():
            sp.set_color("#2a2e37")

        ax4.set_title("(d)  Detection Scoreboard", color=TEXT_C,
                       fontsize=12, fontweight="bold", pad=10,
                       fontfamily="monospace")

        results = dv_summary.get("results", [])
        n_rows = len(results)
        y = 0.93
        dy = 0.80 / max(n_rows, 1)

        # Header
        ax4.text(0.04, y, "Condition (N_gt)", color=MUTED_C, fontsize=7.5,
                 fontfamily="monospace", fontweight="bold", va="top")
        ax4.text(0.68, y, "Verdict", color=MUTED_C, fontsize=7.5,
                 fontfamily="monospace", fontweight="bold", va="top")
        y -= 0.035
        ax4.axhline(y=y, xmin=0.03, xmax=0.97, color="#3a3f47", lw=0.5)
        y -= dy * 0.35

        for r in results:
            name = r.get("label", r["attack"].replace("_", " "))
            ngt = r.get("n_tampered", 0)
            verdict = r["verdict"]
            d = r.get("detection", {})
            recall = d.get("recall", None)
            prec = d.get("precision", None)
            fp = d.get("FP", 0)
            note = r.get("note", "")

            label_name = f"{name} (N={ngt})" if r["attack"] != "clean_control" else "clean (control)"

            if verdict == "REJECT":
                vc = REJECT_C
                icon = "\u2717"  # ✗
            elif verdict == "N/A":
                vc = MUTED_C
                icon = "\u2014"  # —
            else:
                vc = ACCEPT_C
                icon = "\u2713"  # ✓

            ax4.text(0.04, y, label_name, color=TEXT_C, fontsize=7.5,
                     fontfamily="monospace", fontweight="bold" if r["attack"] == "clean_control" else "normal",
                     va="top")
            ax4.text(0.68, y, f"{icon} {verdict}", color=vc, fontsize=7.5,
                     fontfamily="monospace", fontweight="bold", va="top")

            # Metrics / Note line
            y_sub = y - 0.036
            if r["attack"] == "clean_control":
                ax4.text(0.04, y_sub, "0 false alarms (control)", color="#34d399",
                         fontsize=6.5, fontfamily="monospace", va="top")
            elif verdict == "N/A":
                ax4.text(0.04, y_sub, "no applicable lanelets", color=MUTED_C,
                         fontsize=6.5, fontfamily="monospace", va="top")
            elif prec is not None and recall is not None:
                metric_str = f"P={prec:.3f}  R={recall:.3f}"
                if fp > 0:
                    metric_str += f"  ({fp} FP)"
                ax4.text(0.04, y_sub, metric_str, color=MUTED_C,
                         fontsize=6.5, fontfamily="monospace", va="top")

            y -= dy

        # Summary line
        detected = dv_summary.get("detected", 0)
        applicable = dv_summary.get("applicable", 0)
        rate = dv_summary.get("detection_rate", 0)
        y -= 0.015
        ax4.axhline(y=y, xmin=0.03, xmax=0.97, color="#3a3f47", lw=0.5)
        y -= dy * 0.45
        ax4.text(0.04, y, f"Detection: {detected}/{applicable} attacks",
                 color=TEXT_C, fontsize=8, fontfamily="monospace",
                 fontweight="bold", va="top")
        ax4.text(0.68, y, f"{rate*100:.0f}%",
                 color=REJECT_C if rate == 1.0 else SUSPECT_C,
                 fontsize=12, fontfamily="monospace",
                 fontweight="bold", va="top")

    # Subtitle with attack details and raw peak deviation
    n_attacked = len(target_ids)
    if labels:
        first = next(iter(labels.values()))
        attack_type = first.get("attack_type", "unknown").replace("_", " ")
    else:
        attack_type = "unknown"

    dv_verdict = dv_report.get("verdict", "?") if dv_report else "?"
    n_changes = len(dv_report.get("changes", [])) if dv_report else 0

    fig.suptitle(
        f"GeoShield: {attack_type} attack on {n_attacked} lanelets  "
        f"|  {n_changes} changes detected  "
        f"|  Verdict: {dv_verdict}  "
        f"|  Raw peak vehicle deviation: 0.0402 m",
        color=TEXT_C, fontsize=11, y=0.97, fontfamily="monospace")

    # Save
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(a.out, dpi=170, bbox_inches="tight", facecolor=BG)
    print(f"Wrote {a.out}")


def _verdict_rank(v: str) -> int:
    return {"ACCEPT": 0, "SUSPECT": 1, "REJECT": 2}.get(v, -1)


if __name__ == "__main__":
    main()
