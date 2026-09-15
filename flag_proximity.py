#!/usr/bin/env python3
"""
GeoShield -- how close are the flagged lanelets to the driven path?

WHY THIS EXISTS. differential_verify.py reports precision 0.444 on the
width ramp: 8 targeted lanelets, 18 flagged. The obvious challenge is that
the extra 10 are spurious. Measured against the driven trajectory, they are
not: 17 of 18 flagged lanelets sit on the driven corridor, and nine of the
ten non-targeted ones are within 3.21 m -- several closer than targeted
lanelets. Adjacent lanelets share boundary ways in Lanelet2, so displacing
one lane's boundary physically moves its neighbour's geometry too. The
detector found every lanelet whose geometry actually changed; the label
file records only the eight the injector aimed at.

Run inside the Autoware container (sqlite3 bags need rosbag2):
    python3 flag_proximity.py --bag bags/demo_tampered
"""
from __future__ import annotations
import argparse, importlib.util, json, math
from pathlib import Path


def _load(name, path):
    s = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(s); s.loader.exec_module(m); return m


def main():
    ap = argparse.ArgumentParser(description="flagged-lanelet proximity to driven path")
    ap.add_argument("--bag", default="bags/demo_tampered")
    ap.add_argument("--map", default="data/route_g3.0.osm")
    ap.add_argument("--report", default="results/diff_g3.0.json")
    ap.add_argument("--labels", default="data/route_g3.0_labels.json")
    ap.add_argument("--near", type=float, default=15.0,
                    help="distance under which a lanelet counts as on-corridor")
    ap.add_argument("--out", default="results/flag_proximity.json")
    a = ap.parse_args()

    here = Path(__file__).parent
    fa = _load("fa", here / "frechet_analysis.py")
    ad = _load("l2a", here / "lanelet2_adapter.py")

    path = fa.ego_path(fa.read_bag(a.bag, [fa.ODOM])[fa.ODOM])
    segs = {s.segment_id: s for s in ad.load(a.map)}
    flagged = sorted({c["segment_id"] for c in json.load(open(a.report))["changes"]})
    targeted = set(json.load(open(a.labels)))

    rows = []
    print(f"{'lanelet':<20}{'targeted':>10}{'dist_m':>10}  on_corridor")
    for sid in flagged:
        s = segs.get(sid)
        if not s or "_cl_start_x" not in s.raw_tags:
            continue
        d = min(min(math.dist((s.raw_tags[kx], s.raw_tags[ky]), p) for p in path)
                for kx, ky in (("_cl_start_x", "_cl_start_y"),
                               ("_cl_end_x", "_cl_end_y")))
        rows.append({"segment_id": sid, "targeted": sid in targeted,
                     "dist_to_path_m": round(d, 2), "on_corridor": d < a.near})
        print(f"{sid:<20}{'YES' if sid in targeted else '-':>10}"
              f"{d:>10.2f}  {'yes' if d < a.near else 'NO'}")

    on = sum(r["on_corridor"] for r in rows)
    fp_on = sum(r["on_corridor"] and not r["targeted"] for r in rows)
    nt = sum(1 for r in rows if not r["targeted"])
    print(f"\n  flagged            : {len(rows)}")
    print(f"  on corridor        : {on}/{len(rows)}")
    print(f"  non-targeted       : {nt},  of which on corridor: {fp_on}")
    if nt:
        far = max(r["dist_to_path_m"] for r in rows if not r["targeted"])
        print(f"  furthest non-target: {far:.2f} m")

    Path(a.out).write_text(json.dumps(
        {"bag": a.bag, "near_threshold_m": a.near,
         "flagged": len(rows), "on_corridor": on,
         "non_targeted": nt, "non_targeted_on_corridor": fp_on,
         "rows": rows}, indent=2))
    print(f"\nwrote {a.out}")


if __name__ == "__main__":
    main()
