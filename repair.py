#!/usr/bin/env python3
"""
GeoShield -- Phase 5: detect, then REPAIR.

    python3 repair.py --clean-reference clean.osm --tampered data/route_g3.0.osm \
        --out data/repaired_g3.0.osm --report results/repair_g3.0.json

WHY THIS MODULE EXISTS
----------------------
Everything before this produces alarms. This produces a corrected map, which
is what the fourth simulator condition needs. Without repair there is no
"tampered + GeoShield" run, and without that row the project demonstrates an
attack rather than a defence.

Measured context from Phase 6 (Nishi-Shinjuku, 8-lanelet route):

    total gain    d_Fp      d_Fe      outcome
    off-route     0.020     0.083     noise floor
    2.0 m         0.312     0.204     under 0.5 m threshold
    3.0 m         1.998     1.414     OVER threshold
    >= 4.5 m      --        --        ego failed to localise

The claim to establish: running repair on the 3.0 m map should bring d_Fe
back under the 0.5 m threshold.

HOW REPAIR WORKS
----------------
Detection reuses the Tier-1 signal that survived Phase 4: a lanelet's width
compared with its own neighbours (width_vs_nbr_median), which reached recall
0.640 on ramps above 2 m at a 5% flag budget.

Correction is deliberately CONSERVATIVE. We do not try to recover the exact
original width -- that information is gone. We clamp a flagged lanelet toward
the median width of its unflagged neighbours, which is the safest defensible
estimate. Over-correcting a legitimately wide lanelet is itself a hazard, so
the clamp is bounded and lanelets with no clean neighbours are left alone and
reported instead.

Geometry is then rewritten the same way the injector wrote it: by displacing
the LEFT boundary along its local normal. Repair is the inverse of the edit,
so it must use the same mechanism, or the centreline ends up shifted twice.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from lanelet2_adapter import load as load_map


# ----------------------------------------------------------------------
# detection
# ----------------------------------------------------------------------

def detect(segs, clean_ref=None, chain_len: int = 5,
           trend_pct: float = 95.0, **_) -> dict:
    """
    Chain-based detection. Per-lanelet thresholding does NOT work here.

    MEASURED FAILURE OF THE PER-LANELET APPROACH (Nishi-Shinjuku, g3.0 map):
        honest neighbour delta p90 = 0.826 m
        threshold at 2.5 x p90     = 2.065 m
        ramp step (3.0 m / 8)      = 0.375 m
        result: TP 0, FP 9, recall 0.000

    Every individual step of a ramp sits far below the honest noise floor, so
    no per-lanelet threshold separates them. Worse, the thresholds that fire at
    all fire on legitimately wide intersection lanelets, and "repairing" those
    corrupted the map badly enough that the ego could no longer localise.

    The signal is in the SEQUENCE, not the segment. Tier-1 rule R1 scores a
    chain by (least-squares slope of width) x (fraction of steps moving in the
    slope direction). A merge gives one big step: high slope, low consistency.
    A ramp gives many small aligned steps: moderate slope, consistency ~1.
    R1 reached recall 1.000 on ramp attacks where per-lanelet features reached
    0.000, so repair is driven by R1 here.
    """
    from tier1_rules import _chains, _chain_trend, calibrate

    by = {s.segment_id: s for s in segs}

    # Calibrate on a clean reference map when available; otherwise on the map
    # under test, which is conservative -- an attack inflates its own threshold.
    ref = clean_ref if clean_ref is not None else segs
    ref_by = {s.segment_id: s for s in ref}
    trends = [t for t in (_chain_trend(c, ref_by)
                          for c in _chains(ref, ref_by, chain_len))
              if t is not None]
    trends.sort()
    thr = trends[int(trend_pct / 100 * len(trends))] if trends else 0.3

    flags, runs = {}, []
    for chain in _chains(segs, by, chain_len):
        t = _chain_trend(chain, by)
        if t is None or t < thr:
            continue
        ids = [x.segment_id for x in chain]
        runs.append({"lanelets": ids, "trend": round(t, 4)})
        for x in chain:
            prev = flags.get(x.segment_id, {}).get("trend", 0)
            if t > prev:
                flags[x.segment_id] = {"width": x.width_m, "trend": round(t, 4)}

    return {"flags": flags, "runs": runs,
            "trend_threshold": round(thr, 4),
            "n_ref_chains": len(trends),
            "method": "chain_trend_R1"}


def plan_repair(segs, det: dict, max_correction: float = 3.0) -> dict:
    """
    Decide a corrected width per flagged lanelet.

    Uses only UNFLAGGED neighbours as the reference. On a ramp attack the
    immediate neighbours are usually tampered too, so taking their median
    would anchor the repair to the attack itself and correct almost nothing.
    """
    by = {s.segment_id: s for s in segs}
    flagged = set(det["flags"])
    plan = {}

    for sid in flagged:
        s = by[sid]
        clean_nbrs = [by[t].width_m
                      for t in (s.predecessors + s.successors)
                      if t in by and t not in flagged and by[t].width_m is not None]

        if not clean_nbrs:
            # widen the search one hop out
            two_hop = []
            for t in (s.predecessors + s.successors):
                if t not in by:
                    continue
                for u in (by[t].predecessors + by[t].successors):
                    if u in by and u not in flagged and by[u].width_m is not None:
                        two_hop.append(by[u].width_m)
            clean_nbrs = two_hop

        if not clean_nbrs:
            # RAMP CASE. On a contiguous ramp every neighbour is tampered too,
            # so local search finds no anchor -- the synthetic test left 8 of
            # 12 lanelets unrepairable this way. Instead, walk outward along
            # the chain until an unflagged lanelet is found in each direction,
            # then linearly interpolate between those two clean endpoints.
            # A ramp is monotonic by construction, so interpolation recovers
            # the intended profile closely.
            def walk(start, forward, limit=15):
                cur, dist = start, 0
                seen = {start}
                while dist < limit:
                    nxt = [t for t in (by[cur].successors if forward
                                       else by[cur].predecessors)
                           if t in by and t not in seen]
                    if not nxt:
                        return None, dist
                    cur = nxt[0]
                    seen.add(cur)
                    dist += 1
                    if cur not in flagged and by[cur].width_m is not None:
                        return by[cur].width_m, dist
                return None, dist

            w_fwd, d_fwd = walk(sid, True)
            w_bwd, d_bwd = walk(sid, False)

            if w_fwd is not None and w_bwd is not None:
                total = d_fwd + d_bwd
                target = w_bwd + (w_fwd - w_bwd) * (d_bwd / total) if total else w_bwd
                plan[sid] = {"action": "clamp", "original": s.width_m,
                             "repaired": round(target, 3),
                             "shift": round(target - s.width_m, 3),
                             "reference_n": 2, "method": "chain_interpolation"}
            elif w_fwd is not None or w_bwd is not None:
                target = w_fwd if w_fwd is not None else w_bwd
                plan[sid] = {"action": "clamp", "original": s.width_m,
                             "repaired": round(target, 3),
                             "shift": round(target - s.width_m, 3),
                             "reference_n": 1, "method": "chain_nearest"}
            else:
                plan[sid] = {"action": "no_reference", "original": s.width_m}
            continue

        target = statistics.median(clean_nbrs)
        delta = target - s.width_m
        # bound the correction: over-correcting a legitimately wide lanelet
        # is itself a hazard
        if abs(delta) > max_correction:
            delta = math.copysign(max_correction, delta)
            target = s.width_m + delta

        plan[sid] = {
            "action": "clamp",
            "original": s.width_m,
            "repaired": round(target, 3),
            "shift": round(delta, 3),
            "reference_n": len(clean_nbrs),
        }
    return plan


# ----------------------------------------------------------------------
# geometry rewrite
# ----------------------------------------------------------------------

def apply_repair(in_path: str, out_path: str, plan: dict,
                 iterations: int = 6) -> dict:
    """
    Displace the LEFT boundary to restore the intended width, ITERATIVELY.

    WHY NOT A SINGLE PASS. Adjacent lanelets SHARE boundary nodes, so moving
    one lanelet's boundary also moves its neighbours'. On a contiguous ramp
    the corrections compound: a single pass overshot by a mean factor of 1.88
    (intended -1.500 m, actual -2.977 m), leaving lanelets at 0.8-1.5 m
    instead of the 3.0 m target.

    A fixed 1/1.88 fudge factor would be wrong too -- the coupling depends on
    how many flagged lanelets share each node, which varies along the chain.
    Instead we apply a damped correction, re-measure the resulting widths, and
    repeat. This converges without needing to model the sharing explicitly.
    """
    import copy as _copy

    tree = ET.parse(in_path)
    root = tree.getroot()

    nodes = {int(n.get("id")): n for n in root.findall("node")}
    ways = {int(w.get("id")): [int(nd.get("ref")) for nd in w.findall("nd")]
            for w in root.findall("way")}

    left_of, right_of = {}, {}
    for rel in root.findall("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        if tags.get("type") != "lanelet":
            continue
        rid = int(rel.get("id"))
        for mem in rel.findall("member"):
            if mem.get("role") == "left":
                left_of[rid] = int(mem.get("ref"))
            elif mem.get("role") == "right":
                right_of[rid] = int(mem.get("ref"))

    def xy(nid):
        n = nodes.get(nid)
        if n is None:
            return None
        d = {t.get("k"): t.get("v") for t in n.findall("tag")}
        try:
            return float(d["local_x"]), float(d["local_y"])
        except (KeyError, TypeError, ValueError):
            return None

    def set_xy(nid, x, y):
        for t in nodes[nid].findall("tag"):
            if t.get("k") == "local_x":
                t.set("v", f"{x:.4f}")
            elif t.get("k") == "local_y":
                t.set("v", f"{y:.4f}")

    def measure(lid):
        """Current mean width of a lanelet, from live node positions."""
        lw, rw = left_of.get(lid), right_of.get(lid)
        if lw not in ways or rw not in ways:
            return None
        lp = [xy(n) for n in ways[lw]]
        rp = [xy(n) for n in ways[rw]]
        lp = [q for q in lp if q]
        rp = [q for q in rp if q]
        if len(lp) < 2 or len(rp) < 2:
            return None
        k = min(len(lp), len(rp))
        return sum(math.dist(lp[int(i*(len(lp)-1)/(k-1))],
                             rp[int(i*(len(rp)-1)/(k-1))])
                   for i in range(k)) / k

    targets = {int(sid.split(":")[1]): rec["repaired"]
               for sid, rec in plan.items() if rec.get("action") == "clamp"}

    stats = {"repaired": 0, "skipped": 0, "no_reference": 0, "iterations": 0}
    stats["no_reference"] = sum(1 for r in plan.values()
                                if r.get("action") != "clamp")

    damping = 0.6            # under-correct each pass; converges, no oscillation
    for it in range(iterations):
        max_err = 0.0
        for lid, target in targets.items():
            wid = left_of.get(lid)
            if wid is None or wid not in ways:
                continue
            cur = measure(lid)
            if cur is None:
                continue
            err = target - cur
            max_err = max(max_err, abs(err))
            if abs(err) < 0.01:
                continue

            shift = err * damping
            pts = ways[wid]
            for i, nid in enumerate(pts):
                pnt = xy(nid)
                if pnt is None:
                    continue
                q = xy(pts[min(i + 1, len(pts) - 1)]) or pnt
                r = xy(pts[max(i - 1, 0)]) or pnt
                tx, ty = q[0] - r[0], q[1] - r[1]
                mag = math.hypot(tx, ty)
                if mag < 1e-9:
                    continue
                nx, ny = -ty / mag, tx / mag
                set_xy(nid, pnt[0] + nx * shift, pnt[1] + ny * shift)
        stats["iterations"] = it + 1
        if max_err < 0.02:
            break

    stats["repaired"] = len(targets)
    stats["final_max_error_m"] = round(max_err, 4)

    tree.write(out_path, encoding="utf-8", xml_declaration=True)
    return stats


# ----------------------------------------------------------------------

def evaluate(det: dict, truth_path: str | None) -> dict:
    """Score detection against ground truth, when labels are available."""
    if not truth_path or not Path(truth_path).exists():
        return {}
    raw = json.load(open(truth_path))
    truth = set(raw.get("labels", raw).keys())
    pred = set(det["flags"])
    tp, fp, fn = len(pred & truth), len(pred - truth), len(truth - pred)
    p = tp / (tp + fp) if tp + fp else 0.0
    r = tp / (tp + fn) if tp + fn else 0.0
    return {"TP": tp, "FP": fp, "FN": fn,
            "precision": round(p, 3), "recall": round(r, 3),
            "f1": round(2 * p * r / (p + r), 3) if p + r else 0.0}


def main():
    ap = argparse.ArgumentParser(description="GeoShield detect + repair")
    ap.add_argument("--tampered", required=True, help="map to repair")
    ap.add_argument("--out", required=True, help="repaired .osm output")
    ap.add_argument("--labels", help="ground-truth labels, to score detection")
    ap.add_argument("--clean-ref", dest="clean_ref",
                    help="clean map to calibrate the trend threshold on")
    ap.add_argument("--chain-len", type=int, default=5, dest="chain_len")
    ap.add_argument("--trend-pct", type=float, default=95.0, dest="trend_pct",
                    help="flag chains above this percentile of clean trends")
    ap.add_argument("--max-correction", type=float, default=3.0,
                    dest="max_correction")
    ap.add_argument("--report", help="write a JSON report")
    a = ap.parse_args()

    print(f"loading {a.tampered}")
    segs = load_map(a.tampered)
    print(f"  {len(segs)} lanelets")

    clean_ref = None
    if a.clean_ref:
        print(f"calibrating on {a.clean_ref}")
        clean_ref = load_map(a.clean_ref)

    det = detect(segs, clean_ref, a.chain_len, a.trend_pct)
    print(f"\nmethod: {det['method']}")
    print(f"trend threshold (p{a.trend_pct:g} of {det['n_ref_chains']} "
          f"reference chains): {det['trend_threshold']}")
    print(f"suspicious chains: {len(det['runs'])}")
    print(f"flagged lanelets : {len(det['flags'])}")

    scores = evaluate(det, a.labels)
    if scores:
        print(f"\ndetection vs ground truth:")
        print(f"  TP {scores['TP']}  FP {scores['FP']}  FN {scores['FN']}")
        print(f"  precision {scores['precision']}  recall {scores['recall']}"
              f"  F1 {scores['f1']}")

    plan = plan_repair(segs, det, a.max_correction)
    clamps = [p for p in plan.values() if p.get("action") == "clamp"]
    print(f"\nrepair plan: {len(clamps)} clamps, "
          f"{len(plan) - len(clamps)} without a clean reference")
    for sid, p in list(plan.items())[:8]:
        if p.get("action") == "clamp":
            print(f"  {sid:<18} {p['original']:.3f} -> {p['repaired']:.3f} m "
                  f"(shift {p['shift']:+.3f})")

    stats = apply_repair(a.tampered, a.out, plan)
    print(f"\nwrote {a.out}")
    print(f"  geometry repaired : {stats['repaired']}")
    print(f"  no reference      : {stats['no_reference']}")
    print(f"  skipped           : {stats['skipped']}")

    if a.report:
        Path(a.report).write_text(json.dumps({
            "tampered": a.tampered, "output": a.out,
            "calibration": det["calibration_p90"],
            "n_flagged": len(det["flags"]),
            "detection": scores, "plan": plan, "apply": stats,
        }, indent=2))
        print(f"  report            : {a.report}")

    print("\nNext: load the repaired map in the simulator and re-measure.")
    print("The claim to test is that d_Fe falls from 1.414 m back under 0.5 m.")


if __name__ == "__main__":
    main()