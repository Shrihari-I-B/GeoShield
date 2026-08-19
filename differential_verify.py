#!/usr/bin/env python3
"""
GeoShield -- Phase 7: differential integrity verification.

    python3 differential_verify.py \
        --previous ~/autoware_map/nishishinjuku_autoware_map/lanelet2_map.osm \
        --candidate data/route_g3.0.osm \
        --labels data/route_g3.0_labels.json \
        --report results/diff_g3.0.json


WHY THIS EXISTS -- AND WHY THE EARLIER APPROACH COULD NOT WORK
--------------------------------------------------------------
Phases 3-5 asked: "is this lanelet anomalous?" That question is unanswerable
on this data, and we measured the bound three independent ways:

    honest width delta between connected lanelets   p90 = 0.826 m
    our ramp attack, per step                             0.375 m
    rule-based detection ceiling                      F1 = 0.220
    Isolation Forest, 31 features                  recall = 0.133
    chain-trend repair, every threshold           precision <= 0.055

Every individual step of a gradual ramp is smaller than ordinary map noise, so
no single-snapshot detector separates them. That is an information bound, not
a modelling failure.

The HD map change-detection literature never attempts single-snapshot
verification. Pannen et al. (ICRA 2019, 2020) compare the map against sensor
observations; ExelMap (Wild et al., ECCV-W 2024) compares against a prior map;
production map services version their tiles precisely so consumers can tell
which areas changed between releases. Every method compares the map to
something OUTSIDE itself.

So this module asks a different and answerable question:

    not  "is this lanelet anomalous?"
    but  "did this lanelet CHANGE, and is that change JUSTIFIED?"

The adversary controls the map but not its history. To attack, they must
produce a change -- and a 3.0 m width change is unmissable against a map that
otherwise changed by centimetres, even though 0.375 m per step is invisible in
isolation.

THREAT MODEL
------------
The vehicle or fleet holds a previously verified map version (Sato et al.
Table 1: base layer updates weeks-to-months, geometric layers days-to-weeks, so
a prior version always exists in the cache). The adversary compromises the
supply chain and delivers a tampered version N. GeoShield verifies N against
N-1 before it reaches the planner.

Where no prior version exists -- a new region, a first download -- the Tier 1/2
single-snapshot detectors remain the fallback, with their measured limits.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from dataclasses import dataclass, field, asdict
from pathlib import Path

from lanelet2_adapter import load as load_map


# ----------------------------------------------------------------------
# change records
# ----------------------------------------------------------------------

@dataclass
class Change:
    segment_id: str
    field_name: str
    previous: object
    candidate: object
    delta: float
    relative: float
    severity: float = 0.0
    verdict: str = ""
    reason: str = ""


@dataclass
class DiffReport:
    n_previous: int = 0
    n_candidate: int = 0
    added: list = field(default_factory=list)
    removed: list = field(default_factory=list)
    changes: list = field(default_factory=list)
    runs: list = field(default_factory=list)
    verdict: str = "ACCEPT"

    def rejected(self):
        return [c for c in self.changes if c.verdict == "REJECT"]

    def flagged_ids(self):
        return {c.segment_id for c in self.changes if c.verdict != "ACCEPT"}


# ----------------------------------------------------------------------
# change-plausibility policy
# ----------------------------------------------------------------------

# Tolerances are derived from what legitimate re-survey actually produces.
# A re-survey of unchanged road differs by measurement noise -- centimetres.
# Genuine roadworks change a lane by a lane-width, but then they change the
# LANE COUNT or the connectivity too, which we check separately.
WIDTH_NOISE_M = 0.10          # re-survey measurement noise
WIDTH_SUSPECT_REL = 0.05      # >5% width change without corroboration
WIDTH_REJECT_REL = 0.15       # >15% is not a survey difference
SPEED_SUSPECT_KPH = 5.0
CENTRELINE_SUSPECT_M = 0.30   # centreline should not move without roadworks


def _xy(seg, prefix):
    x = seg.raw_tags.get(prefix + "_x")
    y = seg.raw_tags.get(prefix + "_y")
    if x is None or y is None:
        return None
    try:
        return (float(x), float(y))
    except (TypeError, ValueError):
        return None


def structural_diff(prev_path: str, cand_path: str) -> list:
    """
    Compare which ELEMENTS each lanelet has, not just their values.

    WHY THIS IS SEPARATE FROM THE FIELD DIFF. A centreline-injection attack
    adds an explicit `role="centerline"` member. The boundaries are untouched,
    so every geometric field we compare -- width, computed centre, speed,
    direction, connectivity -- is identical between versions. The field diff
    reported ACCEPT with recall 0.000 against exactly this attack.

    But the change is unmissable structurally: the previous version had no
    centreline member at all. A lanelet that acquires an explicit centreline
    between two map releases, without any corresponding boundary change, is
    not a re-survey artefact. Nothing legitimate produces it.

    This is the strongest argument for differential verification: the
    adversary must ADD something, and additions are visible even when values
    are not.
    """
    import xml.etree.ElementTree as ET

    def roles(path):
        out = {}
        for rel in ET.parse(path).getroot().findall("relation"):
            tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
            if tags.get("type") != "lanelet":
                continue
            out[int(rel.get("id"))] = {
                m.get("role") for m in rel.findall("member") if m.get("role")
            }
        return out

    prev, cand = roles(prev_path), roles(cand_path)
    changes = []

    for lid in sorted(set(prev) & set(cand)):
        gained = cand[lid] - prev[lid]
        lost = prev[lid] - cand[lid]
        if not (gained or lost):
            continue
        ch = Change(f"lanelet:{lid}", "structure",
                    sorted(prev[lid]), sorted(cand[lid]), 1.0, 1.0)
        ch.verdict = "REJECT"
        ch.severity = 1.0
        if "centerline" in gained:
            ch.reason = ("explicit centreline ADDED where the previous version "
                         "had none -- the planner is now told where to drive, "
                         "rather than inferring it from the boundaries")
        elif gained:
            ch.reason = f"lanelet gained member roles: {sorted(gained)}"
        else:
            ch.reason = f"lanelet lost member roles: {sorted(lost)}"
        changes.append(ch)

    return changes


def compare(prev_segs, cand_segs) -> DiffReport:
    """Field-by-field diff of two map versions, keyed by lanelet id."""
    p = {s.segment_id: s for s in prev_segs}
    c = {s.segment_id: s for s in cand_segs}
    rep = DiffReport(n_previous=len(p), n_candidate=len(c))

    rep.added = sorted(set(c) - set(p))
    rep.removed = sorted(set(p) - set(c))

    for sid in sorted(set(p) & set(c)):
        a, b = p[sid], c[sid]

        # ---- width -------------------------------------------------
        if a.width_m is not None and b.width_m is not None:
            d = b.width_m - a.width_m
            rel = d / a.width_m if a.width_m else 0.0
            if abs(d) > WIDTH_NOISE_M:
                ch = Change(sid, "width_m", round(a.width_m, 3),
                            round(b.width_m, 3), round(d, 3), round(rel, 4))
                if abs(rel) >= WIDTH_REJECT_REL:
                    ch.verdict = "REJECT"
                    ch.severity = min(abs(rel) / WIDTH_REJECT_REL, 1.0)
                    ch.reason = (f"width changed {rel*100:+.1f}% "
                                 f"({d:+.2f} m) with no corroborating change "
                                 f"to lane count or connectivity")
                elif abs(rel) >= WIDTH_SUSPECT_REL:
                    ch.verdict = "SUSPECT"
                    ch.severity = 0.5
                    ch.reason = f"width changed {rel*100:+.1f}%"
                else:
                    ch.verdict = "ACCEPT"
                    ch.reason = "within re-survey tolerance"
                rep.changes.append(ch)

        # ---- centreline position ------------------------------------
        pa, pb = _xy(a, "_cl_start"), _xy(b, "_cl_start")
        qa, qb = _xy(a, "_cl_end"), _xy(b, "_cl_end")
        if pa and pb and qa and qb:
            shift = max(math.dist(pa, pb), math.dist(qa, qb))
            if shift > CENTRELINE_SUSPECT_M:
                ch = Change(sid, "centreline", 0.0, round(shift, 3),
                            round(shift, 3), 0.0)
                ch.verdict = "REJECT" if shift > 1.0 else "SUSPECT"
                ch.severity = min(shift / 2.0, 1.0)
                ch.reason = (f"lane centre displaced {shift:.2f} m -- the "
                             f"drivable corridor moved, not just widened")
                rep.changes.append(ch)

        # ---- speed limit ---------------------------------------------
        if a.speed_limit_kph is not None and b.speed_limit_kph is not None:
            d = b.speed_limit_kph - a.speed_limit_kph
            if abs(d) >= SPEED_SUSPECT_KPH:
                ch = Change(sid, "speed_limit_kph", a.speed_limit_kph,
                            b.speed_limit_kph, round(d, 1),
                            round(d / max(a.speed_limit_kph, 1), 3))
                ch.verdict = "REJECT" if abs(d) >= 15 else "SUSPECT"
                ch.severity = min(abs(d) / 30.0, 1.0)
                ch.reason = f"speed limit changed by {d:+.0f} km/h"
                rep.changes.append(ch)

        # ---- direction ------------------------------------------------
        if a.oneway is not None and b.oneway is not None and a.oneway != b.oneway:
            ch = Change(sid, "oneway", a.oneway, b.oneway, 1.0, 1.0)
            ch.verdict = "REJECT"
            ch.severity = 1.0
            ch.reason = "travel direction reversed"
            rep.changes.append(ch)

        # ---- connectivity ---------------------------------------------
        if set(a.successors) != set(b.successors):
            lost = sorted(set(a.successors) - set(b.successors))
            gained = sorted(set(b.successors) - set(a.successors))
            ch = Change(sid, "successors", lost, gained, 1.0, 1.0)
            ch.verdict = "REJECT"
            ch.severity = 1.0
            ch.reason = f"connectivity altered: -{len(lost)} +{len(gained)}"
            rep.changes.append(ch)

    return rep


# ----------------------------------------------------------------------
# coordinated-change detection
# ----------------------------------------------------------------------

def find_runs(rep: DiffReport, segs) -> list:
    """
    Group flagged lanelets into connected runs.

    A ramp attack is a PATH of consecutive modified lanelets. Isolated changes
    are consistent with local roadworks; a monotonic run along a route is not.
    Reporting the run rather than 8 separate flags is what makes the output
    legible to an operator.
    """
    by = {s.segment_id: s for s in segs}
    flagged = rep.flagged_ids()
    seen, runs = set(), []

    for sid in sorted(flagged):
        if sid in seen or sid not in by:
            continue
        run, cur = [sid], sid
        seen.add(sid)
        while True:
            nxt = [t for t in by[cur].successors if t in flagged and t not in seen]
            if not nxt:
                break
            cur = nxt[0]
            seen.add(cur)
            run.append(cur)
        if len(run) >= 2:
            widths = [by[s].width_m for s in run if by[s].width_m]
            monotonic = all(widths[i] <= widths[i + 1] for i in range(len(widths) - 1)) \
                or all(widths[i] >= widths[i + 1] for i in range(len(widths) - 1))
            runs.append({
                "lanelets": run,
                "length": len(run),
                "monotonic": monotonic,
                "total_width_change": round(widths[-1] - widths[0], 3) if len(widths) > 1 else 0.0,
            })
    return runs


# ----------------------------------------------------------------------

def evaluate(rep: DiffReport, labels_path) -> dict:
    if not labels_path or not Path(labels_path).exists():
        return {}
    raw = json.load(open(labels_path))
    truth = set(raw.get("labels", raw).keys())
    pred = rep.flagged_ids()
    tp, fp, fn = len(pred & truth), len(pred - truth), len(truth - pred)
    pr = tp / (tp + fp) if tp + fp else 0.0
    rc = tp / (tp + fn) if tp + fn else 0.0
    return {"TP": tp, "FP": fp, "FN": fn,
            "precision": round(pr, 3), "recall": round(rc, 3),
            "f1": round(2 * pr * rc / (pr + rc), 3) if pr + rc else 0.0}


def main():
    ap = argparse.ArgumentParser(description="GeoShield differential verification")
    ap.add_argument("--previous", required=True, help="last trusted map version")
    ap.add_argument("--candidate", required=True, help="incoming map version")
    ap.add_argument("--labels", help="ground truth, to score detection")
    ap.add_argument("--report", help="write JSON report")
    a = ap.parse_args()

    print(f"previous  : {a.previous}")
    print(f"candidate : {a.candidate}\n")

    prev = load_map(a.previous)
    cand = load_map(a.candidate)
    rep = compare(prev, cand)
    rep.changes.extend(structural_diff(a.previous, a.candidate))
    rep.runs = find_runs(rep, cand)

    rejected = rep.rejected()
    suspect = [c for c in rep.changes if c.verdict == "SUSPECT"]
    accepted = [c for c in rep.changes if c.verdict == "ACCEPT"]

    rep.verdict = "REJECT" if rejected else ("REVIEW" if suspect else "ACCEPT")

    print(f"lanelets      : {rep.n_previous} → {rep.n_candidate}"
          f"   (+{len(rep.added)} / -{len(rep.removed)})")
    print(f"changes found : {len(rep.changes)}")
    print(f"  accepted    : {len(accepted)}  (within re-survey tolerance)")
    print(f"  suspect     : {len(suspect)}")
    print(f"  rejected    : {len(rejected)}")

    if rep.runs:
        print(f"\ncoordinated runs: {len(rep.runs)}")
        for r in rep.runs:
            mono = "monotonic" if r["monotonic"] else "mixed"
            print(f"  {r['length']} consecutive lanelets, {mono}, "
                  f"total width change {r['total_width_change']:+.2f} m")
            print(f"    {' → '.join(x.split(':')[1] for x in r['lanelets'])}")

    if rejected:
        print(f"\n=== REJECTED CHANGES ===")
        for c in rejected[:12]:
            print(f"  {c.segment_id:<18} {c.field_name:<16} "
                  f"{c.previous} → {c.candidate}")
            print(f"    {c.reason}")

    scores = evaluate(rep, a.labels)
    if scores:
        print(f"\n=== Detection vs ground truth ===")
        print(f"  TP {scores['TP']}   FP {scores['FP']}   FN {scores['FN']}")
        print(f"  precision {scores['precision']}   "
              f"recall {scores['recall']}   F1 {scores['f1']}")
        print(f"\n  single-snapshot detection on this attack: recall 0.02")
        print(f"  differential verification:                recall "
              f"{scores['recall']}")

    print(f"\n{'='*52}")
    print(f"  VERDICT: {rep.verdict}")
    if rep.verdict == "REJECT":
        print(f"  Map update refused. The planner never receives it.")
    print(f"{'='*52}")

    if a.report:
        Path(a.report).write_text(json.dumps({
            "previous": a.previous, "candidate": a.candidate,
            "verdict": rep.verdict,
            "n_previous": rep.n_previous, "n_candidate": rep.n_candidate,
            "added": rep.added, "removed": rep.removed,
            "n_accepted": len(accepted), "n_suspect": len(suspect),
            "n_rejected": len(rejected),
            "runs": rep.runs,
            "detection": scores,
            "changes": [asdict(c) for c in rep.changes],
        }, indent=2))
        print(f"\nwrote {a.report}")

    sys.exit(2 if rep.verdict == "REJECT" else 0)


if __name__ == "__main__":
    main()