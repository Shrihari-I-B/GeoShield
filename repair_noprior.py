#!/usr/bin/env python3
"""GeoShield repair without a trusted prior.

When a prior exists, repair_differential restores geometry exactly and this
module is unnecessary. A vehicle loading a map for the first time, or entering
a new region, has no prior -- and that is the case the architecture claims to
cover and never did.

The two Tier 1 rules that survived evaluation both detect by neighbour
consensus: a speed limit out of line with every neighbour, a categorical
attribute contradicting all of them. The same evidence that identifies the
anomaly also determines the correct value. That symmetry is what makes
no-prior repair possible at all.

    R4_speed_neighbour      -> the supported neighbour speed
    R5_attribute_neighbour  -> the unanimous neighbour value
    R5_topology (orphan)    -> NOT repairable; the map is refused
    geometry                -> not detected, so not repaired

Choosing the target. The rule fires when the lanelet disagrees with EVERY
neighbour, so the neighbourhood is unanimous against it -- but not necessarily
unanimous with itself. lanelet:124584, spoofed to 98.7 km/h, has neighbours
[60, 5]: the median is 32.5 and the original was 60. So the target is the
neighbour value with the most support, tie-broken by minimal correction.

Limits, stated rather than discovered: this repairs toward local agreement, not
toward truth. An attacker who tampers a whole neighbourhood consistently defeats
it, and a legitimate value that genuinely disagrees with all its neighbours will
be "repaired" into error. Those are the false positives, and they are measured
below as damage rather than hidden.
"""

import argparse
import json
import os
import xml.etree.ElementTree as ET
from collections import Counter

# Consensus repair is NOT idempotent: repairing a correctly-valued segment
# changes it. Differential repair restores an identical value and is a no-op,
# which is why precision does not matter there. Here it is the only thing that
# matters, so a rule may apply repairs only if its measured clean-map
# false-positive rate supports it.
#
# Measured on Nishi-Shinjuku, 2026-10-29, repairing with the clean map withheld:
#
#   R5_attribute_neighbour   FPR 0.0000   1 restored, 0 damaged   -> APPLY
#   R4_speed_neighbour       FPR 0.0256   1 restored, 15 damaged  -> PROPOSE
#   R5_topology (orphan)     not repairable at all                -> REFUSE
#
# The speed damage is not benign. It rewrote a 5 km/h private lane to 60 km/h,
# which is more dangerous than the tampering it was correcting. At a precision
# of 0.0625 an automatic repair does 15 wrong edits per right one, so those
# corrections are reported for review and never written.
APPLY_RULES = {"R5_attribute_neighbour"}
PROPOSE_RULES = {"R4_speed_neighbour"}
REPAIRABLE = APPLY_RULES | PROPOSE_RULES


def _consensus(flagged_value, neighbour_values):
    """Most-supported neighbour value, tie-broken by smallest correction."""
    if not neighbour_values:
        return None
    counts = Counter(neighbour_values)
    best = max(counts.values())
    cands = [v for v, n in counts.items() if n == best]
    if len(cands) == 1:
        return cands[0]
    try:
        return min(cands, key=lambda v: abs(float(v) - float(flagged_value)))
    except (TypeError, ValueError):
        return cands[0]


def plan(segments, findings):
    """[(segment_id, attribute, from, to)] plus the unrepairable ids."""
    by = {s.segment_id: s for s in segments}
    fixes, propose, refuse = [], [], []
    for f in findings:
        if f.rule not in REPAIRABLE:
            refuse.append((f.segment_id, f.rule))
            continue
        s = by.get(f.segment_id)
        if s is None:
            continue
        attr = "speed_limit_kph" if f.rule == "R4_speed_neighbour" else None
        if attr is None:
            # R5_attribute_neighbour names the attribute in its detail text
            for cand in ("oneway", "tunnel", "bridge"):
                if f"declares {cand}=" in f.detail:
                    attr = cand
                    break
        if attr is None:
            refuse.append((f.segment_id, f.rule))
            continue
        nb = [getattr(by[t], attr, None)
              for t in (s.successors + s.predecessors) if t in by]
        nb = [v for v in nb if v is not None]
        tgt = _consensus(getattr(s, attr, None), nb)
        if tgt is None:
            refuse.append((f.segment_id, f.rule))
            continue
        rec = (f.segment_id, attr, getattr(s, attr, None), tgt)
        (fixes if f.rule in APPLY_RULES else propose).append(rec)
    return fixes, propose, refuse


TAG_FOR = {"speed_limit_kph": "speed_limit", "oneway": "one_way",
           "tunnel": "tunnel", "bridge": "bridge"}


def _as_tag(attr, value):
    if attr == "speed_limit_kph":
        return f"{value:g}"               # km/h; parse_speed round-trips this
    return "yes" if value else "no"


def apply(in_path, out_path, fixes) -> int:
    tree = ET.parse(in_path)
    root = tree.getroot()
    rels = {int(r.get("id")): r for r in root.findall("relation")}
    n = 0
    for sid, attr, _old, new in fixes:
        rel = rels.get(int(sid.split(":")[1]))
        if rel is None:
            continue
        key = TAG_FOR.get(attr)
        if key is None:
            continue
        for t in rel.findall("tag"):
            if t.get("k") == key:
                t.set("v", _as_tag(attr, new))
                n += 1
                break
        else:
            ET.SubElement(rel, "tag", {"k": key, "v": _as_tag(attr, new)})
            n += 1
    tree.write(out_path, encoding="utf-8", xml_declaration=True)
    return n


def main():
    ap = argparse.ArgumentParser(description="GeoShield no-prior repair")
    ap.add_argument("--candidate", required=True)
    ap.add_argument("--calibrate-on", help="clean map for thresholds; defaults "
                                           "to the candidate itself")
    ap.add_argument("--out", required=True)
    ap.add_argument("--truth", help="clean map, for EVALUATION ONLY -- the "
                                    "repair never reads it")
    ap.add_argument("--labels")
    ap.add_argument("--report")
    a = ap.parse_args()

    from tier1_rules import calibrate, load_map, load_truth, run_tier1
    from fusion import TIER1_MINIMAL

    cand = load_map(a.candidate)
    cal = calibrate(load_map(a.calibrate_on) if a.calibrate_on else cand)
    res = run_tier1(cand, cal, enabled=TIER1_MINIMAL)

    fixes, propose, refuse = plan(cand, res.findings)
    written = apply(a.candidate, a.out, fixes)

    print(f"candidate : {a.candidate}")
    print(f"findings  : {len(res.findings)}")
    print(f"repaired  : {len(fixes)} ({written} tags written) -> {a.out}")
    print(f"proposed  : {len(propose)} (reported, NOT written -- the rule's "
          f"precision does not support automatic repair)")
    print(f"unrepairable: {len(refuse)}  "
          f"({', '.join(sorted({r for _, r in refuse})) or 'none'})")
    # Only a rule with a clean-map false-positive rate of zero can gate without
    # a prior. R5_orphan fires 10 times on the CLEAN map and R4_speed_neighbour
    # 15 times; with no prior there is no baseline to compare those counts
    # against, so they are advisory. R5_attribute_neighbour fires zero times on
    # the clean map, so a single firing is evidence on its own.
    gating = [f for f in res.findings if f.rule in APPLY_RULES]
    verdict = "REJECT" if gating else "ADMIT"
    print(f"verdict   : {verdict}"
          f"   ({len(gating)} gating finding(s); {len(propose) + len(refuse)} "
          f"advisory)")
    for sid, attr, old, new in fixes[:10]:
        print(f"    {sid:<20} {attr:<16} {old} -> {new}")

    if propose:
        print("  proposals for review:")
        for sid, attr, old, new in propose[:10]:
            print(f"    {sid:<20} {attr:<16} {old} -> {new}")
    rep = {"candidate": a.candidate, "out": a.out,
           "verdict": verdict,
           "applied": [[s, at, o, n] for s, at, o, n in fixes],
           "proposed": [[s, at, o, n] for s, at, o, n in propose],
           "unrepairable": refuse}

    if a.truth:
        clean = {s.segment_id: s for s in load_map(a.truth)}
        truth = load_truth(a.labels) if a.labels else set()
        restored = damaged = wrong = 0
        for sid, attr, _old, new in fixes:
            orig = getattr(clean.get(sid), attr, None)
            if orig is None:
                continue
            ok = (abs(float(orig) - float(new)) < 1e-6
                  if isinstance(orig, (int, float)) else orig == new)
            if sid in truth:
                restored += int(ok)
                wrong += int(not ok)
            else:
                damaged += int(not ok)
        print(f"\nagainst the clean map (never read by the repair):")
        print(f"  tampered lanelets restored correctly : {restored} of {len(truth)}")
        print(f"  tampered lanelets repaired wrongly   : {wrong}")
        print(f"  untampered lanelets damaged          : {damaged} "
              f"of {len(fixes) - restored - wrong}")
        rep["evaluation"] = {"restored": restored, "wrong": wrong,
                             "damaged": damaged, "truth": len(truth)}

    if a.report:
        json.dump(rep, open(a.report, "w"), indent=2)
        print(f"\nwrote {a.report}")


if __name__ == "__main__":
    main()
