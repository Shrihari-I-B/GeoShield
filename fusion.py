#!/usr/bin/env python3
"""GeoShield P_attack -- combining the tiers into one per-segment score.

Tiers 1 and 2 run in parallel, never as a cascade: a cascade caps recall at
Tier 1. The differential check against a trusted prior runs alongside them when
a prior exists. P_attack fuses them with a weighted noisy-OR:

    P_attack(s) = 1 - prod_i (1 - w_i * s_i(s))

Noisy-OR rather than a mean, because these are near-independent evidence
sources of very different reliability, and averaging a near-perfect signal with
a near-chance one degrades the good one. One tier firing hard should carry the
decision; three firing weakly should not accumulate into an alarm.

The weights are design parameters, set from measured per-tier performance on
this map rather than tuned until the output looked good. The ablation this
script prints is what justifies them.

Measured on Nishi-Shinjuku, 2026-10-29:

    differential   recall 1.000, precision 0.471-1.000      -> w 1.0
    Tier 1         recall 1.000 on attribute and topology,
                   0.000 on geometry, clean-map FPR 0.0225  -> w 0.8
    Tier 2         PR-AUC 0.019, 2.3x the base rate         -> w 0.2

Tier 2's weight is low because its measured contribution is low. Carrying a
tier at a weight its evidence does not support would be dressing, not design.

Note on scales: Tier 1 gives severity in [0,1] and Tier 2 an anomaly score in
[0,1], but differential_verify produces a categorical verdict. Quantising
REJECT -> 1.0, SUSPECT -> 0.5, absent -> 0.0 is a design decision, not a
measurement, and the fused score inherits that coarseness.
"""

import argparse
import json
import os
from typing import Optional

W_DIFFERENTIAL, W_TIER1, W_TIER2 = 1.0, 0.8, 0.2
VERDICT_SCORE = {"REJECT": 1.0, "SUSPECT": 0.5, "ACCEPT": 0.0}


# The configuration established on 2026-10-29: R1, R2 and R4_speed_implausible
# were cut on evidence (R1 alone flags ~145 lanelets on any map, attack or not).
# Feeding the full rule set into a fusion imports a false-positive carpet that
# the rule-level ablation already rejected.
TIER1_MINIMAL = ["R4_speed_neighbour", "R5_attribute_neighbour"]


def tier1_scores(segments, cal, rules=None) -> dict:
    from tier1_rules import run_tier1
    return run_tier1(segments, cal, enabled=rules or TIER1_MINIMAL).scores(segments)


def tier2_scores(clean_segs, cand_segs, contamination=0.03, seed=42) -> dict:
    """Anomaly in [0,1], normalised against the CLEAN score distribution.

    tier2_iforest.fit_score rescales against the candidate's own min and max,
    so its top segment scores 1.0 even on an untampered map. Referencing the
    clean set instead makes scores comparable across maps, which fusion needs.
    Ranking is unchanged, so PR-AUC is identical either way.
    """
    from features import extract, standardise
    from tier2_iforest import fit_score

    _scores, model, stats = fit_score(clean_segs, cand_segs,
                                      contamination=contamination, seed=seed)
    _, clean_rows = extract(clean_segs)
    clean_std, _ = standardise(clean_rows, stats)
    ref = model.score_samples(clean_std)
    lo, hi = min(ref), max(ref)
    span = (hi - lo) or 1.0

    ids, rows = extract(cand_segs)
    std, _ = standardise(rows, stats)
    raw = model.score_samples(std)
    return {i: max(0.0, min(1.0, (hi - r) / span)) for i, r in zip(ids, raw)}


def differential_scores(report_path: str, segments) -> dict:
    """Categorical verdicts quantised onto [0,1]."""
    s = {seg.segment_id: 0.0 for seg in segments}
    rep = json.load(open(report_path))
    for ch in rep.get("changes", []):
        v = VERDICT_SCORE.get(ch.get("verdict"), 0.0)
        sid = ch.get("segment_id")
        if sid in s:
            s[sid] = max(s[sid], v)
    return s


def p_attack(parts: dict, segments) -> dict:
    """parts: {name: (scores, weight)}. Weighted noisy-OR."""
    out = {}
    for seg in segments:
        q = 1.0
        for scores, w in parts.values():
            q *= (1.0 - w * scores.get(seg.segment_id, 0.0))
        out[seg.segment_id] = 1.0 - q
    return out


def average_precision(scores: dict, truth: set) -> float:
    ranked = sorted(scores.items(), key=lambda kv: (-kv[1], kv[0]))
    P = len(truth)
    tp = fp = 0
    acc = 0.0
    for sid, _ in ranked:
        if sid in truth:
            tp += 1
            acc += tp / (tp + fp)
        else:
            fp += 1
    return acc / P if P else 0.0


def at_threshold(scores: dict, truth: set, thr: float) -> dict:
    flagged = {s for s, v in scores.items() if v >= thr}
    tp, fp = len(flagged & truth), len(flagged - truth)
    fn = len(truth - flagged)
    prec = tp / len(flagged) if flagged else 0.0
    rec = tp / len(truth) if truth else 0.0
    return {"flagged": len(flagged), "tp": tp, "fp": fp, "fn": fn,
            "precision": prec, "recall": rec,
            "f1": 2 * prec * rec / (prec + rec) if prec + rec else 0.0,
            "fpr": fp / (len(scores) - len(truth)) if scores else 0.0}


def main():
    ap = argparse.ArgumentParser(description="GeoShield P_attack fusion")
    ap.add_argument("--clean", required=True, help="clean map, for Tier 2 and calibration")
    ap.add_argument("--candidate", required=True, help="map under test")
    ap.add_argument("--labels", required=True)
    ap.add_argument("--diff", help="differential_verify report; omit for the "
                                   "no-prior configuration")
    ap.add_argument("--threshold", type=float, default=0.5)
    ap.add_argument("--all-rules", action="store_true", dest="all_rules",
                    help="use all five Tier 1 rules instead of the minimal set")
    ap.add_argument("--out")
    a = ap.parse_args()

    from tier1_rules import calibrate, load_map, load_truth

    clean = load_map(a.clean)
    cand = load_map(a.candidate)
    truth = load_truth(a.labels)
    cal = calibrate(clean)

    avail = {
        "tier1": (tier1_scores(cand, cal,
                                None if a.all_rules else TIER1_MINIMAL), W_TIER1),
        "tier2": (tier2_scores(clean, cand), W_TIER2),
    }
    if a.diff:
        avail["differential"] = (differential_scores(a.diff, cand), W_DIFFERENTIAL)

    base = len(truth) / len(cand)
    print(f"segments {len(cand)}   tampered {len(truth)}   base rate {base:.4f}")
    print(f"weights: " + ", ".join(f"{k}={w}" for k, (_, w) in avail.items()))

    combos = []
    names = list(avail)
    for i in range(1, 1 << len(names)):
        sel = [names[j] for j in range(len(names)) if i >> j & 1]
        combos.append(sel)
    combos.sort(key=lambda c: (len(c), c))

    print(f"\n{'configuration':<34}{'AP':>8}{'lift':>7}{'flag':>7}"
          f"{'TP':>4}{'FP':>6}{'prec':>7}{'rec':>7}{'FPR':>8}")
    print("-" * 87)
    rows = []
    for sel in combos:
        sc = p_attack({k: avail[k] for k in sel}, cand)
        ap_ = average_precision(sc, truth)
        m = at_threshold(sc, truth, a.threshold)
        label = " + ".join(sel)
        print(f"{label:<34}{ap_:>8.4f}{ap_/base:>7.1f}{m['flagged']:>7}"
              f"{m['tp']:>4}{m['fp']:>6}{m['precision']:>7.3f}"
              f"{m['recall']:>7.3f}{m['fpr']:>8.4f}")
        rows.append({"config": sel, "average_precision": ap_, **m})

    if a.out:
        json.dump({"base_rate": base, "threshold": a.threshold,
                   "weights": {k: w for k, (_, w) in avail.items()},
                   "rows": rows}, open(a.out, "w"), indent=2)
        print(f"\nwrote {a.out}")


if __name__ == "__main__":
    main()
