#!/usr/bin/env python3
"""
GeoShield -- Phase 4b: Tier 2 anomaly detection (Isolation Forest).

    python3 tier2_iforest.py \
        --clean ~/autoware_map/nishishinjuku_autoware_map/lanelet2_map.osm \
        --tampered data/tampered_ramp.osm \
        --labels data/ramp_labels.json


WHAT THIS HAS TO BEAT
---------------------
Phase 3 separability analysis (chain trend statistic, Nishi-Shinjuku):

    chain length   3      4      5      6      7
    best F1      0.196  0.158  0.188  0.208  0.220

Attacked chains sit INSIDE the 90th percentile of clean chains, so no single
statistic can separate them. Max achievable F1 with rules alone: 0.22.

Isolation Forest's chance of beating that is the JOINT distribution. A ramped
lanelet is unremarkable on trend, unremarkable on taper, unremarkable on
neighbour delta -- but the COMBINATION is rare. Isolation Forest partitions
the full 28-dimensional space, so it can isolate points that no single axis
would flag.

If it does not beat 0.22, that is a legitimate finding and the ablation should
report it. A layer that does not earn its place gets cut.


WHY NOVELTY DETECTION, NOT OUTLIER DETECTION
--------------------------------------------
We fit ONLY on the clean map. The model learns the shape of "normal", then
scores tampered segments as departures from it.

Fitting on the tampered set instead would let the attack shape the notion of
normal -- the model absorbs the ramp into its idea of a typical lanelet and
detection collapses. This distinction (novelty vs outlier detection) is the
single easiest way to get an invalid result here, so it is enforced in code:
`fit()` takes clean segments and nothing else.


HOW ISOLATION FOREST WORKS
--------------------------
Most anomaly detectors model normality and measure distance from it. Isolation
Forest inverts the question: how hard is this point to SEPARATE from the rest?

Build random binary trees. At each node pick a random feature and a random
split value. Recurse until every point sits alone. A point far from the crowd
gets isolated after few splits; a point buried in a dense cluster needs many.
So the expected path length E(h(x)) is itself an anomaly measure:

    c(n) = 2H(n-1) - 2(n-1)/n,     H(i) ~ ln(i) + 0.5772
    s(x,n) = 2^(-E(h(x)) / c(n))

s -> 1 means clearly anomalous, s -> 0.5 means unremarkable.
"""

from __future__ import annotations

import argparse
import json
import sys

from features import FEATURE_NAMES, extract, standardise


def _need_sklearn():
    try:
        from sklearn.ensemble import IsolationForest       # noqa: F401
        return True
    except ImportError:
        print("scikit-learn is required:\n"
              "  pip install scikit-learn\n"
              "  (Ubuntu 24.04: python3 -m venv .venv && source .venv/bin/activate first)",
              file=sys.stderr)
        return False


def fit_score(clean_segs, tampered_segs, contamination=0.03,
              n_estimators=300, seed=42):
    """
    Fit on clean, score tampered. Returns {segment_id: anomaly_score}.

    contamination sets where sklearn puts its own decision boundary. It should
    match the attack budget declared in Phase 2 (0.03), because that is the
    class imbalance the system is designed for -- it is an assumption, not a
    tuning knob, and the sensitivity of results to it belongs in the report.
    """
    from sklearn.ensemble import IsolationForest

    _, clean_rows = extract(clean_segs)
    clean_std, stats = standardise(clean_rows)

    model = IsolationForest(
        n_estimators=n_estimators,
        contamination=contamination,
        max_samples="auto",
        random_state=seed,          # seeded: results must be reproducible
        n_jobs=-1,
    ).fit(clean_std)

    ids, rows = extract(tampered_segs)
    std, _ = standardise(rows, stats)        # clean statistics, no leakage

    # score_samples: higher = more normal. Flip and rescale to 0..1.
    raw = model.score_samples(std)
    lo, hi = min(raw), max(raw)
    span = (hi - lo) or 1.0
    return {i: float((hi - r) / span) for i, r in zip(ids, raw)}, model, stats


def evaluate(scores: dict[str, float], truth: set[str], thr: float) -> dict:
    pred = {s for s, v in scores.items() if v >= thr}
    tp = len(pred & truth)
    fp = len(pred - truth)
    fn = len(truth - pred)
    tn = len(scores) - tp - fp - fn
    p = tp / (tp + fp) if tp + fp else 0.0
    r = tp / (tp + fn) if tp + fn else 0.0
    return {"thr": thr, "TP": tp, "FP": fp, "FN": fn, "TN": tn,
            "precision": p, "recall": r,
            "f1": 2 * p * r / (p + r) if p + r else 0.0,
            "fpr": fp / (fp + tn) if fp + tn else 0.0}


def pr_auc(scores: dict[str, float], truth: set[str]) -> float:
    """
    Area under the precision-recall curve.

    PR-AUC, not ROC-AUC. With ~3% positives the false-positive rate has a huge
    denominator, so ROC-AUC looks impressive even for a poor detector. PR is
    the honest metric for rare positives.
    """
    ranked = sorted(scores.items(), key=lambda kv: -kv[1])
    tp = fp = 0
    total = len(truth)
    if not total:
        return 0.0
    area, prev_r = 0.0, 0.0
    for sid, _ in ranked:
        if sid in truth:
            tp += 1
        else:
            fp += 1
        p = tp / (tp + fp)
        r = tp / total
        area += p * (r - prev_r)
        prev_r = r
    return area


def main():
    ap = argparse.ArgumentParser(description="GeoShield Tier 2 -- Isolation Forest")
    ap.add_argument("--clean", required=True)
    ap.add_argument("--tampered", required=True)
    ap.add_argument("--labels", required=True)
    ap.add_argument("--contamination", type=float, default=0.03)
    ap.add_argument("--trees", type=int, default=300)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--out")
    a = ap.parse_args()

    if not _need_sklearn():
        sys.exit(1)

    from lanelet2_adapter import load
    clean = load(a.clean)
    tampered = load(a.tampered)

    raw = json.load(open(a.labels))
    recs = raw.get("labels", raw)
    truth = set(recs.keys())

    print(f"clean {len(clean)}  tampered map {len(tampered)}  "
          f"tampered segs {len(truth)} ({100*len(truth)/len(tampered):.2f}%)")
    print(f"features: {len(FEATURE_NAMES)}")

    scores, model, _ = fit_score(clean, tampered, a.contamination,
                                 a.trees, a.seed)

    print("\n=== Threshold sweep ===")
    print(f"{'thr':>6}{'TP':>5}{'FP':>6}{'FN':>5}"
          f"{'prec':>9}{'recall':>9}{'F1':>9}{'FPR':>9}")
    rows = []
    for t in [0.3, 0.4, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9]:
        m = evaluate(scores, truth, t)
        rows.append(m)
        print(f"{t:>6.2f}{m['TP']:>5}{m['FP']:>6}{m['FN']:>5}"
              f"{m['precision']:>9.3f}{m['recall']:>9.3f}"
              f"{m['f1']:>9.3f}{m['fpr']:>9.3f}")

    best = max(rows, key=lambda m: m["f1"])
    auc = pr_auc(scores, truth)

    print(f"\nbest F1     {best['f1']:.3f} at thr {best['thr']:.2f}"
          f"  (P {best['precision']:.3f}  R {best['recall']:.3f})")
    print(f"PR-AUC      {auc:.3f}")
    print(f"\nTier 1 ceiling (Phase 3 separability): F1 0.220")
    verdict = "BEATS the rule-based ceiling" if best["f1"] > 0.22 else \
              "does NOT beat the rule-based ceiling"
    print(f"Tier 2 {verdict}.")

    # rank of the tampered segments -- more informative than F1 when positives
    # are few: it says how close the attack is to the top of the suspicion list
    ranked = sorted(scores.items(), key=lambda kv: -kv[1])
    pos = {sid: i + 1 for i, (sid, _) in enumerate(ranked)}
    hits = sorted(pos[s] for s in truth if s in pos)
    if hits:
        print(f"\ntampered segment ranks (of {len(scores)}): {hits}")
        print(f"  best {hits[0]}  median {hits[len(hits)//2]}  worst {hits[-1]}")

    if a.out:
        json.dump({"sweep": rows, "pr_auc": auc, "best": best,
                   "scores": scores}, open(a.out, "w"), indent=2)
        print(f"\nwrote {a.out}", file=sys.stderr)


if __name__ == "__main__":
    main()