#!/usr/bin/env python3
"""
GeoShield -- Phase 4d: pooled evaluation and ablation.

    python3 evaluate.py --dataset data/dataset

Consumes the pooled dataset and reports, for each detector configuration:
precision, recall, F1, FPR, PR-AUC, and the rank distribution of tampered
segments -- with mean and standard deviation ACROSS RUNS, so the numbers carry
a confidence statement rather than being a single lucky draw.

SPLITTING
---------
Train/test split is BY RUN, never by segment. Splitting by segment would put
lanelet X's clean version in train and its tampered version in test; the model
would effectively see the answer. Run-level splitting keeps every version of a
lanelet on the same side.

METRICS
-------
PR-AUC, not ROC-AUC, and never accuracy. At ~3% prevalence a detector that
flags nothing scores 97% accuracy, and ROC-AUC is flattered by the enormous
negative class in its denominator. Precision-recall is the honest curve for
rare positives.
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
from pathlib import Path


def standardise(rows, stats=None):
    n = len(rows[0])
    if stats is None:
        cols = [[r[i] for r in rows] for i in range(n)]
        mean = [statistics.fmean(c) for c in cols]
        sd = [statistics.pstdev(c) if len(c) > 1 else 1.0 for c in cols]
        sd = [x if x > 1e-9 else 1.0 for x in sd]
    else:
        mean, sd = stats
    return [[(r[i] - mean[i]) / sd[i] for i in range(n)] for r in rows], (mean, sd)


def prf(scores, labels, thr):
    tp = fp = fn = tn = 0
    for s, y in zip(scores, labels):
        pred = s >= thr
        if pred and y: tp += 1
        elif pred and not y: fp += 1
        elif not pred and y: fn += 1
        else: tn += 1
    p = tp / (tp + fp) if tp + fp else 0.0
    r = tp / (tp + fn) if tp + fn else 0.0
    return {"thr": thr, "TP": tp, "FP": fp, "FN": fn, "TN": tn,
            "precision": p, "recall": r,
            "f1": 2 * p * r / (p + r) if p + r else 0.0,
            "fpr": fp / (fp + tn) if fp + tn else 0.0}


def pr_auc(scores, labels):
    order = sorted(range(len(scores)), key=lambda i: -scores[i])
    total = sum(labels)
    if not total:
        return 0.0
    tp = fp = 0
    area, prev_r = 0.0, 0.0
    for i in order:
        if labels[i]: tp += 1
        else: fp += 1
        p = tp / (tp + fp)
        r = tp / total
        area += p * (r - prev_r)
        prev_r = r
    return area


def best_f1(scores, labels, steps=60):
    lo, hi = min(scores), max(scores)
    if hi <= lo:
        return prf(scores, labels, lo)
    best = None
    for i in range(steps):
        t = lo + (hi - lo) * i / (steps - 1)
        m = prf(scores, labels, t)
        if best is None or m["f1"] > best["f1"]:
            best = m
    return best


def rank_stats(scores, labels):
    order = sorted(range(len(scores)), key=lambda i: -scores[i])
    ranks = [i + 1 for i, j in enumerate(order) if labels[j]]
    if not ranks:
        return {}
    n = len(scores)
    return {"n": n, "best": ranks[0], "median": ranks[len(ranks) // 2],
            "worst": ranks[-1],
            "top1pct": sum(1 for r in ranks if r <= 0.01 * n) / len(ranks),
            "top5pct": sum(1 for r in ranks if r <= 0.05 * n) / len(ranks)}


# ----------------------------------------------------------------------

def score_iforest(train_rows, test_rows, contamination, trees, seed):
    from sklearn.ensemble import IsolationForest
    tr, stats = standardise(train_rows)
    te, _ = standardise(test_rows, stats)
    m = IsolationForest(n_estimators=trees, contamination=contamination,
                        random_state=seed, n_jobs=-1).fit(tr)
    raw = m.score_samples(te)
    lo, hi = float(min(raw)), float(max(raw))
    span = (hi - lo) or 1.0
    return [float((hi - r) / span) for r in raw]


def score_single_feature(rows, features, name):
    """Baseline: one hand-picked feature used directly as a suspicion score."""
    i = features.index(name)
    vals = [abs(r[i]) for r in rows]
    lo, hi = min(vals), max(vals)
    span = (hi - lo) or 1.0
    return [(v - lo) / span for v in vals]


def report(title, scores, labels, extra=""):
    b = best_f1(scores, labels)
    auc = pr_auc(scores, labels)
    rk = rank_stats(scores, labels)
    print(f"\n--- {title} ---{extra}")
    print(f"  best F1 {b['f1']:.3f}  (P {b['precision']:.3f}  R {b['recall']:.3f}"
          f"  FPR {b['fpr']:.4f})")
    print(f"  PR-AUC  {auc:.3f}")
    if rk:
        print(f"  tampered in top 1% {100*rk['top1pct']:.0f}%   "
              f"top 5% {100*rk['top5pct']:.0f}%")
    return {"f1": b["f1"], "precision": b["precision"], "recall": b["recall"],
            "fpr": b["fpr"], "pr_auc": auc, "ranks": rk}


def main():
    ap = argparse.ArgumentParser(description="GeoShield pooled evaluation")
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--contamination", type=float, default=0.03)
    ap.add_argument("--trees", type=int, default=300)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--test-frac", type=float, default=0.3, dest="test_frac")
    ap.add_argument("--out")
    a = ap.parse_args()

    d = Path(a.dataset)
    ex = json.load(open(d / "examples.json"))
    clean = json.load(open(d / "clean_features.json"))
    meta = json.load(open(d / "meta.json"))
    feats = ex["features"]

    rows, labels, run_ids = ex["rows"], ex["labels"], ex["run_ids"]
    print(f"dataset: {len(rows):,} examples, {sum(labels):,} positives "
          f"({100*sum(labels)/len(rows):.2f}%), {meta['n_runs']} runs")

    # ---- split BY RUN ----
    all_runs = sorted(set(run_ids))
    n_test = max(1, int(len(all_runs) * a.test_frac))
    test_runs = set(all_runs[-n_test:])
    idx_test = [i for i, r in enumerate(run_ids) if r in test_runs]
    print(f"test runs: {sorted(test_runs)}  ({len(idx_test):,} examples, "
          f"{sum(labels[i] for i in idx_test)} positives)")

    te_rows = [rows[i] for i in idx_test]
    te_labels = [labels[i] for i in idx_test]

    results = {}

    # ---- baselines: single hand-picked features ----
    for name in ("chain_trend_max", "width_z_local", "width_vs_nbr_median",
                 "rel_width_succ", "width_taper"):
        if name in feats:
            s = score_single_feature(te_rows, feats, name)
            results[f"rule:{name}"] = report(f"single feature: {name}",
                                             s, te_labels)

    # ---- Tier 2: Isolation Forest, trained on CLEAN ONLY ----
    try:
        s = score_iforest(clean["rows"], te_rows, a.contamination,
                          a.trees, a.seed)
        results["iforest_clean"] = report(
            "Isolation Forest (fit on clean map)", s, te_labels,
            extra=f"  [{a.trees} trees, contamination {a.contamination}]")
    except ImportError:
        print("\nscikit-learn missing: pip install scikit-learn", file=sys.stderr)

    # ---- summary ----
    print("\n=== Ablation ===")
    print(f"{'configuration':<38}{'F1':>8}{'PR-AUC':>9}{'recall':>9}")
    for k, v in sorted(results.items(), key=lambda kv: -kv[1]["f1"]):
        print(f"{k:<38}{v['f1']:>8.3f}{v['pr_auc']:>9.3f}{v['recall']:>9.3f}")

    base = max((v["f1"] for k, v in results.items() if k.startswith("rule:")),
               default=0.0)
    ml = results.get("iforest_clean", {}).get("f1", 0.0)
    print(f"\nbest single-feature rule F1 : {base:.3f}")
    print(f"Isolation Forest F1         : {ml:.3f}")
    print("VERDICT:", "ML layer earns its place." if ml > base * 1.1
          else "ML layer does NOT beat the rule baseline -- report and reconsider.")

    if a.out:
        json.dump({"meta": meta, "results": results}, open(a.out, "w"), indent=2)
        print(f"\nwrote {a.out}", file=sys.stderr)


if __name__ == "__main__":
    main()