#!/usr/bin/env python3
"""Rank-based evaluation of Tier 1's trend statistic.

A single operating point tells you whether today's threshold works. It does
not tell you whether the statistic carries signal at all. This ranks every
lanelet by the strongest chain trend it participates in, then reports the
precision-recall curve and average precision -- the metric the evaluation
plan calls for, and the one no threshold choice can flatter.

A random ranker scores AP ~= the base rate. Anything near that is noise.
"""

import argparse
import json
from collections import defaultdict

from tier1_rules import (_chains, _chain_trend, _chain_trend_robust,
                         calibrate, load_map, load_truth)


def lanelet_scores(segments, length, trend=_chain_trend):
    """Each lanelet scores the strongest chain trend it takes part in."""
    by = {s.segment_id: s for s in segments}
    best = {s.segment_id: 0.0 for s in segments}
    chains = 0
    for chain in _chains(segments, by, length):
        t = trend(chain, by)
        if t is None:
            continue
        chains += 1
        for s in chain:
            if t > best[s.segment_id]:
                best[s.segment_id] = t
    return best, chains


def pr_curve(scores, truth):
    ranked = sorted(scores.items(), key=lambda kv: (-kv[1], kv[0]))
    P = len(truth)
    tp = fp = 0
    ap = 0.0
    pts = []
    for sid, sc in ranked:
        if sid in truth:
            tp += 1
            ap += tp / (tp + fp)
        else:
            fp += 1
        pts.append((sc, tp / (tp + fp), tp / P if P else 0.0, sid in truth))
    return pts, (ap / P if P else 0.0), ranked


def main():
    ap_ = argparse.ArgumentParser(description="Tier 1 rank-based evaluation")
    ap_.add_argument("--clean", required=True)
    ap_.add_argument("--tampered", required=True)
    ap_.add_argument("--labels", required=True)
    ap_.add_argument("--chain-length", type=int, default=5, dest="chain_length")
    ap_.add_argument("--statistic", choices=("ls", "median"), default="ls")
    ap_.add_argument("--out")
    a = ap_.parse_args()

    tam = load_map(a.tampered)
    truth = load_truth(a.labels)
    cal = calibrate(load_map(a.clean))

    fn = _chain_trend if a.statistic == 'ls' else _chain_trend_robust
    print(f'statistic: {a.statistic}')
    scores, n_chains = lanelet_scores(tam, a.chain_length, fn)
    pts, avg_p, ranked = pr_curve(scores, truth)

    base = len(truth) / len(scores)
    print(f"segments {len(scores)}   chains scored {n_chains}   "
          f"tampered {len(truth)}   base rate {base:.4f}")
    print(f"average precision {avg_p:.4f}   "
          f"(random ranker would score about {base:.4f})")
    print(f"lift over chance   {avg_p / base:.1f}x" if base else "")

    full = next((p for p in pts if p[2] >= 1.0), None)
    if full:
        print(f"at recall 1.000: score >= {full[0]:.4f}, precision {full[1]:.4f}")
    best = max(pts, key=lambda p: (2 * p[1] * p[2] / (p[1] + p[2])
                                   if p[1] + p[2] else 0.0))
    print(f"best F1 {2*best[1]*best[2]/(best[1]+best[2]):.4f} at score "
          f"{best[0]:.4f}  (precision {best[1]:.3f}, recall {best[2]:.3f})")

    print(f"\nclean-map calibration for reference: p90 {cal['trend_p90']:.4f}  "
          f"p95 {cal['trend_p95']:.4f}  p99 {cal['trend_p99']:.4f}  "
          f"p99.5 {cal['trend_p995']:.4f}")

    print("\nrank  score    lanelet                 tampered")
    for i, (sid, sc) in enumerate(ranked[:20], 1):
        print(f"{i:>4}  {sc:>7.4f}  {sid:<22}  {'YES' if sid in truth else ''}")
    miss = [(i, s, v) for i, (s, v) in enumerate(ranked, 1) if s in truth]
    print("\ntampered lanelets by rank: "
          + ", ".join(f"{s.split(':')[-1]}@{i}" for i, s, _ in miss))

    if a.out:
        json.dump({"average_precision": avg_p, "base_rate": base,
                   "chains_scored": n_chains, "calibration": cal,
                   "ranked": [[s, v] for s, v in ranked]},
                  open(a.out, "w"), indent=2)
        print(f"\nwrote {a.out}")


if __name__ == "__main__":
    main()
