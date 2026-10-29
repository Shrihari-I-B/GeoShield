#!/usr/bin/env python3
"""Tier 1 evaluated per attack type -- the table the evaluation plan asks for.

One attack family tells you about one attack family. This runs every labelled
attack map in data/ through Tier 1, overall and rule by rule, so a rule that
is blind to width ramps but decisive on speed spoofing is visible as such
rather than averaged into a single misleading number.
"""

import argparse
import glob
import json
import os

from tier1_rules import RULES, calibrate, evaluate, load_map, load_truth, run_tier1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--clean", required=True)
    ap.add_argument("--glob", default="data/dv_*.osm")
    ap.add_argument("--exclude", default="",
                    help="comma-separated rules to disable")
    ap.add_argument("--out")
    a = ap.parse_args()

    drop = {r for r in a.exclude.split(',') if r}
    active = [r for r in RULES if r not in drop]
    if drop:
        print(f'disabled: {sorted(drop)}')
    cal = calibrate(load_map(a.clean))
    rows = []

    for path in sorted(glob.glob(a.glob)):
        name = os.path.basename(path)[3:-4]
        labels = path[:-4] + "_labels.json"
        if not os.path.exists(labels):
            print(f"  (skipping {name}: no labels)")
            continue
        segs = load_map(path)
        truth = load_truth(labels)
        res = run_tier1(segs, cal, enabled=active)
        m = evaluate(res.scores(segs), truth)
        per = {}
        for rule in active:
            r = run_tier1(segs, cal, enabled=[rule])
            per[rule] = evaluate(r.scores(segs), truth)
        rows.append((name, len(segs), m, per))

    w = max((len(r[0]) for r in rows), default=20)
    print(f"\n{'attack':<{w}} {'n':>5} {'truth':>6} {'flag':>6} {'TP':>4} "
          f"{'FP':>5} {'prec':>6} {'rec':>6} {'F1':>6} {'FPR':>7}")
    print("-" * (w + 56))
    for name, n, m, _ in rows:
        print(f"{name:<{w}} {n:>5} {m['truth']:>6} {m['flagged']:>6} "
              f"{m['tp']:>4} {m['fp']:>5} {m['precision']:>6.3f} "
              f"{m['recall']:>6.3f} {m['f1']:>6.3f} {m['fpr']:>7.4f}")

    print(f"\nper-rule recall (TP/truth)\n{'attack':<{w}}", end="")
    for rule in active:
        print(f"{rule.replace('_', ' ')[:13]:>15}", end="")
    print()
    print("-" * (w + 15 * len(active)))
    for name, _, m, per in rows:
        print(f"{name:<{w}}", end="")
        for rule in active:
            p = per[rule]
            print(f"{p['recall']:>8.3f}/{p['fp']:<6}", end="")
        print()
    print(f"\n{'':<{w}}" + "".join(f"{'recall/FP':>15}" for _ in active))

    if a.out:
        json.dump({"calibration": cal,
                   "rows": [{"attack": n, "segments": s, "overall": m,
                             "per_rule": p} for n, s, m, p in rows]},
                  open(a.out, "w"), indent=2)
        print(f"\nwrote {a.out}")


if __name__ == "__main__":
    main()
