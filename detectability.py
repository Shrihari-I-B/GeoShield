#!/usr/bin/env python3
"""
GeoShield -- Phase 4e: magnitude-stratified detectability analysis.

    python3 detectability.py --dataset data/dataset \
        --map ~/autoware_map/nishishinjuku_autoware_map/lanelet2_map.osm

THE QUESTION THIS ANSWERS
-------------------------
The pooled evaluation reported best F1 = 0.158 and PR-AUC = 0.074 across 955
tampered segments. Two very different explanations fit that number equally
well, and they lead to opposite conclusions:

  (A) DETECTION IS HARD. The signal exists but our detectors cannot separate
      it. Fix: better features, sequence models, more layers.

  (B) THE SIGNAL IS ABSENT. The injector samples ramp steps from a range whose
      lower end (0.10 m) sits BELOW the clean map's median honest width delta
      (0.138 m). Many tampered segments may have changed by less than ordinary
      map noise. Nothing can detect a change smaller than the variation it
      hides in -- not because detection is hard, but because there is nothing
      there.

Averaging over all 955 positives cannot tell these apart, because it mixes
attacks spanning two orders of magnitude into one number.

So we stratify by ACTUAL PER-SEGMENT CHANGE (|tampered - original|), expressed
both in metres and as a percentile of the clean delta distribution, and measure
detection separately in each band.

WHAT THE RESULT MEANS
---------------------
  Detection rises with magnitude
      -> a real DETECTABILITY THRESHOLD exists. Report it as a curve, and
         state the smallest change GeoShield reliably catches. Then compare
         against Sato et al.: attacks below +0.5 m did not meaningfully move
         the planned trajectory. If the undetectable band sits inside the
         ineffective band, the finding is strong -- every attack large enough
         to endanger the vehicle is large enough to catch.

  Detection flat across magnitude
      -> the FEATURES are wrong, not the attack. Even a +3 m change being
         missed means the feature set does not encode the signature at all.
         (Note `centreline_gap_succ` measured 0.000 everywhere -- the one
         feature aimed at the injector's actual edit signature is dead.)
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
from pathlib import Path


# ----------------------------------------------------------------------

def clean_delta_distribution(map_path: str) -> list[float]:
    """Honest lanelet-to-lanelet width deltas on the untampered map."""
    from lanelet2_adapter import load
    segs = load(map_path)
    by = {s.segment_id: s for s in segs}
    out = []
    for s in segs:
        if s.width_m is None or s.highway_class not in ("road", None):
            continue
        for t in s.successors:
            u = by.get(t)
            if u and u.width_m is not None:
                out.append(abs(s.width_m - u.width_m))
    return sorted(out)


def percentile_of(value: float, dist: list[float]) -> float:
    """What fraction of honest deltas are smaller than `value`."""
    if not dist:
        return 0.0
    lo, hi = 0, len(dist)
    while lo < hi:
        mid = (lo + hi) // 2
        if dist[mid] < value:
            lo = mid + 1
        else:
            hi = mid
    return 100.0 * lo / len(dist)


# ----------------------------------------------------------------------

def load_dataset(d: Path):
    raw = json.load(open(d / "examples.json"))
    meta = json.load(open(d / "meta.json"))
    clean = json.load(open(d / "clean_features.json"))
    mags = raw.get("magnitudes")
    types = raw.get("attack_types")
    if mags is None:
        print("examples.json has no 'magnitudes'. Rebuild the dataset with the\n"
              "updated build_dataset.py.", file=sys.stderr)
        sys.exit(1)
    ex = [{"features": r, "label": l, "magnitude": m,
           "attack_type": t, "run": run, "segment_id": sid}
          for r, l, m, t, run, sid in zip(
              raw["rows"], raw["labels"], mags, types,
              raw["run_ids"], raw["segment_ids"])]
    if isinstance(clean, dict):
        clean = clean.get("rows", clean.get("features", clean))
    return ex, meta, clean


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


def score_iforest(examples, clean_rows, trees=300, contamination=0.03, seed=42):
    from sklearn.ensemble import IsolationForest
    cs, stats = standardise(clean_rows)
    model = IsolationForest(n_estimators=trees, contamination=contamination,
                            random_state=seed, n_jobs=-1).fit(cs)
    rows = [e["features"] for e in examples]
    xs, _ = standardise(rows, stats)
    raw = model.score_samples(xs)
    lo, hi = min(raw), max(raw)
    span = (hi - lo) or 1.0
    return [float((hi - r) / span) for r in raw]


def score_feature(examples, idx):
    return [abs(e["features"][idx]) for e in examples]


# ----------------------------------------------------------------------

def stratified(examples, scores, dist, bands, label):
    """
    Detection performance within each magnitude band.

    Negatives are shared across bands (a clean segment belongs to no band), so
    within a band we ask: of the positives whose change falls in this band, how
    many rank above the score threshold that yields a fixed 5% flag rate over
    the whole set? That keeps the operating point constant while the positive
    population varies, which is the only way the bands are comparable.
    """
    ranked = sorted(scores, reverse=True)
    thr = ranked[int(0.05 * len(ranked))]        # fixed 5% flag budget

    rows = []
    for lo, hi in bands:
        idxs = [i for i, e in enumerate(examples)
                if e["label"] == 1 and lo <= e.get("magnitude", 0.0) < hi]
        if not idxs:
            rows.append((lo, hi, 0, 0.0, None, None))
            continue
        caught = sum(1 for i in idxs if scores[i] >= thr)
        mags = [examples[i]["magnitude"] for i in idxs]
        pct = percentile_of(statistics.median(mags), dist)
        rows.append((lo, hi, len(idxs), caught / len(idxs),
                     statistics.median(mags), pct))

    print(f"\n=== Detection vs attack magnitude -- {label} ===")
    print(f"(flag budget fixed at 5% of all examples; threshold {thr:.3f})")
    print(f"{'band [m]':>14}{'n':>7}{'recall':>9}{'median':>9}{'pctile of':>12}")
    print(f"{'':>14}{'':>7}{'':>9}{'|delta|':>9}{'clean deltas':>12}")
    print("-" * 53)
    for lo, hi, n, rec, med, pct in rows:
        if n == 0:
            print(f"{lo:>6.2f}-{hi:<7.2f}{n:>7}{'--':>9}{'--':>9}{'--':>12}")
        else:
            print(f"{lo:>6.2f}-{hi:<7.2f}{n:>7}{rec:>9.3f}"
                  f"{med:>9.3f}{pct:>11.1f}%")
    return rows


def stratified_by_type(examples, scores, dist, bands, label, atype):
    """
    Detection within ONE attack type, banded by magnitude.

    Mixing attack types corrupts the magnitude axis. `magnitude` is
    |tampered - original| in whatever units the changed field uses: metres for
    width attacks, km/h for speed spoofs, 1.0 for categorical flips (oneway,
    tunnel/bridge). Pooling them drops a 50 km/h speed change into the same
    "5-100 m" band as a 30 m width change, then scores both with width
    features. The apparent recall COLLAPSE in the top band of the pooled run
    (width_vs_nbr_median: 0.640 at 2-5 m, then 0.052 at 5-100 m) was that
    artefact, not a detector failure.
    """
    sub = [(i, e) for i, e in enumerate(examples)
           if e["label"] == 1 and e.get("attack_type") == atype]
    if not sub:
        return []

    ranked = sorted(scores, reverse=True)
    thr = ranked[int(0.05 * len(ranked))]      # fixed 5% flag budget

    rows = []
    for lo, hi in bands:
        idxs = [i for i, e in sub if lo <= e["magnitude"] < hi]
        if not idxs:
            continue
        caught = sum(1 for i in idxs if scores[i] >= thr)
        mags = [examples[i]["magnitude"] for i in idxs]
        rows.append((lo, hi, len(idxs), caught / len(idxs),
                     statistics.median(mags),
                     percentile_of(statistics.median(mags), dist)))

    if not rows:
        return []

    unit = "m" if "width" in atype else ("km/h" if "speed" in atype else "flag")
    print(f"\n--- {label}   attack: {atype}   (magnitude in {unit}) ---")
    print(f"{'band':>15}{'n':>7}{'recall':>9}{'median':>10}")
    print("-" * 41)
    for lo, hi, n, rec, med, pct in rows:
        print(f"{lo:>6.2f}-{hi:<8.2f}{n:>7}{rec:>9.3f}{med:>10.3f}")
    return rows


def main():
    ap = argparse.ArgumentParser(description="magnitude-stratified detectability")
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--map", required=True)
    ap.add_argument("--out")
    a = ap.parse_args()

    d = Path(a.dataset)
    examples, meta, clean_rows = load_dataset(d)
    names = meta.get("feature_names") or meta.get("features") or []

    dist = clean_delta_distribution(a.map)
    print(f"clean honest width deltas: n={len(dist)}  "
          f"median {statistics.median(dist):.3f}  "
          f"p90 {dist[int(0.90*len(dist))]:.3f}  "
          f"p99 {dist[int(0.99*len(dist))]:.3f} m")

    from collections import Counter
    tc = Counter(e["attack_type"] for e in examples if e["label"] == 1)
    print("\n=== Attack types ===")
    for k, v in tc.most_common():
        ms = [e["magnitude"] for e in examples
              if e["label"] == 1 and e["attack_type"] == k]
        unit = "m" if "width" in k else ("km/h" if "speed" in k else "flag")
        print(f"  {k:<22}{v:>5}   |delta| min {min(ms):.2f}  "
              f"median {statistics.median(ms):.2f}  max {max(ms):.2f}  [{unit}]")
    print("\n  Units differ per attack type -- bands compare only WITHIN a type.")

    bands = [(0.0, 0.15), (0.15, 0.3), (0.3, 0.5), (0.5, 1.0),
             (1.0, 2.0), (2.0, 5.0), (5.0, 1e6)]

    scorers = {"iforest": score_iforest(examples, clean_rows)}
    for f in ("chain_trend_max", "rel_width_succ", "width_vs_nbr_median",
              "width_z_local", "speed_vs_nbr_median", "d_speed_succ"):
        if f in names:
            scorers[f] = score_feature(examples, names.index(f))

    width_types = [t for t in tc if "width" in t]
    speed_types = [t for t in tc if "speed" in t]
    other_types = [t for t in tc if t not in width_types + speed_types]

    plan = [("iforest", list(tc))]
    for f in ("chain_trend_max", "rel_width_succ", "width_vs_nbr_median",
              "width_z_local"):
        if f in scorers:
            plan.append((f, width_types))
    for f in ("speed_vs_nbr_median", "d_speed_succ"):
        if f in scorers:
            plan.append((f, speed_types))

    results = {}
    for det, types in plan:
        for at in types:
            r = stratified_by_type(examples, scorers[det], dist, bands, det, at)
            if r:
                results[f"{det}|{at}"] = r

    print("\n=== Interpretation ===")
    for key in ("width_vs_nbr_median|width_ramp", "rel_width_succ|width_ramp",
                "iforest|width_ramp"):
        r = results.get(key)
        if not r:
            continue
        small = [x for x in r if x[1] <= 1.0]
        large = [x for x in r if x[0] >= 1.0]
        if not (small and large):
            continue
        rs = statistics.fmean([x[3] for x in small])
        rl = statistics.fmean([x[3] for x in large])
        print(f"  {key}")
        print(f"    recall, ramp < 1.0 m : {rs:.3f}")
        print(f"    recall, ramp > 1.0 m : {rl:.3f}")
        verdict = ("DETECTABILITY THRESHOLD" if rl > rs + 0.15
                   else "flat -- features do not capture the signature")
        print(f"    -> {verdict}")

    print("\n  If a threshold exists: state the smallest reliably detected")
    print("  displacement, and compare with Sato et al. -- attacks below")
    print("  +0.5 m did not move the trajectory enough to matter. An")
    print("  undetectable band sitting INSIDE the ineffective band means")
    print("  every attack big enough to endanger the vehicle is big enough")
    print("  to catch, which is the strong version of the result.")

    if a.out:
        json.dump({k: [list(x) for x in v] for k, v in results.items()},
                  open(a.out, "w"), indent=2)
        print(f"\nwrote {a.out}", file=sys.stderr)


if __name__ == "__main__":
    main()