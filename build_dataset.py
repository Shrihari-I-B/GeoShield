#!/usr/bin/env python3
"""
GeoShield -- Phase 4c: evaluation dataset generator.

    python3 build_dataset.py \
        --map ~/autoware_map/nishishinjuku_autoware_map/lanelet2_map.osm \
        --runs 30 --out data/dataset

WHY THIS EXISTS
---------------
The single-run evaluation had 6 tampered lanelets out of 979 (0.61%). At that
prevalence every metric is noise: one misclassification moves recall by 17
points, and precision is capped so low that even a perfectly ranking detector
scores badly. We were measuring sampling variance, not detector quality.

There is also no public dataset of tampered HD maps to fall back on. Sato et
al. faced the same gap and hand-edited lanelets in Vector Map Builder. This
generator does the same thing reproducibly and at scale.

DESIGN
------
Each RUN is an independent attack campaign on the same clean map with its own
seed. Runs are then POOLED: every (run, segment) pair becomes one evaluation
example. 30 runs x ~979 segments gives ~29k examples with ~200-900 positives,
which is enough for precision, recall and PR-AUC to mean something.

Pooling across runs rather than tampering one map harder is deliberate. A
single map with 30% of its lanelets attacked is not a threat model anyone
believes; 30 realistic 1-3% campaigns is. Prevalence stays honest and the
variance across runs becomes measurable -- report mean +/- std, not one number.

ANTI-LEAKAGE
------------
* Every attack parameter is resampled per run (ramp length, total gain, speed
  factor, target selection). A model cannot memorise a fixed signature.
* Clean features are computed ONCE from the untampered map and reused as the
  training set, so the model never sees a tampered example while fitting.
* Runs are split into train/test by RUN ID, never by segment. Splitting by
  segment would put the same lanelet's clean and tampered versions on opposite
  sides of the split.
"""

from __future__ import annotations

import argparse
import json
import random
import sys
import time
from collections import Counter
from pathlib import Path

from lanelet2_adapter import load as load_map
from attack_injector import ATTACKS, campaign
from features import FEATURE_NAMES, extract


def build(map_path: str, n_runs: int, budget: float, base_seed: int,
          attack: str | None, out_dir: Path) -> dict:
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"loading clean map: {map_path}", file=sys.stderr)
    clean = load_map(map_path)
    print(f"  {len(clean)} segments", file=sys.stderr)

    # ---- clean reference features: the Tier 2 training set ----
    t0 = time.time()
    clean_ids, clean_rows = extract(clean)
    print(f"  clean features in {time.time()-t0:.1f}s", file=sys.stderr)

    runs, all_rows, all_labels, all_run_ids, all_seg_ids = [], [], [], [], []
    all_mags, all_atypes = [], []
    attack_counter = Counter()

    for r in range(n_runs):
        seed = base_seed + r
        rng = random.Random(seed)

        if attack:
            res = ATTACKS[attack](clean, rng)
        else:
            res = campaign(clean, rng, budget)

        truth = set(res.labels.keys())
        if not truth:
            print(f"  run {r:>3} seed {seed}: nothing tampered, skipped",
                  file=sys.stderr)
            continue

        ids, rows = extract(res.segments)
        for sid, row in zip(ids, rows):
            all_seg_ids.append(sid)
            all_run_ids.append(r)
            all_rows.append(row)
            all_labels.append(1 if sid in truth else 0)
            # Per-example attack magnitude. Averaging detection over attacks
            # spanning two orders of magnitude hides whether a detectability
            # threshold exists, so record |tampered - original| per segment.
            rec = res.labels.get(sid)
            mag = 0.0
            if rec is not None:
                try:
                    mag = abs(float(rec.tampered) - float(rec.original))
                except (TypeError, ValueError):
                    mag = 1.0          # categorical flip: oneway, tunnel/bridge
            all_mags.append(mag)
            all_atypes.append(rec.attack_type if rec is not None else "")

        for rec in res.labels.values():
            attack_counter[rec.attack_type] += 1

        runs.append({
            "run": r, "seed": seed,
            "n_tampered": len(truth),
            "rate": len(truth) / len(res.segments),
            "attacks": dict(Counter(v.attack_type for v in res.labels.values())),
        })
        print(f"  run {r:>3} seed {seed}: {len(truth):>3} tampered "
              f"({100*len(truth)/len(res.segments):.2f}%)", file=sys.stderr)

    n_pos = sum(all_labels)
    meta = {
        "map": map_path,
        "n_segments": len(clean),
        "n_runs": len(runs),
        "n_examples": len(all_rows),
        "n_positive": n_pos,
        "prevalence": n_pos / len(all_rows) if all_rows else 0.0,
        "budget": budget,
        "base_seed": base_seed,
        "attack": attack or "campaign",
        "features": FEATURE_NAMES,
        "attack_counts": dict(attack_counter),
        "runs": runs,
    }

    # ---- write ----
    with open(out_dir / "clean_features.json", "w") as fh:
        json.dump({"ids": clean_ids, "rows": clean_rows,
                   "features": FEATURE_NAMES}, fh)
    with open(out_dir / "examples.json", "w") as fh:
        json.dump({"segment_ids": all_seg_ids, "run_ids": all_run_ids,
                   "rows": all_rows, "labels": all_labels,
                   "magnitudes": all_mags, "attack_types": all_atypes,
                   "features": FEATURE_NAMES}, fh)
    with open(out_dir / "meta.json", "w") as fh:
        json.dump(meta, fh, indent=2)

    return meta


def main():
    ap = argparse.ArgumentParser(description="GeoShield dataset generator")
    ap.add_argument("--map", required=True, help="clean Lanelet2 .osm")
    ap.add_argument("--runs", type=int, default=30)
    ap.add_argument("--budget", type=float, default=0.03,
                    help="fraction of segments tampered per run")
    ap.add_argument("--seed", type=int, default=1000)
    ap.add_argument("--attack", choices=sorted(ATTACKS),
                    help="single attack type; omit for mixed campaigns")
    ap.add_argument("--out", default="data/dataset")
    a = ap.parse_args()

    meta = build(a.map, a.runs, a.budget, a.seed, a.attack, Path(a.out))

    print(f"\n=== Dataset ===")
    print(f"runs             : {meta['n_runs']}")
    print(f"examples         : {meta['n_examples']:,}")
    print(f"positives        : {meta['n_positive']:,}")
    print(f"prevalence       : {100*meta['prevalence']:.2f}%")
    print(f"features         : {len(meta['features'])}")
    print(f"\nattack mix:")
    for k, v in sorted(meta["attack_counts"].items(), key=lambda kv: -kv[1]):
        print(f"  {k:<24}{v:>6}")

    rates = [r["rate"] for r in meta["runs"]]
    if rates:
        import statistics
        print(f"\nper-run tamper rate: mean {100*statistics.fmean(rates):.2f}% "
              f"min {100*min(rates):.2f}% max {100*max(rates):.2f}%")
    print(f"\nwrote {a.out}/examples.json, clean_features.json, meta.json")


if __name__ == "__main__":
    main()