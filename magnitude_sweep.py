#!/usr/bin/env python3
"""Detection against attack magnitude, for the width-ramp family.

Every width result so far is measured at a single total gain of 3.0 m. The
question a reviewer asks next is where detection breaks down -- and the
reference point is Sato et al., whose smallest effective attack is +0.5 m.

Sweeps total_gain over the same eight route lanelets, injects through the
real writer, verifies against the trusted prior, and records what was
actually realised in the written map alongside what was intended. Realisation
matters here: an attacker cannot make an isolated geometric edit stick, and
even a run tapers at its outer ends.
"""

import argparse
import json
import subprocess
import sys
from pathlib import Path

TARGETS = ("3012234,3013054,3013093,3012977,"
           "3002017,3013032,3002007,3002013")


def run(cmd):
    return subprocess.run(cmd, capture_output=True, text=True)


def main():
    ap = argparse.ArgumentParser(description="detection vs attack magnitude")
    ap.add_argument("--clean", required=True)
    ap.add_argument("--gains", default="0.25,0.5,0.75,1.0,1.5,2.0,3.0,4.5,6.0")
    ap.add_argument("--out", default="results/magnitude_sweep.json")
    a = ap.parse_args()

    from tier1_rules import load_map
    clean = {s.segment_id: s for s in load_map(a.clean)}

    rows = []
    print(f"{'gain':>6}{'max delta':>10}{'ratio':>8}{'flag':>6}{'TP':>4}"
          f"{'FP':>5}{'prec':>7}{'recall':>8}  verdict")
    print("-" * 64)

    for g in [float(x) for x in a.gains.split(",")]:
        tag = f"sweep_g{g:g}".replace(".", "p")
        pre = f"data/{tag}"
        osm = f"data/{tag}.osm"
        rep = f"results/{tag}.json"

        r = run([sys.executable, "attack_injector.py", "--lanelet2", a.clean,
                 "--attack", "width_ramp", "--target", TARGETS,
                 "--total-gain", str(g), "--out", pre, "--out-map", osm])
        if not Path(osm).exists():
            print(f"{g:>6.2f}   injector failed: {r.stderr.strip()[:40]}")
            continue

        run([sys.executable, "differential_verify.py", "--previous", a.clean,
             "--candidate", osm, "--labels", f"{pre}_labels.json",
             "--report", rep])
        if not Path(rep).exists():
            print(f"{g:>6.2f}   no report"); continue
        d = json.load(open(rep))

        truth = set(json.load(open(f"{pre}_labels.json")))
        flagged = {c["segment_id"] for c in d.get("changes", [])
                   if c.get("verdict") in ("REJECT", "SUSPECT")}
        tp, fp = len(flagged & truth), len(flagged - truth)
        prec = tp / len(flagged) if flagged else 0.0
        rec = tp / len(truth) if truth else 0.0

        tam = {s.segment_id: s for s in load_map(osm)}
        # The ramp ends at total_gain, so the END of the run is what should
        # be compared against it. Summing all eight per-lanelet deltas and
        # dividing by the end-to-end gain gives ~4.0 at every magnitude --
        # the sum of a linear ramp over n lanelets is n*G/2, not G.
        deltas = [tam[s].width_m - clean[s].width_m
                  for s in truth if s in tam and s in clean]
        realised = max(deltas) if deltas else 0.0

        print(f"{g:>6.2f}{realised:>10.3f}{realised/g:>8.3f}{len(flagged):>6}"
              f"{tp:>4}{fp:>5}{prec:>7.3f}{rec:>8.3f}  {d.get('verdict')}")
        rows.append({"total_gain_m": g, "realised_total_m": round(realised, 4),
                     "realisation_ratio": round(realised / g, 4),
                     "flagged": len(flagged), "tp": tp, "fp": fp,
                     "precision": round(prec, 4), "recall": round(rec, 4),
                     "verdict": d.get("verdict")})

    json.dump({"targets": TARGETS.split(","), "rows": rows},
              open(a.out, "w"), indent=2)
    print(f"\nwrote {a.out}")
    sato = [r for r in rows if abs(r["total_gain_m"] - 0.5) < 1e-6]
    if sato:
        s = sato[0]
        print(f"\nAt Sato et al.'s smallest effective attack (+0.5 m): "
              f"recall {s['recall']:.3f}, verdict {s['verdict']}")


if __name__ == "__main__":
    main()
