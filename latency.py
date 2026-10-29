#!/usr/bin/env python3
"""Stage latency for the GeoShield gate.

The evaluation plan asks for latency. The figure that matters is end to end,
read against the fact that the gate runs ONCE at map ingest, not per planning
cycle -- a few seconds added to a map load is a different proposition from a
few seconds added to a control loop.

Measured 2026-10-30, Nishi-Shinjuku, 979 lanelets, warm OSM cache:

    Tier 1 minimal, 2 rules      0.00 s    the entire no-prior detector
    Tier 3 match + coverage      0.04 s    not in the inference path
    Tier 2 Isolation Forest      0.91 s    for PR-AUC 0.018
    repair_differential          1.21 s
    differential_verify          1.66 s
    gate end to end              4.57 s

Figures move with import warmth. A first run inside one process charged
sklearn's import to Tier 2 and read 1.91 s, which is how "Tier 2 is the most
expensive stage" nearly became a quoted fact. Run this standalone, and twice.

The comparison that matters is Tier 1 against Tier 2: 0.00 s for the rules
that actually gate, 0.91 s for the model contributing PR-AUC 0.018.
"""

import argparse
import json
import os
import subprocess
import sys
import time


def main():
    ap = argparse.ArgumentParser(description="GeoShield stage latency")
    ap.add_argument("--clean", required=True)
    ap.add_argument("--candidate", required=True)
    ap.add_argument("--area", default="nishi-shinjuku")
    ap.add_argument("--out", default="results/latency.json")
    a = ap.parse_args()

    from tier1_rules import calibrate, load_map, run_tier1
    from fusion import TIER1_MINIMAL

    stages, state = [], {}

    def t(label, fn, note=""):
        s = time.perf_counter()
        state["r"] = fn()
        d = time.perf_counter() - s
        stages.append({"stage": label, "seconds": round(d, 3), "note": note})
        print(f"  {label:<42}{d:>8.2f} s   {note}")
        return state["r"]

    print(f"stage latency, {a.candidate}\n")
    clean = t("load clean map", lambda: load_map(a.clean))
    cand = t("load candidate map", lambda: load_map(a.candidate))
    cal = t("calibrate thresholds", lambda: calibrate(clean))
    t("Tier 1 minimal (2 rules)",
      lambda: run_tier1(cand, cal, enabled=TIER1_MINIMAL),
      "the whole no-prior detector")

    t("differential_verify", lambda: subprocess.run(
        [sys.executable, "differential_verify.py", "--previous", a.clean,
         "--candidate", a.candidate, "--report", "/tmp/lat_dv.json"],
        capture_output=True))
    t("repair_differential", lambda: subprocess.run(
        [sys.executable, "repair_differential.py", "--candidate", a.candidate,
         "--prior", a.clean, "--diff", "/tmp/lat_dv.json",
         "--include-suspect", "--out", "/tmp/lat_rep.osm"],
        capture_output=True))

    import tier2_iforest, tier3_osm_verify, osm_adapter
    t("Tier 2 Isolation Forest (fit + score)",
      lambda: tier2_iforest.fit_score(clean, cand), "PR-AUC 0.018")
    box = osm_adapter.AREAS[a.area]
    osm = osm_adapter.load(box)
    t("Tier 3 OSM match + coverage",
      lambda: tier3_osm_verify.match(clean, osm, 25.0, 35.0,
                                     (box[0] + box[2]) / 2.0),
      "warm cache; not in the inference path")

    print()
    gate = t("GATE end to end, with prior", lambda: subprocess.run(
        [sys.executable, "geoshield_verifier.py", "--candidate", a.candidate,
         "--prior", a.clean], capture_output=True))

    total = stages[-1]["seconds"]
    print(f"\n  The gate runs once at map ingest, not per planning cycle.")
    print(f"  {total:.1f} s against a map load Autoware itself takes seconds to do.")

    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
    json.dump({"map": a.candidate, "segments": len(cand), "stages": stages},
              open(a.out, "w"), indent=2)
    print(f"\n  wrote {a.out}")


if __name__ == "__main__":
    main()
