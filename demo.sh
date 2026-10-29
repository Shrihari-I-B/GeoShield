#!/usr/bin/env bash
# GeoShield demonstration.
#
#   bash demo.sh            regenerate, gate, and present the evidence
#   bash demo.sh --quick    present the evidence only, from existing results
#   bash demo.sh --drive    operator checklist for the live Autoware run
#
# Every figure below is READ FROM results/*.json at run time. The previous
# demo.sh hardcoded 0.0402 m as an attack effect; that number was later shown
# to be the run-to-run noise floor, measured on a map Autoware never saw as
# tampered. A demo that can assert a stale number will eventually assert a
# false one, so this one cannot: if a result file is missing it says so.

set -uo pipefail
cd "$(dirname "$0")"

MODE=full
case "${1:-}" in
  --quick) MODE=quick ;;
  --drive) MODE=drive ;;
  "") ;;
  *) echo "usage: bash demo.sh [--quick|--drive]" >&2; exit 2 ;;
esac

B=$'\033[1m'; C=$'\033[0;36m'; Y=$'\033[1;33m'; R=$'\033[0;31m'; N=$'\033[0m'
rule () { printf '%s\n' "────────────────────────────────────────────────────────────────────"; }
head1 () { echo; rule; printf '%s%s%s\n' "$B" "$1" "$N"; rule; }

if [ "$MODE" = drive ]; then
  bash reproduce.sh drive
  exit 0
fi

head1 "GeoShield — a map integrity gate at the ingest boundary"
cat <<TXT
An HD map arrives from an upstream supplier. GeoShield inspects it before
Autoware's planner ever loads it, refuses what fails, repairs what it can,
and admits the result.

Two claims, both measured, both reproducible from this repository:
  1. Lane width is witnessed by neither OpenStreetMap nor within-map
     self-consistency.
  2. It is therefore recoverable only against a trusted prior.
TXT

if [ "$MODE" = full ]; then
  head1 "1. Regenerate the attacks and verify — no ROS, no simulator"
  bash reproduce.sh detect || { echo "${R}detect failed${N}"; exit 1; }

  head1 "2. The gate's decisions"
  bash reproduce.sh gate || { echo "${R}gate failed${N}"; exit 1; }
fi

head1 "3. The blind spot, measured"
python3 - <<'PY'
import json, os
p = 'results/tier3_coverage.json'
if not os.path.exists(p):
    print("  results/tier3_coverage.json missing — run:")
    print("    python3 tier3_osm_verify.py --map <clean> --out results/tier3_coverage.json")
else:
    d = json.load(open(p))
    print(f"  OSM cross-verification, {d['matched']} of {d['hd_total']} lanelets "
          f"matched geometrically\n")
    print(f"  {'attribute':<20}{'witnessed':>10}{'ceiling':>10}   <- max recall Tier 3 can reach")
    for k, v in d["attrs"].items():
        mark = "   <-- the attack that matters" if k == "width_m" else ""
        print(f"  {k:<20}{v['witnessed']:>10}{v['of_map']:>10.3f}{mark}")
    print("\n  You cannot witness what is not tagged.")
PY

head1 "4. Detection against a trusted prior"
python3 - <<'PY'
import json, os
p = 'results/dv_summary.json'
if not os.path.exists(p):
    print("  results/dv_summary.json missing — run: python3 run_all_dv.py --map <clean>")
else:
    d = json.load(open(p))
    rows = d if isinstance(d, list) else d.get("results", d.get("rows", []))
    print(f"  {'attack':<24}{'N':>4}{'verdict':>9}{'prec':>8}{'recall':>8}{'F1':>8}")
    for r in rows:
        if not isinstance(r, dict):
            continue
        det = r.get("detection", r)
        na = str(r.get("verdict", "-")) == "N/A"
        cells = ("     -   " * 3 if na else
                 f"{det.get('precision',0):>8.3f}{det.get('recall',0):>8.3f}"
                 f"{det.get('f1',0):>8.3f}")
        print(f"  {str(r.get('attack','?')):<24}{r.get('n_tampered','-'):>4}"
              f"{str(r.get('verdict','-')):>9}{cells}")
PY

head1 "5. Safety impact in the Autoware planning simulator"
python3 - <<'PY'
import glob, json, os
fs = sorted(glob.glob('results/fr_*.json'))
if not fs:
    print("  no results/fr_*.json — run: bash demo.sh --drive")
else:
    rows = []
    for f in fs:
        try:
            d = json.load(open(f))
        except Exception:
            continue
        v = d.get("d_Fe")
        if not isinstance(v, (int, float)):
            continue
        rows.append((os.path.basename(f).replace('fr_', '').replace('.json', ''),
                     v, d.get("autoware_build", "?"), bool(d.get("truncated"))))
    rows.sort(key=lambda r: r[1])
    print(f"  {'comparison':<44}{'d_Fe (m)':>10}{'build':>17}")
    for name, v, b, tr in rows:
        print(f"  {name:<44}{v:>10.4f}{b:>17}"
              + ("" if tr else "  (untruncated)"))
    noise = [v for n_, v, _, _ in rows if 'repeat' in n_ or 'shield' in n_]
    atk = [v for n_, v, _, _ in rows
           if 'repeat' not in n_ and 'shield' not in n_]
    if noise and atk:
        print(f"\n  non-attack conditions   max {max(noise):.4f} m")
        print(f"  attack conditions       min {min(atk):.4f} m"
              f"   ({min(atk)/max(noise):.1f}x the worst non-attack run)")
    print("\n  Sorted ascending: the noise band is at the top, the attacks below it.")
PY

head1 "6. What this does NOT do"
cat <<TXT
  - Width tampering is invisible to every no-prior check tested: eight
    approaches, rule-based and learned. The prior is not an optimisation.
  - Without a prior the gate covers one of four attack families. Geometry is
    neither detected nor repaired.
  - Tier 1 cannot examine 215 of 979 lanelets at all — those with fewer than
    two connected neighbours. That is a stated evasion.
  - Measured on one map, Nishi-Shinjuku. Generality is untested by decision.

  Live Autoware run:   bash demo.sh --drive
  Full reproduction:   bash reproduce.sh detect && bash reproduce.sh gate
TXT
echo
