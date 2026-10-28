#!/usr/bin/env bash
# GeoShield reproduction
#
#   bash reproduce.sh detect    files only: no ROS, no Docker, no simulator
#   bash reproduce.sh drive     operator checklist, then trajectory analysis
#
# detect regenerates both attack maps from the clean map, runs detection and
# repair, and asserts every detection and repair figure quoted in README.md.
# It exits non-zero if any of them has moved. The detection pipeline does not
# depend on the simulator, and neither does this half of the script.
#
# Note: the attack maps are deliberately NOT committed (data/ is gitignored).
# Only the clean Nishi-Shinjuku map is an external input.

set -euo pipefail
cd "$(dirname "$0")"

CLEAN="${GEOSHIELD_CLEAN_MAP:-$HOME/autoware_map/nishishinjuku_autoware_map/lanelet2_map.osm}"
MAPROOT="${GEOSHIELD_MAP_ROOT:-$HOME/autoware_map}"
BUILD="${GEOSHIELD_BUILD:-0.50.0-docker}"

# The eight lanelets of the victim route, in route order.
# Start 3012234, goal 3002007, 393.2 m.
TARGETS=3012234,3013054,3013093,3012977,3002017,3013032,3002007,3002013

R=results
D=data

export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1

dv () {  # previous candidate labels report
  python3 differential_verify.py --previous "$1" --candidate "$2" \
    ${3:+--labels "$3"} --report "$4" >/dev/null 2>&1 || true
}

assert_dv () {  # report labels want_flagged want_tp want_fp name
  python3 - "$@" <<'PY'
import json, sys
rep, labels, e_fl, e_tp, e_fp, name = sys.argv[1:7]
d = json.load(open(rep))
flagged = {c["segment_id"] for c in d.get("changes", [])
           if c.get("verdict") in ("REJECT", "SUSPECT")}
truth = set(json.load(open(labels))) if labels != "-" else set()
tp, fp = len(flagged & truth), len(flagged - truth)
rec = tp / len(truth) if truth else float("nan")
prec = tp / len(flagged) if flagged else float("nan")
print(f"  {name:<28} verdict={d.get('verdict'):<7} flagged={len(flagged):<3} "
      f"TP={tp} FP={fp} recall={rec:.3f} precision={prec:.3f}")
bad = [m for m, got, want in
       (("flagged", len(flagged), int(e_fl)), ("TP", tp, int(e_tp)),
        ("FP", fp, int(e_fp))) if got != want]
if bad:
    sys.exit(f"  MISMATCH in {name}: " + ", ".join(bad) + " differ from README")
PY
}

assert_same_geometry () {  # restored.osm prior.osm name
  python3 - "$@" <<'PY'
import sys, xml.etree.ElementTree as ET
a, b, name = sys.argv[1:4]
def nodes(p):
    out = {}
    for n in ET.parse(p).getroot().findall("node"):
        t = {x.get("k"): x.get("v") for x in n.findall("tag")}
        out[n.get("id")] = (n.get("lat"), n.get("lon"),
                            t.get("local_x"), t.get("local_y"))
    return out
def close(p, q, tol):
    if p is None or q is None:
        return p == q
    try:
        return abs(float(p) - float(q)) <= tol
    except ValueError:
        return p == q
A, B = nodes(a), nodes(b)
only = set(A) ^ set(B)
diff = [i for i in set(A) & set(B)
        if not (close(A[i][0], B[i][0], 1e-8) and close(A[i][1], B[i][1], 1e-8)
                and close(A[i][2], B[i][2], 1e-3) and close(A[i][3], B[i][3], 1e-3))]
print(f"  {name:<28} {len(set(A) & set(B))} shared nodes, "
      f"{len(diff)} differing, {len(only)} in one map only")
if diff or only:
    sys.exit(f"  MISMATCH in {name}: repaired map is not the prior")
PY
}

make_map_dir () {  # osm dirname
  local d="$MAPROOT/$2"
  mkdir -p "$d"
  cp "$1" "$d/lanelet2_map.osm"
  ln -sf ../nishishinjuku_autoware_map/pointcloud_map.pcd "$d/pointcloud_map.pcd"
  # No map_projector_info.yaml on purpose: Autoware then defaults to MGRS and
  # projects from lat/lon, which is the behaviour the attack writers must match.
}

cmd_detect () {
  [ -f "$CLEAN" ] || { echo "clean map not found: $CLEAN" >&2; exit 1; }
  mkdir -p "$D" "$R" tests

  echo "== regenerating attacks from $CLEAN"
  python3 attack_injector.py --lanelet2 "$CLEAN" --attack width_ramp \
    --target "$TARGETS" --total-gain 3.0 \
    --out "$D/repro_width" --out-map "$D/repro_width.osm" >/dev/null
  python3 centerline_attack.py --map "$CLEAN" --target "$TARGETS" \
    --max-shift 1.0 --out "$D/repro_centerline.osm" \
    --labels "$D/repro_centerline_labels.json" >/dev/null

  # Compare against the archived maps that produced the recorded bags, if
  # present. A matching hash is a bonus; geometric equality is the criterion.
  # lat/lon is written to 12 decimals, four orders of magnitude finer than the
  # 0.1 mm of local_x/local_y, so the last digit carries no physical meaning.
  for pair in "repro_width.osm route_g3.0c.osm width-ramp" \
              "repro_centerline.osm route_cl1.0.osm centreline"; do
    set -- $pair
    [ -f "$D/$2" ] || continue
    if [ "$(md5sum < "$D/$1")" = "$(md5sum < "$D/$2")" ]; then
      echo "  $3: byte-identical to archived $2"
    else
      echo "  $3: hash differs from archived $2, comparing geometry instead"
      assert_same_geometry "$D/$1" "$D/$2" "$3 vs archive"
    fi
  done

  echo "== detection"
  dv "$CLEAN" "$CLEAN"                 ""                              "$R/repro_dv_clean.json"
  dv "$CLEAN" "$D/repro_width.osm"      "$D/repro_width_labels.json"      "$R/repro_dv_width.json"
  dv "$CLEAN" "$D/repro_centerline.osm" "$D/repro_centerline_labels.json" "$R/repro_dv_centerline.json"
  assert_dv "$R/repro_dv_clean.json"      -                                 0  0 0 "clean control"
  assert_dv "$R/repro_dv_width.json"      "$D/repro_width_labels.json"      17 8 9 "width ramp"
  assert_dv "$R/repro_dv_centerline.json" "$D/repro_centerline_labels.json" 8  8 0 "centreline injection"

  echo "== repair"
  python3 repair_differential.py --candidate "$D/repro_width.osm" --prior "$CLEAN" \
    --diff "$R/repro_dv_width.json" --include-suspect \
    --out "$D/repro_width_restored.osm" --report "$R/repro_repair_width.json"
  python3 repair_differential.py --candidate "$D/repro_centerline.osm" --prior "$CLEAN" \
    --diff "$R/repro_dv_centerline.json" --include-suspect \
    --out "$D/repro_centerline_restored.osm" --report "$R/repro_repair_centerline.json"
  python3 repair_differential.py --candidate "$CLEAN" --prior "$CLEAN" \
    --diff "$R/repro_dv_clean.json" --include-suspect \
    --out "$D/repro_clean_restored.osm" --report "$R/repro_repair_clean.json"

  echo "== repaired maps against the prior"
  assert_same_geometry "$D/repro_width_restored.osm"      "$CLEAN" "width ramp repaired"
  assert_same_geometry "$D/repro_centerline_restored.osm" "$CLEAN" "centreline repaired"
  assert_same_geometry "$D/repro_clean_restored.osm"      "$CLEAN" "clean control repaired"

  echo "== re-verifying the repaired maps"
  dv "$CLEAN" "$D/repro_width_restored.osm"      "" "$R/repro_dv_width_restored.json"
  dv "$CLEAN" "$D/repro_centerline_restored.osm" "" "$R/repro_dv_centerline_restored.json"
  assert_dv "$R/repro_dv_width_restored.json"      - 0 0 0 "width ramp re-verified"
  assert_dv "$R/repro_dv_centerline_restored.json" - 0 0 0 "centreline re-verified"

  echo "== building map directories for the simulator"
  make_map_dir "$D/repro_width.osm"                rb_width
  make_map_dir "$D/repro_width_restored.osm"       rb_width_shield
  make_map_dir "$D/repro_centerline.osm"           rb_centerline
  make_map_dir "$D/repro_centerline_restored.osm"  rb_centerline_shield
  make_map_dir "$D/repro_clean_restored.osm"       rb_clean_shield
  echo "  under $MAPROOT: rb_width rb_width_shield rb_centerline rb_centerline_shield rb_clean_shield"

  echo
  echo "detect: all detection and repair figures in README.md reproduced."
}

cmd_drive () {
  cat <<'TXT'
== Autoware runs

Autoware permits one recording per launch and the map argument changes between
conditions, so this is a checklist, not a pipeline. Seven launches.

Host, once:
  xhost +local:docker
  ros2 daemon stop

Each run, in the container (docker exec -it autoware bash):
  cp /geoshield/config/autoware_overrides/system.yaml \
     /opt/autoware/share/autoware_launch/config/system/diagnostics/system.yaml
  ros2 launch autoware_launch planning_simulator.launch.xml \
     map_path:=/autoware_map/<MAP> \
     vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit

Then, in a second container shell:
  cd /geoshield && python3 scenario_runner.py \
     --scenario data/scenario_clean.json --duration 150 --bag bags/<BAG>

  MAP                        BAG
  nishishinjuku_autoware_map rb_clean
  nishishinjuku_autoware_map rb_clean_repeat
  rb_clean_shield            rb_clean_shield
  rb_width                   rb_width
  rb_width_shield            rb_width_shield
  rb_centerline              rb_centerline
  rb_centerline_shield       rb_centerline_shield

Rules learned the hard way:
  - Relaunch between every recording. A stale stack leaves
    duplicated_node_checker STALE, which blocks autonomous mode while the
    transition manager still reports is_all_ok: true.
  - Never run ros2 from the host; it lacks autoware_adapi_v1_msgs.
  - Type nothing in any terminal during a recording.
  - scenario_runner.py records to <bag>.pending and promotes only on passing
    its own start-pose and distance checks. A REJECTED run is not a result.

When all seven bags exist, re-run: bash reproduce.sh drive
TXT
  local miss=0
  for b in rb_clean rb_clean_repeat rb_clean_shield rb_width rb_width_shield \
           rb_centerline rb_centerline_shield; do
    [ -d "bags/$b" ] || { echo "  missing bags/$b"; miss=1; }
  done
  [ "$miss" = 0 ] || { echo; echo "drive: bags incomplete, stopping before analysis."; return 0; }

  echo
  echo "== trajectory analysis (build $BUILD)"
  for pair in "rb_clean_repeat noise" "rb_width width" "rb_width_shield width_shield" \
              "rb_centerline centerline" "rb_centerline_shield centerline_shield" \
              "rb_clean_shield clean_shield"; do
    set -- $pair
    python3 frechet_analysis.py --clean bags/rb_clean --tampered "bags/$1" \
      --build "$BUILD" --out "$R/repro_fr_$2.json"
  done
}

case "${1:-}" in
  detect) cmd_detect ;;
  drive)  cmd_drive ;;
  *) echo "usage: bash reproduce.sh {detect|drive}" >&2; exit 2 ;;
esac
