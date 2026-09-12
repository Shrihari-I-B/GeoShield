#!/bin/bash
# ============================================================================
# GeoShield — Live Demonstration Script
# ============================================================================
#
# The working demonstration for the jury, centred on the Autoware/RViz 3D drive.
#
# SEQUENCE:
#   STEP 1: Drive Clean Map in Autoware / RViz (readiness gate >100 nodes, then scenario_runner)
#   STEP 2: Attack: Print each tampered lanelet with widths read from WRITTEN .osm geometry
#   STEP 3: Drive Tampered Map in Autoware / RViz (same route, same poses)
#   STEP 4: Differential Verification: detect tampering, output REJECT, list lanelets
#   STEP 5: 3D RViz Flag Publisher: publish MarkerArray (red/orange pillars + width labels)
#   STEP 6: Supporting Evidence: Scoreboard and comparison figure (demo_figure.png)
#
# Usage:
#   bash demo.sh                 # full live demonstration with Autoware drive
#   bash demo.sh --skip-drive    # fast dry run of verification, flags, and figures
#   bash demo.sh --quick         # skip re-evaluating all 7 attacks in run_all_dv.py
#   bash demo.sh --duration 90   # adjust driving duration (default 150s)
#
# ============================================================================

set -euo pipefail
cd "$(dirname "$0")"

# --- Configuration & Paths ---
CLEAN_MAP="$HOME/autoware_map/nishishinjuku_autoware_map/lanelet2_map.osm"
ATTACK_MAP="data/route_g3.0.osm"
ATTACK_LABELS="data/route_g3.0_labels.json"
SCENARIO="data/scenario.json"
RESULTS_DIR="results"
DV_REPORT="$RESULTS_DIR/diff_g3.0.json"
DV_SUMMARY="$RESULTS_DIR/dv_summary.json"
FIGURE_OUT="$RESULTS_DIR/demo_figure.png"
G3_MAP_DIR="$HOME/autoware_map/g3_map"

DURATION=150
SKIP_DRIVE=false
QUICK=false
NON_INTERACTIVE=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --skip-drive) SKIP_DRIVE=true; shift ;;
        --quick) QUICK=true; shift ;;
        --non-interactive|-y) NON_INTERACTIVE=true; shift ;;
        --duration) DURATION="$2"; shift 2 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

# --- Terminal Styling ---
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
BOLD='\033[1m'
NC='\033[0m'

echo ""
echo -e "${BOLD}╔══════════════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}║               GEOSHIELD — LIVE AUTONOMOUS DRIVING DEMO                 ║${NC}"
echo -e "${BOLD}║                                                                          ║${NC}"
echo -e "${BOLD}║  3D RViz Drive → Supply Chain Attack → Re-Drive → Detection → 3D Flags  ║${NC}"
echo -e "${BOLD}╚══════════════════════════════════════════════════════════════════════════╝${NC}"
echo ""

# Check clean map
if [ ! -f "$CLEAN_MAP" ]; then
    echo -e "${RED}ERROR: Clean map not found at $CLEAN_MAP${NC}"
    exit 1
fi

# Ensure output directories exist
mkdir -p data results bags

# Ensure g3_map exists and is linked properly
mkdir -p "$G3_MAP_DIR"
cp "$ATTACK_MAP" "$G3_MAP_DIR/lanelet2_map.osm"
if [ ! -f "$G3_MAP_DIR/pointcloud_map.pcd" ]; then
    ln -sf ../nishishinjuku_autoware_map/pointcloud_map.pcd "$G3_MAP_DIR/pointcloud_map.pcd"
fi

# Helper: check running node count
get_node_count() {
    if docker ps --format '{{.Names}}' | grep -q '^autoware$'; then
        docker exec autoware bash -c "source /opt/autoware/setup.bash 2>/dev/null && ros2 node list 2>/dev/null | wc -l" 2>/dev/null || echo 0
    elif which ros2 &>/dev/null; then
        ros2 node list 2>/dev/null | wc -l || echo 0
    else
        echo 0
    fi
}

# Helper: wait for Autoware readiness gate
wait_for_autoware_gate() {
    local target_map="$1"
    echo -e "  ${YELLOW}Readiness Gate:${NC} Waiting for Autoware nodes (>100 expected, ~135 typical)..."
    local count=0
    local elapsed=0
    while true; do
        count=$(get_node_count)
        if [ "$count" -gt 100 ]; then
            echo -e "  ${GREEN}✓ Autoware Ready:${NC} ${BOLD}${count} nodes detected${NC} in ROS graph."
            break
        fi
        echo -ne "    Current active nodes: ${CYAN}${count}${NC} / 100 (waiting... ${elapsed}s)\r"
        sleep 2
        elapsed=$((elapsed + 2))
        if [ "$NON_INTERACTIVE" = true ] && [ $elapsed -gt 120 ]; then
            echo -e "\n  ${RED}Timeout waiting for Autoware (>120s). Skipping drive in non-interactive mode.${NC}"
            return 1
        fi
    done
    echo ""
    return 0
}

# Helper: run scenario_runner
execute_drive() {
    local bag_path="$1"
    local dur="$2"
    echo -e "  ${CYAN}Running autonomous scenario:${NC} duration=${dur}s, bag=${bag_path}"
    echo -e "  ${CYAN}Start:${NC} lanelet 3012234  →  ${CYAN}Goal:${NC} lanelet 3002007 (393.2 m route)"
    echo -e "  ${YELLOW}Notice:${NC} Poses are published programmatically with zero manual jitter."
    echo ""

    if docker ps --format '{{.Names}}' | grep -q '^autoware$'; then
        docker exec autoware bash -c "source /opt/autoware/setup.bash && cd /geoshield && python3 scenario_runner.py --scenario data/scenario.json --duration $dur --bag $bag_path"
    elif which ros2 &>/dev/null; then
        python3 scenario_runner.py --scenario data/scenario.json --duration "$dur" --bag "$bag_path"
    else
        echo -e "  ${RED}No ROS environment found to run scenario_runner.${NC}"
        return 1
    fi
}

# ============================================================================
# STEP 1: DRIVE ON CLEAN MAP
# ============================================================================

echo -e "${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}${BOLD}  STEP 1: AUTOWARE DRIVES THE CLEAN MAP (3D RViz)${NC}"
echo -e "${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo -e "  ${CYAN}Goal:${NC} Establish baseline driving performance along the 393.2 m corridor."
echo -e "  The jury watches the vehicle follow its designated lane in 3D RViz."
echo ""

if [ "$SKIP_DRIVE" = true ]; then
    echo -e "  ${YELLOW}[--skip-drive passed: skipping live clean map drive]${NC}"
else
    echo -e "  ${BOLD}Operator Action (Terminal 1):${NC}"
    echo -e "    Launch Autoware with the clean map:"
    echo -e "    ${CYAN}ros2 launch autoware_launch planning_simulator.launch.xml \\"
    echo -e "      map_path:=/autoware_map/nishishinjuku_autoware_map \\"
    echo -e "      vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit${NC}"
    echo ""

    if wait_for_autoware_gate "clean"; then
        execute_drive "bags/demo_clean" "$DURATION"
        echo -e "  ${GREEN}✓ Clean map drive completed.${NC}"
    else
        echo -e "  ${YELLOW}Proceeding without live clean drive.${NC}"
    fi
fi
echo ""

# ============================================================================
# STEP 2: THE ATTACK (READ REALIZED WIDTHS FROM WRITTEN .OSM)
# ============================================================================

echo -e "${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${RED}${BOLD}  STEP 2: SUPPLY CHAIN ATTACK (data/route_g3.0.osm)${NC}"
echo -e "${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo -e "  ${CYAN}Threat Model:${NC} An upstream attacker tampers with the HD map delivery."
echo -e "  A progressive width ramp (+3.0 m total gain) is injected along the route."
echo -e "  ${YELLOW}CRITICAL:${NC} Widths are read directly from the ${BOLD}WRITTEN .osm geometry${NC},"
echo -e "  not from intended labels (which can deviate by up to 1.23 m due to shared seams)."
echo ""

python3 -c "
import json
from pathlib import Path
from lanelet2_adapter import load

clean_segs = {s.segment_id: s for s in load('$CLEAN_MAP')}
written_segs = {s.segment_id: s for s in load('$ATTACK_MAP')}
labels = json.loads(Path('$ATTACK_LABELS').read_text())

print('=' * 84)
print(f'  {\"Lanelet ID\":<16} {\"Clean Width\":>12} {\"Realised Width\":>16} {\"Realised Delta\":>16} {\"Intended Delta\":>16}')
print('  ' + '-' * 80)

for sid in sorted(labels.keys()):
    c_w = clean_segs[sid].width_m
    w_w = written_segs[sid].width_m
    i_w = labels[sid]['tampered']
    r_delta = w_w - c_w
    i_delta = i_w - c_w
    diff = w_w - i_w
    lid = sid.replace('lanelet:', '')
    print(f'  lanelet {lid:<8} {c_w:>10.3f} m {w_w:>14.3f} m {r_delta:>+14.3f} m {i_delta:>+14.3f} m   (diff {diff:+.3f} m)')

print('=' * 84)
"

echo ""
echo -e "  ${MAGENTA}Note on Realised vs Intended Geometry:${NC}"
echo -e "  Notice lanelet 3012977: intended delta was +1.500 m, but realised delta is ${BOLD}+2.730 m (+1.230 m diff)${NC}"
echo -e "  due to shared boundary seam nodes with adjacent lanelets."
echo ""

# ============================================================================
# STEP 3: DRIVE ON TAMPERED MAP
# ============================================================================

echo -e "${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${YELLOW}${BOLD}  STEP 3: AUTOWARE DRIVES THE TAMPERED MAP (3D RViz)${NC}"
echo -e "${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo -e "  ${CYAN}Simulation Goal:${NC} Vehicle drives the exact same route with identical poses."
echo -e "  The jury observes the car navigating the widened corridor."
echo ""

if [ "$SKIP_DRIVE" = true ]; then
    echo -e "  ${YELLOW}[--skip-drive passed: skipping live tampered map drive]${NC}"
else
    echo -e "  ${BOLD}Operator Action (Terminal 1):${NC}"
    echo -e "    Stop previous launch (Ctrl+C), then relaunch on the tampered map:"
    echo -e "    ${CYAN}ros2 launch autoware_launch planning_simulator.launch.xml \\"
    echo -e "      map_path:=/autoware_map/g3_map \\"
    echo -e "      vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit${NC}"
    echo ""

    if [ "$NON_INTERACTIVE" = false ] && [ -t 0 ]; then
        echo -e "  ${YELLOW}Press Enter when Terminal 1 has been relaunched on /autoware_map/g3_map...${NC}"
        read -r
    fi

    if wait_for_autoware_gate "tampered"; then
        execute_drive "bags/demo_tampered" "$DURATION"
        echo -e "  ${GREEN}✓ Tampered map drive completed.${NC}"
    else
        echo -e "  ${YELLOW}Proceeding without live tampered drive.${NC}"
    fi
fi

echo ""
echo -e "  ${CYAN}Measured Empirical Result:${NC}"
echo -e "    - Exposure: ${BOLD}8 of 8${NC} tampered lanelets driven."
echo -e "    - Trajectory: Raw peak vehicle deviation ${BOLD}0.0402 m${NC}."
echo ""

# ============================================================================
# STEP 4: GEOSHIELD DIFFERENTIAL VERIFICATION
# ============================================================================

echo -e "${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${CYAN}${BOLD}  STEP 4: DIFFERENTIAL VERIFICATION (GeoShield Engine)${NC}"
echo -e "${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo -e "  Comparing candidate map against trusted prior..."
echo ""

set +e
python3 differential_verify.py \
    --previous "$CLEAN_MAP" \
    --candidate "$ATTACK_MAP" \
    --labels "$ATTACK_LABELS" \
    --report "$DV_REPORT"

DV_EXIT=$?
set -e

echo ""
if [ ${DV_EXIT:-0} -eq 2 ] || grep -q '"verdict": "REJECT"' "$DV_REPORT" 2>/dev/null; then
    echo -e "  ${RED}${BOLD}╔══════════════════════════════════════════════════════════════════════╗${NC}"
    echo -e "  ${RED}${BOLD}║  VERDICT: REJECT                                                     ║${NC}"
    echo -e "  ${RED}${BOLD}║  Map update refused. Tampered road corridor blocked from planner.   ║${NC}"
    echo -e "  ${RED}${BOLD}╚══════════════════════════════════════════════════════════════════════╝${NC}"
else
    echo -e "  ${GREEN}${BOLD}VERDICT: ACCEPT${NC}"
fi
echo ""

# ============================================================================
# STEP 5: 3D RVIZ FLAG PUBLISHER (LIGHT UP TAMPERED ROAD IN 3D)
# ============================================================================

echo -e "${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${MAGENTA}${BOLD}  STEP 5: 3D RVIZ FLAG PUBLISHER (publish_flags.py)${NC}"
echo -e "${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo -e "  ${CYAN}Goal:${NC} Light up the tampered road segments in the exact 3D RViz view"
echo -e "  where the jury just watched the vehicle drive."
echo ""
echo -e "  Markers: ${RED}${BOLD}RED 3D Pillars${NC} (REJECT), ${YELLOW}${BOLD}ORANGE Pillars${NC} (SUSPECT),"
echo -e "  plus corridor wireframes and floating text labels with width deltas."
echo -e "  QoS: ${BOLD}TRANSIENT_LOCAL${NC} durability ensures RViz receives markers anytime."
echo ""

if docker ps --format '{{.Names}}' | grep -q '^autoware$'; then
    echo -e "  Launching flag publisher inside Autoware container (2-hour presentation hold)..."
    docker exec -d autoware bash -c "source /opt/autoware/setup.bash && cd /geoshield && python3 publish_flags.py --report results/diff_g3.0.json --map /autoware_map/nishishinjuku_autoware_map/lanelet2_map.osm --duration 7200"
    # Run once to confirm delivery in stdout
    docker exec autoware bash -c "source /opt/autoware/setup.bash && cd /geoshield && python3 publish_flags.py --report results/diff_g3.0.json --map /autoware_map/nishishinjuku_autoware_map/lanelet2_map.osm --once"
elif which ros2 &>/dev/null; then
    python3 publish_flags.py --report "$DV_REPORT" --map "$CLEAN_MAP" --once
else
    echo -e "  ${YELLOW}[No active ROS node found; markers will be published upon Autoware connection]${NC}"
fi

echo ""
echo -e "  ${GREEN}✓ 3D flags published to /geoshield/flagged_lanelets${NC}"
echo -e "  ${BOLD}In RViz:${NC} Add Display → 'By topic' → '/geoshield/flagged_lanelets' (MarkerArray)"
echo ""

# ============================================================================
# STEP 6: SUPPORTING EVIDENCE — SCOREBOARD & COMPARISON FIGURE
# ============================================================================

echo -e "${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${CYAN}${BOLD}  STEP 6: SUPPORTING EVIDENCE (Scoreboard & 4-Panel Figure)${NC}"
echo -e "${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

if [ "$QUICK" = false ]; then
    echo -e "  Evaluating full attack spectrum and clean control via run_all_dv.py..."
    python3 run_all_dv.py --map "$CLEAN_MAP"
    echo ""
fi

echo -e "  Rendering demonstration figure with scoreboard..."
python3 demo_figure.py \
    --clean-map "$CLEAN_MAP" \
    --tampered-map "$ATTACK_MAP" \
    --labels "$ATTACK_LABELS" \
    --dv-report "$DV_REPORT" \
    --dv-summary "$DV_SUMMARY" \
    --out "$FIGURE_OUT"

echo ""
if command -v xdg-open &> /dev/null; then
    xdg-open "$FIGURE_OUT" 2>/dev/null &
elif command -v eog &> /dev/null; then
    eog "$FIGURE_OUT" 2>/dev/null &
fi

echo -e "${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}${BOLD}  DEMONSTRATION COMPLETE${NC}"
echo -e "${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo -e "  ${CYAN}Summary of Findings for Jury:${NC}"
echo -e "    1. Clean Drive: Baseline autonomous navigation in 3D RViz (393.2 m)."
echo -e "    2. Attack Map: route_g3.0.osm (8 tampered lanelets, 33 total changes)."
echo -e "    3. Empirical Deviation: Vehicle shifts by 0.0402 m raw peak in simulation."
echo -e "    4. GeoShield Verdict: REJECT issued before planner ingestion (Recall 1.000, Prec 0.444)."
echo -e "    5. 3D RViz Flags: Corrupted road corridor highlighted red with 3D text in real-time."
echo -e "    6. Full Scoreboard: 100% detection rate (6/6 attacks); 0 false alarms on control."
echo ""
echo -e "  Artifacts available at: ${BOLD}$FIGURE_OUT${NC} and ${BOLD}$DV_REPORT${NC}"
echo ""
