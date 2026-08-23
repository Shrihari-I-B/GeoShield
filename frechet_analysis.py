#!/usr/bin/env python3
"""
GeoShield -- Phase 6b: trajectory deviation analysis.

Runs on the LAPTOP, host-side, with no ROS install and no container:

    pip install mcap mcap-ros2-support --break-system-packages   # once
    cd ~/Development/projects/geoshield
    python3 frechet_analysis.py \
        --clean    data/bags/clean_run_good \
        --tampered data/bags/route_g3.0 \
        --build 0.52.0-ec2 \
        --out results/results_g3.0_truncated.json

--build labels the Autoware version that RECORDED the bags, not the one
analysing them. Analysis is build-independent; recording is not, and figures
from 0.50.0 and 0.52.0 are not comparable.

WHAT IT MEASURES
----------------
Two numbers, matching Sato et al. (VehicleSec 2025) so the results are
directly comparable to their published figures:

    d_Fp   planning deviation  -- /planning/trajectory
    d_Fe   driven deviation    -- /localization/kinematic_state

Their reported d_Fe for lane width expansion on this same Nishi-Shinjuku map:

    w_l = 3.5 m  ->  0.6049 m
    w_l = 4.0 m  ->  0.8419 m
    w_l = 4.5 m  ->  1.0965 m
    w_l = 5.0 m  ->  route not completed

with a safety threshold of th = 0.5 m, derived from a 3.0 m lane and a
1.895 m vehicle (Lexus RX450h).

WHY FRECHET AND NOT POINTWISE DISTANCE
--------------------------------------
Imagine walking one path while a dog walks the other, both moving forward
only, never backwards. The Frechet distance is the shortest leash that
permits it. Unlike averaging pointwise distances, it respects ORDER: two
trajectories can pass through the same region in very different sequences,
and Frechet penalises that where a naive comparison would not. It is also
insensitive to differing sample counts, which matters here because the two
runs recorded 4,552 and 4,581 messages.

Implemented as the standard dynamic-programming recursion over the coupling
measure, with the trajectories resampled to bound cost at O(n*m).

THE ENDPOINT ARTEFACT -- WHY truncate_common() EXISTS
-----------------------------------------------------
MEASURED, and it invalidated a published-in-our-own-report number.

This script originally reported d_Fe = 1.414 m for the 3.0 m width ramp
(clean_run_good vs route_g3.0). That figure was an artefact, not a
deviation. Both runs recorded for a fixed 90 s wall-clock window; the
tampered run was 1.414 m further along the route when recording stopped.
Beyond the clean path's final sample there is nothing left to match against,
so every trailing tampered sample couples to that same final clean point and
the offset grows monotonically to exactly the endpoint gap.

The evidence that identified it:

    last 10 lateral offsets : 0.474 -> 1.414  (monotonic, no plateau)
    max excluding last 20   : 0.051
    endpoint gap            : 1.414            <- identical to reported d_Fe

Cutting both paths to the shorter one's ARC LENGTH removes it. The corrected
value for the same two bags is 0.051 m. compare_runs.py has carried this fix
since it was found; this script did not, so every results_*.json produced by
it before this revision carries the artefact and must be re-derived.

--no-truncate reproduces the old behaviour deliberately, so the artefact can
be demonstrated rather than merely asserted.

d_Fp IS NOT INDEPENDENTLY VERIFIED
----------------------------------
planned_path() takes only the LAST Trajectory message of each bag. Two runs
that stopped at different points along the route yield two "last plans"
beginning at different ego positions -- an analogous artefact by a different
mechanism, which arc-length truncation does not fully remove because the
plans do not share a common origin to begin with. compare_runs.py never
computes d_Fp, so the previously reported 1.998 m has never been checked
against a second implementation.

plan_origin_gap_m is emitted as a diagnostic: it is the distance between the
two plans' first points. A large value means the two plans start from
different places and d_Fp is measuring that offset as much as any tampering
effect. Treat d_Fp as UNVERIFIED until this is resolved.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path


# ----------------------------------------------------------------------
# bag reading
# ----------------------------------------------------------------------

def _mcap_files(path: str) -> list:
    """All .mcap files in a bag directory, sorted. A bag may be split across
    several; compare_runs.py takes only the first, which is equivalent for
    single-file bags and wrong for split ones."""
    d = Path(path)
    if d.is_file() and d.suffix == ".mcap":
        return [d]
    return sorted(d.glob("*.mcap"))


def read_bag_mcap(path: str, topics: list[str]) -> dict[str, list]:
    """
    Read via the mcap libraries, decoding message definitions EMBEDDED in the
    file. Needs no ROS install and no Autoware message packages.

    WHY THIS IS THE DEFAULT. rosbag2_py resolves types from the environment
    and parses metadata.yaml with yaml-cpp. Bags recorded on Autoware 0.52.0
    carry `version: 9` metadata whose QoS block writes `history: unknown`;
    the 0.50.0 reader cannot convert it and aborts before reaching any
    message:

        RuntimeError: Exception on parsing info file:
        yaml-cpp: error at line 15, column 11: bad conversion

    The .mcap payload is fine -- only the sidecar metadata is unreadable.
    Decoding from embedded schemas sidesteps metadata.yaml entirely, so bags
    remain readable across Autoware versions and on machines with no ROS at
    all. This is the same property that made Foxglove work where RViz could
    not (handoff decision 24), applied to analysis rather than display.

    It also explains a discrepancy in the record: compare_runs.py has always
    read .mcap directly and ran fine on the laptop, while this script was
    tied to the machine that recorded the bags. The two were never portable
    on equal terms.
    """
    try:
        from mcap.reader import make_reader
        from mcap_ros2.decoder import DecoderFactory
    except ImportError:
        sys.exit("mcap libraries not found. Run:\n"
                 "  pip install mcap mcap-ros2-support --break-system-packages")

    files = _mcap_files(path)
    if not files:
        sys.exit(f"no .mcap inside {path}")

    out: dict[str, list] = {t: [] for t in topics}
    for f in files:
        with open(f, "rb") as fh:
            reader = make_reader(fh, decoder_factories=[DecoderFactory()])
            for _, channel, message, decoded in reader.iter_decoded_messages(
                    topics=topics):
                out[channel.topic].append((message.log_time, decoded))
    for t in topics:
        out[t].sort(key=lambda pair: pair[0])
    return out


def read_bag_rosbag2(path: str, topics: list[str]) -> dict[str, list]:
    """Read via rosbag2_py. Requires a sourced Autoware workspace, and fails
    on bags whose metadata.yaml is newer than the local rosbag2. Retained as
    a fallback for bags with no .mcap (e.g. sqlite3 storage)."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        sys.exit(f"ROS 2 python packages not found ({exc}).\n"
                 "Source an Autoware setup, or use --backend mcap.")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id=""),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    out = {t: [] for t in topics}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic not in out:
            continue
        msg_cls = get_message(types[topic])
        out[topic].append((stamp, deserialize_message(data, msg_cls)))
    return out


def read_bag(path: str, topics: list[str], backend: str = "auto") -> dict[str, list]:
    """Dispatch to a backend. 'auto' prefers mcap whenever a .mcap exists."""
    if backend == "mcap":
        return read_bag_mcap(path, topics)
    if backend == "rosbag2":
        return read_bag_rosbag2(path, topics)
    return (read_bag_mcap(path, topics) if _mcap_files(path)
            else read_bag_rosbag2(path, topics))


def which_backend(path: str, backend: str = "auto") -> str:
    if backend != "auto":
        return backend
    return "mcap" if _mcap_files(path) else "rosbag2"


def ego_path(msgs) -> list[tuple[float, float]]:
    """Driven path from Odometry, de-duplicated -- the ego is stationary for
    a while at the start of every run and those repeats add no information."""
    pts = []
    for _, m in msgs:
        p = m.pose.pose.position
        if not pts or math.dist(pts[-1], (p.x, p.y)) > 0.05:
            pts.append((p.x, p.y))
    return pts


def planned_path(msgs) -> list[tuple[float, float]]:
    """
    Planned path from the LAST Trajectory message.

    The planner republishes at ~10 Hz, each message a fresh plan from the
    ego's current position. Taking the final one gives a complete plan rather
    than a concatenation of partially-overlapping ones.

    CAVEAT -- see the module docstring. Two runs that stopped at different
    points produce last-plans with different origins. plan_origin_gap_m
    quantifies that; d_Fp is reported UNVERIFIED.
    """
    if not msgs:
        return []
    _, last = msgs[-1]
    return [(p.pose.position.x, p.pose.position.y) for p in last.points]


# ----------------------------------------------------------------------
# arc length and truncation
# ----------------------------------------------------------------------

def arc_lengths(pts) -> list[float]:
    """Cumulative arc length along a polyline. Same implementation as
    compare_runs.py, kept byte-identical so the two agree by construction."""
    out = [0.0]
    for i in range(1, len(pts)):
        out.append(out[-1] + math.dist(pts[i - 1], pts[i]))
    return out


def truncate_common(a, b):
    """
    Cut both paths at the shorter one's arc length.

    Without this the trailing samples of the longer path have nothing to match
    against and the reported deviation collapses to the endpoint gap.
    """
    la, lb = arc_lengths(a), arc_lengths(b)
    limit = min(la[-1], lb[-1])
    ta = [p for p, d in zip(a, la) if d <= limit]
    tb = [p for p, d in zip(b, lb) if d <= limit]
    return ta, tb, limit


# ----------------------------------------------------------------------
# Frechet
# ----------------------------------------------------------------------

def resample(pts: list[tuple[float, float]], n: int) -> list[tuple[float, float]]:
    """n points spaced evenly along arc length."""
    if len(pts) < 2:
        return pts * n
    cum, total = [0.0], 0.0
    for i in range(len(pts) - 1):
        total += math.dist(pts[i], pts[i + 1])
        cum.append(total)
    if total == 0:
        return [pts[0]] * n

    out, j = [], 0
    for i in range(n):
        target = total * i / (n - 1)
        while j < len(cum) - 2 and cum[j + 1] < target:
            j += 1
        span = cum[j + 1] - cum[j]
        t = 0.0 if span == 0 else (target - cum[j]) / span
        out.append((pts[j][0] + t * (pts[j + 1][0] - pts[j][0]),
                    pts[j][1] + t * (pts[j + 1][1] - pts[j][1])))
    return out


def frechet(P: list, Q: list) -> float:
    """
    Discrete Frechet distance, iterative DP (no recursion depth limits).

        ca[i][j] = max( dist(P_i, Q_j),
                        min( ca[i-1][j], ca[i-1][j-1], ca[i][j-1] ) )
    """
    n, m = len(P), len(Q)
    if n == 0 or m == 0:
        return float("nan")

    prev = [0.0] * m
    cur = [0.0] * m
    for i in range(n):
        for j in range(m):
            d = math.dist(P[i], Q[j])
            if i == 0 and j == 0:
                cur[j] = d
            elif i == 0:
                cur[j] = max(cur[j - 1], d)
            elif j == 0:
                cur[j] = max(prev[j], d)
            else:
                cur[j] = max(min(prev[j], prev[j - 1], cur[j - 1]), d)
        prev, cur = cur, prev
    return prev[m - 1]


def max_pointwise(P: list, Q: list) -> float:
    """Max nearest-neighbour distance -- a lateral-deviation proxy."""
    return max(min(math.dist(p, q) for q in Q) for p in P) if P and Q else float("nan")


def tail_offsets(P: list, Q: list, k: int = 10) -> list[float]:
    """
    Nearest-neighbour offset for the last k points of P.

    Diagnostic for the endpoint artefact: if these rise monotonically to a
    value equal to the endpoint gap, the deviation is a recording-window
    artefact rather than a lateral displacement.
    """
    if not P or not Q:
        return []
    return [round(min(math.dist(p, q) for q in Q), 4) for p in P[-k:]]


# ----------------------------------------------------------------------

TRAJ = "/planning/trajectory"
ODOM = "/localization/kinematic_state"


def analyse(clean_bag: str, tampered_bag: str, samples: int = 400,
            truncate: bool = True, build: str = "unspecified",
            backend: str = "auto") -> dict:
    be = which_backend(clean_bag, backend)
    print(f"reading {clean_bag}   [backend: {be}]")
    c = read_bag(clean_bag, [TRAJ, ODOM], backend)
    print(f"reading {tampered_bag}   [backend: "
          f"{which_backend(tampered_bag, backend)}]")
    t = read_bag(tampered_bag, [TRAJ, ODOM], backend)

    res = {
        "clean_bag": clean_bag,
        "tampered_bag": tampered_bag,
        "autoware_build": build,
        "truncated": truncate,
        "backend": be,
        "schema": "frechet_analysis/2",
    }

    # --- driven trajectory, d_Fe
    ce, te = ego_path(c[ODOM]), ego_path(t[ODOM])
    print(f"\ndriven points : clean {len(ce)}   tampered {len(te)}")

    if len(ce) >= 2 and len(te) >= 2:
        cl, tl = arc_lengths(ce)[-1], arc_lengths(te)[-1]
        res["driven_length_clean_m"] = round(cl, 2)
        res["driven_length_tampered_m"] = round(tl, 2)
        res["endpoint_gap_m"] = round(math.dist(ce[-1], te[-1]), 4)
        res["length_difference_m"] = round(abs(cl - tl), 4)

        if truncate:
            ce_u, te_u, limit = truncate_common(ce, te)
            res["common_arc_length_m"] = round(limit, 2)
            print(f"truncated to common arc length: {limit:.2f} m "
                  f"(clean {cl:.2f} m, tampered {tl:.2f} m)")
            if len(ce_u) < 2 or len(te_u) < 2:
                print("  ! truncation left fewer than 2 points -- "
                      "the runs barely overlap; d_Fe is not meaningful")
        else:
            ce_u, te_u = ce, te
            res["common_arc_length_m"] = None
            print("!! --no-truncate: reproducing the ENDPOINT ARTEFACT "
                  "deliberately. Do not report this value.")

        if len(ce_u) >= 2 and len(te_u) >= 2:
            a, b = resample(ce_u, samples), resample(te_u, samples)
            res["d_Fe"] = round(frechet(a, b), 4)
            res["max_lateral_e"] = round(max_pointwise(a, b), 4)
            res["tail_offsets_e"] = tail_offsets(a, b)
        else:
            res["d_Fe"] = None
            res["max_lateral_e"] = None

        # Artefact check, always computed on the UNtruncated paths so the
        # comparison is available even in the normal (truncated) mode.
        a_raw, b_raw = resample(ce, samples), resample(te, samples)
        res["d_Fe_untruncated"] = round(frechet(a_raw, b_raw), 4)
        if res.get("d_Fe") is not None:
            res["artefact_inflation_m"] = round(
                res["d_Fe_untruncated"] - res["d_Fe"], 4)
    else:
        res["d_Fe"] = None
        print("  ! not enough driven points -- did the vehicle move?")

    # --- planned trajectory, d_Fp  (UNVERIFIED, see module docstring)
    cp, tp = planned_path(c[TRAJ]), planned_path(t[TRAJ])
    print(f"planned points: clean {len(cp)}   tampered {len(tp)}")

    if len(cp) >= 2 and len(tp) >= 2:
        res["plan_origin_gap_m"] = round(math.dist(cp[0], tp[0]), 4)
        if truncate:
            cp_u, tp_u, plimit = truncate_common(cp, tp)
            res["plan_common_arc_length_m"] = round(plimit, 2)
        else:
            cp_u, tp_u = cp, tp
            res["plan_common_arc_length_m"] = None

        if len(cp_u) >= 2 and len(tp_u) >= 2:
            a, b = resample(cp_u, samples), resample(tp_u, samples)
            res["d_Fp"] = round(frechet(a, b), 4)
            res["max_lateral_p"] = round(max_pointwise(a, b), 4)
        else:
            res["d_Fp"] = None
            res["max_lateral_p"] = None
        res["d_Fp_verified"] = False
    else:
        res["d_Fp"] = None
        res["d_Fp_verified"] = False

    return res


def report(r: dict, threshold: float = 0.5) -> None:
    print("\n" + "=" * 58)
    print("  TRAJECTORY DEVIATION: clean vs tampered")
    print("=" * 58)
    print(f"  Autoware build : {r.get('autoware_build', 'unspecified')}")
    print(f"  bag backend    : {r.get('backend', 'unknown')}")

    if r.get("driven_length_clean_m"):
        print(f"  route driven, clean    : {r['driven_length_clean_m']:>8.2f} m")
        print(f"  route driven, tampered : {r['driven_length_tampered_m']:>8.2f} m")
    if r.get("common_arc_length_m"):
        print(f"  compared over          : {r['common_arc_length_m']:>8.2f} m "
              "of common arc length")

    print()
    for key, label in (("d_Fp", "planning deviation  d_Fp"),
                       ("d_Fe", "driven deviation    d_Fe")):
        v = r.get(key)
        if v is None:
            print(f"  {label} :      n/a")
            continue
        verdict = "OVER threshold" if v > threshold else "within threshold"
        suffix = "   [UNVERIFIED]" if key == "d_Fp" else ""
        print(f"  {label} : {v:>8.4f} m   {verdict}{suffix}")

    print(f"\n  safety threshold th = {threshold} m")
    print("  (3.0 m lane, 1.895 m vehicle -- Sato et al.)")

    # ---- endpoint artefact diagnostic
    raw = r.get("d_Fe_untruncated")
    inflation = r.get("artefact_inflation_m")
    if raw is not None and inflation is not None and inflation > 0.01:
        print("\n  ENDPOINT ARTEFACT REMOVED")
        print(f"    without truncation : {raw:.4f} m")
        print(f"    with truncation    : {r['d_Fe']:.4f} m")
        print(f"    inflation          : {inflation:.4f} m")
        print(f"    endpoint gap       : {r.get('endpoint_gap_m', float('nan')):.4f} m")
        if r.get("tail_offsets_e"):
            print(f"    last offsets       : {r['tail_offsets_e'][0]:.3f}"
                  f" -> {r['tail_offsets_e'][-1]:.3f}")
        print("    If inflation tracks the endpoint gap, the untruncated")
        print("    figure was measuring how much further one run travelled")
        print("    before recording stopped -- not a lateral deviation.")

    # ---- d_Fp caveat
    if r.get("d_Fp") is not None:
        gap = r.get("plan_origin_gap_m")
        print("\n  d_Fp CAVEAT: planned_path() uses only the LAST Trajectory")
        print("  message from each bag. If the two runs stopped at different")
        print("  points, the two plans start from different ego positions.")
        if gap is not None:
            print(f"    plan origin gap : {gap:.4f} m")
            if gap > 0.5:
                print("    ^ larger than the safety threshold. d_Fp is likely")
                print("      dominated by this offset. Do not report it.")
        print("  No second implementation checks d_Fp. Treat as UNVERIFIED.")

    # ---- interpretation
    d = r.get("d_Fe")
    if d is not None:
        print("\n  Sato et al. published d_Fe on this map:")
        print("    w_l 3.5 m -> 0.6049    w_l 4.0 m -> 0.8419")
        print("    w_l 4.5 m -> 1.0965    w_l 5.0 m -> route not completed")

        # WHY THIS BRANCH WAS REWRITTEN. The previous version printed, for
        # d < 0.05, "the tampered lanelets are most likely NOT on this route".
        # After truncation the 3.0 m ramp measures 0.051 m -- one millimetre
        # above that cutoff -- for lanelets we know WERE on the route
        # (lanelet 3002013 widened 3.058 -> 6.639 m, centreline displaced
        # 2.79 m, all eight injected lanelets inside the eight-lanelet route).
        # A small d_Fe does not imply the attack missed. It can equally mean
        # the planner absorbed the change. Distinguish the two by checking
        # overlap directly rather than inferring it from the deviation.
        if d < threshold:
            print(f"\n  Measured {d:.4f} m -- below the {threshold} m threshold.")
            print("  Two different situations produce a small d_Fe. Check which:")
            print("    (a) the tampered lanelets were not on the driven route")
            print("        -> compare injected ids against the route ids in")
            print("           the *_segments.json / scenario json for this run")
            print("    (b) they were on the route, and the planner absorbed the")
            print("        change -- Autoware optimises within the drivable")
            print("        area rather than tracking the geometric centre, so")
            print("        widening a lane need not move the driven path")
            print("  These are opposite conclusions. Do not report either")
            print("  without checking the overlap.")
        else:
            print(f"\n  Measured {d:.4f} m EXCEEDS the {threshold} m threshold:")
            print("  the tampered map moved the vehicle out of safe position.")


def main():
    ap = argparse.ArgumentParser(description="Frechet trajectory analysis")
    ap.add_argument("--clean", required=True)
    ap.add_argument("--tampered", required=True)
    ap.add_argument("--samples", type=int, default=400,
                    help="resample count (Frechet is O(n*m))")
    ap.add_argument("--threshold", type=float, default=0.5)
    ap.add_argument("--build", default="unspecified",
                    help="Autoware build that produced the bags, e.g. "
                         "0.50.0-docker or 0.52.0-ec2. Recorded in the JSON; "
                         "results from different builds are not comparable.")
    ap.add_argument("--backend", choices=("auto", "mcap", "rosbag2"),
                    default="auto",
                    help="bag reader. 'mcap' decodes schemas embedded in the "
                         "file and needs no ROS install; 'rosbag2' needs a "
                         "sourced workspace and fails on bags whose "
                         "metadata.yaml is newer than the local rosbag2. "
                         "'auto' prefers mcap when a .mcap is present.")
    ap.add_argument("--no-truncate", action="store_true",
                    help="skip arc-length truncation and reproduce the "
                         "endpoint artefact deliberately (for demonstration "
                         "only -- never for reported values)")
    ap.add_argument("--out")
    a = ap.parse_args()

    for p in (a.clean, a.tampered):
        if not Path(p).exists():
            sys.exit(f"bag not found: {p}")

    if a.build == "unspecified":
        print("! --build not given. Results from Autoware 0.50.0 (Docker) and")
        print("  0.52.0 (EC2) are not comparable; record which produced this.")

    r = analyse(a.clean, a.tampered, a.samples,
                truncate=not a.no_truncate, build=a.build,
                backend=a.backend)
    report(r, a.threshold)

    if a.out:
        Path(a.out).write_text(json.dumps(r, indent=2))
        print(f"\nwrote {a.out}")


if __name__ == "__main__":
    main()