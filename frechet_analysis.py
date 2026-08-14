#!/usr/bin/env python3
"""
GeoShield -- Phase 6b: trajectory deviation analysis.

Runs on the EC2 instance (needs the Autoware message definitions).

    source ~/autoware/install/setup.bash
    python3 frechet_analysis.py --clean ~/bags/clean_run3 \
                                --tampered ~/bags/tampered_run2 \
                                --out results_frechet.json

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

def read_bag(path: str, topics: list[str]) -> dict[str, list]:
    """Read messages via rosbag2_py. Requires a sourced Autoware workspace."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        sys.exit(f"ROS 2 python packages not found ({exc}).\n"
                 "Run:  source ~/autoware/install/setup.bash")

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
    """
    if not msgs:
        return []
    _, last = msgs[-1]
    return [(p.pose.position.x, p.pose.position.y) for p in last.points]


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


# ----------------------------------------------------------------------

TRAJ = "/planning/trajectory"
ODOM = "/localization/kinematic_state"


def analyse(clean_bag: str, tampered_bag: str, samples: int = 400) -> dict:
    print(f"reading {clean_bag}")
    c = read_bag(clean_bag, [TRAJ, ODOM])
    print(f"reading {tampered_bag}")
    t = read_bag(tampered_bag, [TRAJ, ODOM])

    res = {"clean_bag": clean_bag, "tampered_bag": tampered_bag}

    # --- driven trajectory, d_Fe
    ce, te = ego_path(c[ODOM]), ego_path(t[ODOM])
    print(f"\ndriven points : clean {len(ce)}   tampered {len(te)}")
    if len(ce) >= 2 and len(te) >= 2:
        cl = sum(math.dist(ce[i], ce[i + 1]) for i in range(len(ce) - 1))
        tl = sum(math.dist(te[i], te[i + 1]) for i in range(len(te) - 1))
        res["driven_length_clean_m"] = round(cl, 2)
        res["driven_length_tampered_m"] = round(tl, 2)
        a, b = resample(ce, samples), resample(te, samples)
        res["d_Fe"] = round(frechet(a, b), 4)
        res["max_lateral_e"] = round(max_pointwise(a, b), 4)
    else:
        res["d_Fe"] = None
        print("  ! not enough driven points -- did the vehicle move?")

    # --- planned trajectory, d_Fp
    cp, tp = planned_path(c[TRAJ]), planned_path(t[TRAJ])
    print(f"planned points: clean {len(cp)}   tampered {len(tp)}")
    if len(cp) >= 2 and len(tp) >= 2:
        a, b = resample(cp, samples), resample(tp, samples)
        res["d_Fp"] = round(frechet(a, b), 4)
        res["max_lateral_p"] = round(max_pointwise(a, b), 4)
    else:
        res["d_Fp"] = None

    return res


def report(r: dict, threshold: float = 0.5) -> None:
    print("\n" + "=" * 58)
    print("  TRAJECTORY DEVIATION: clean vs tampered")
    print("=" * 58)

    if r.get("driven_length_clean_m"):
        print(f"  route driven, clean    : {r['driven_length_clean_m']:>8.2f} m")
        print(f"  route driven, tampered : {r['driven_length_tampered_m']:>8.2f} m")

    print()
    for key, label in (("d_Fp", "planning deviation  d_Fp"),
                       ("d_Fe", "driven deviation    d_Fe")):
        v = r.get(key)
        if v is None:
            print(f"  {label} :      n/a")
            continue
        verdict = "OVER threshold" if v > threshold else "within threshold"
        print(f"  {label} : {v:>8.4f} m   {verdict}")

    print(f"\n  safety threshold th = {threshold} m")
    print("  (3.0 m lane, 1.895 m vehicle -- Sato et al.)")

    d = r.get("d_Fe")
    if d is not None:
        print("\n  Sato et al. published d_Fe on this map:")
        print("    w_l 3.5 m -> 0.6049    w_l 4.0 m -> 0.8419")
        print("    w_l 4.5 m -> 1.0965    w_l 5.0 m -> route not completed")
        if d < 0.05:
            print(f"\n  Measured {d:.4f} m -- essentially no deviation.")
            print("  The tampered lanelets are most likely NOT on this route.")
            print("  Check the overlap between the injected lanelet ids and the")
            print("  8 lanelets the mission planner used, then re-inject onto")
            print("  the route. Targeting the victim's route is also the more")
            print("  realistic attack model.")
        elif d > threshold:
            print(f"\n  Measured {d:.4f} m EXCEEDS the {threshold} m threshold:")
            print("  the tampered map moved the vehicle out of safe position.")
        else:
            print(f"\n  Measured {d:.4f} m -- deviation present but under threshold.")


def main():
    ap = argparse.ArgumentParser(description="Frechet trajectory analysis")
    ap.add_argument("--clean", required=True)
    ap.add_argument("--tampered", required=True)
    ap.add_argument("--samples", type=int, default=400,
                    help="resample count (Frechet is O(n*m))")
    ap.add_argument("--threshold", type=float, default=0.5)
    ap.add_argument("--out")
    a = ap.parse_args()

    for p in (a.clean, a.tampered):
        if not Path(p).exists():
            sys.exit(f"bag not found: {p}")

    r = analyse(a.clean, a.tampered, a.samples)
    report(r, a.threshold)

    if a.out:
        Path(a.out).write_text(json.dumps(r, indent=2))
        print(f"\nwrote {a.out}")


if __name__ == "__main__":
    main()