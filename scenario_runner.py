#!/usr/bin/env python3
"""
GeoShield -- Phase 6a: headless scenario runner.

Runs on the EC2 instance, alongside a live planning_simulator.

    # discover valid start/goal poses from the map (do this once)
    python3 scenario_runner.py --map ~/autoware_map/nishishinjuku_autoware_map/lanelet2_map.osm --pick-route

    # run one scenario and record a bag
    python3 scenario_runner.py --scenario scenario.json --bag bags/clean_run1

INTERFACE (confirmed on this build, Autoware + ROS 2 Jazzy)
-----------------------------------------------------------
    pose        /initialpose                      PoseWithCovarianceStamped
    goal        /planning/mission_planning/goal   PoseStamped
    engage      /api/operation_mode/change_to_autonomous   (SERVICE, ADAPI)
    planned     /planning/scenario_planning/trajectory
    driven      /localization/kinematic_state

Note the engage path: this build exposes the ADAPI service, not the older
/autoware/engage topic. Both appear in `ros2 topic list`, but the service is
the one that actually transitions the operation mode.

WHY POSES ARE SCRIPTED, NOT CLICKED
-----------------------------------
Without RViz there is no "2D Pose Estimate" button, so poses are published
programmatically. That is not a workaround -- it is the methodologically
correct choice. The Phase 6 result is a four-condition comparison (clean,
tampered, tampered+GeoShield, clean+GeoShield) using Frechet distance. Hand
-clicked poses vary by centimetres between runs, and that variance lands
inside the same range as the effect being measured -- you would be reporting
mouse jitter as attack impact. Scripted poses make every run byte-identical.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path


# ----------------------------------------------------------------------
# route discovery -- read real coordinates out of the map
# ----------------------------------------------------------------------

def load_lanelet_centres(map_path: str) -> dict:
    """
    Metric-frame centre point and heading for each lanelet.

    Poses must be real map coordinates. Invented numbers put the ego outside
    the drivable area and the planner silently refuses to route.
    """
    root = ET.parse(map_path).getroot()

    nodes = {}
    for n in root.findall("node"):
        tags = {t.get("k"): t.get("v") for t in n.findall("tag")}
        try:
            nodes[int(n.get("id"))] = (float(tags["local_x"]), float(tags["local_y"]))
        except (KeyError, TypeError, ValueError):
            pass

    ways = {int(w.get("id")): [int(nd.get("ref")) for nd in w.findall("nd")]
            for w in root.findall("way")}

    out = {}
    for rel in root.findall("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        if tags.get("type") != "lanelet" or tags.get("subtype") != "road":
            continue
        left = right = None
        for m in rel.findall("member"):
            if m.get("role") == "left":
                left = int(m.get("ref"))
            elif m.get("role") == "right":
                right = int(m.get("ref"))
        if left not in ways or right not in ways:
            continue
        lp = [nodes[n] for n in ways[left] if n in nodes]
        rp = [nodes[n] for n in ways[right] if n in nodes]
        if len(lp) < 2 or len(rp) < 2:
            continue

        start = ((lp[0][0] + rp[0][0]) / 2, (lp[0][1] + rp[0][1]) / 2)
        end = ((lp[-1][0] + rp[-1][0]) / 2, (lp[-1][1] + rp[-1][1]) / 2)
        yaw = math.atan2(end[1] - start[1], end[0] - start[0])
        out[int(rel.get("id"))] = {
            "start": start, "end": end, "yaw": yaw,
            "length": math.dist(start, end),
        }
    return out


def load_topology(map_path: str):
    """
    Successor graph, derived the same way as lanelet2_adapter: two lanelets
    are connected when the second one's boundaries START at the node ids where
    the first one's boundaries END.
    """
    root = ET.parse(map_path).getroot()
    ways = {int(w.get("id")): [int(nd.get("ref")) for nd in w.findall("nd")]
            for w in root.findall("way")}

    ends, starts = {}, {}
    for rel in root.findall("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        if tags.get("type") != "lanelet":
            continue
        left = right = None
        for m in rel.findall("member"):
            if m.get("role") == "left":
                left = int(m.get("ref"))
            elif m.get("role") == "right":
                right = int(m.get("ref"))
        if left not in ways or right not in ways:
            continue
        ln, rn = ways[left], ways[right]
        if not ln or not rn:
            continue
        lid = int(rel.get("id"))
        starts.setdefault((ln[0], rn[0]), []).append(lid)
        ends[lid] = (ln[-1], rn[-1])

    succ = {lid: [] for lid in ends}
    for lid, key in ends.items():
        for nxt in starts.get(key, []):
            if nxt != lid:
                succ[lid].append(nxt)
    return succ


def pick_route_reachable(centres: dict, succ: dict, min_len: float = 120.0,
                         max_len: float = 400.0) -> dict | None:
    """
    Pick start and goal by WALKING THE GRAPH, not by euclidean distance.

    Two lanelets 838 m apart in a straight line may sit in different connected
    components -- this map has 75 lanelets with no successors at all, so
    disconnected pieces certainly exist. The planner then refuses to route and
    the run produces no trajectory.

    BFS from a start gives goals that are reachable BY CONSTRUCTION, and the
    accumulated path length is the true driving distance rather than a chord.
    """
    from collections import deque

    best = None
    for start in centres:
        if start not in succ:
            continue
        # BFS, accumulating along-route distance
        seen = {start: 0.0}
        q = deque([start])
        while q:
            cur = q.popleft()
            d = seen[cur]
            if d > max_len:
                continue
            for nxt in succ.get(cur, []):
                if nxt in seen or nxt not in centres:
                    continue
                seen[nxt] = d + centres[nxt]["length"]
                q.append(nxt)

        ok = [(d, lid) for lid, d in seen.items()
              if min_len <= d <= max_len and lid != start]
        if not ok:
            continue
        ok.sort(reverse=True)          # longest reachable route in range
        d, goal = ok[0]
        if best is None or d > best[0]:
            a, b = centres[start], centres[goal]
            best = (d, {
                "start_lanelet": start, "goal_lanelet": goal,
                "start": {"x": a["start"][0], "y": a["start"][1], "yaw": a["yaw"]},
                "goal": {"x": b["end"][0], "y": b["end"][1], "yaw": b["yaw"]},
                "route_length_m": round(d, 1),
                "n_reachable": len(seen),
            })
    return best[1] if best else None


def pick_route(centres: dict, min_sep: float = 150.0) -> dict | None:
    """Pick a start and a goal far enough apart to produce a real route."""
    items = sorted(centres.items(), key=lambda kv: -kv[1]["length"])
    for lid, a in items[:60]:
        for gid, b in items[:60]:
            if lid == gid:
                continue
            if math.dist(a["start"], b["end"]) >= min_sep:
                return {
                    "start_lanelet": lid, "goal_lanelet": gid,
                    "start": {"x": a["start"][0], "y": a["start"][1], "yaw": a["yaw"]},
                    "goal": {"x": b["end"][0], "y": b["end"][1], "yaw": b["yaw"]},
                    "separation_m": round(math.dist(a["start"], b["end"]), 1),
                }
    return None


# ----------------------------------------------------------------------
# ROS interaction (via CLI -- no rclpy dependency, works in any shell)
# ----------------------------------------------------------------------

def _quat(yaw: float) -> tuple[float, float]:
    return math.sin(yaw / 2), math.cos(yaw / 2)


def publish_pose(x, y, yaw, topic="/initialpose"):
    z, w = _quat(yaw)
    msg = (
        "{header: {frame_id: 'map'}, "
        f"pose: {{pose: {{position: {{x: {x}, y: {y}, z: 0.0}}, "
        f"orientation: {{x: 0.0, y: 0.0, z: {z}, w: {w}}}}}, "
        "covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0.25,0,0,0, "
        "0,0,0,0.06853,0,0, 0,0,0,0,0.06853,0, 0,0,0,0,0,0.06853]}}"
    )
    return subprocess.run(
        ["ros2", "topic", "pub", "--once", topic,
         "geometry_msgs/msg/PoseWithCovarianceStamped", msg],
        capture_output=True, text=True, timeout=30)


def publish_goal(x, y, yaw, topic="/planning/mission_planning/goal"):
    z, w = _quat(yaw)
    msg = (
        "{header: {frame_id: 'map'}, "
        f"pose: {{position: {{x: {x}, y: {y}, z: 0.0}}, "
        f"orientation: {{x: 0.0, y: 0.0, z: {z}, w: {w}}}}}}}"
    )
    return subprocess.run(
        ["ros2", "topic", "pub", "--once", topic,
         "geometry_msgs/msg/PoseStamped", msg],
        capture_output=True, text=True, timeout=30)

def wait_autonomous_available(timeout: int = 60) -> bool:
    """
    Poll /api/operation_mode/state until autonomous mode is available.

    WHY THIS REPLACES A FIXED SLEEP. is_autonomous_mode_available is false
    until a route exists -- measured: false with Routing Unset, true within
    seconds of Routing Set. run() previously slept settle=5 s after
    publish_goal() and called engage() regardless. When routing takes longer
    than 5 s the ADAPI service is called before the transition is possible
    and blocks until its own timeout, which raised TimeoutExpired and killed
    the run after 2 s of recording.
    """
    start = time.time()
    while time.time() - start < timeout:
        r = subprocess.run(
            ["ros2", "topic", "echo", "--once",
             "/api/operation_mode/state"],
            capture_output=True, text=True, timeout=10)
        if "is_autonomous_mode_available: true" in r.stdout:
            return True
        time.sleep(2)
    return False


def engage():
    """ADAPI service call -- this build has no working /autoware/engage topic."""
    try:
        return subprocess.run(
            ["ros2", "service", "call", "/api/operation_mode/change_to_autonomous",
             "autoware_adapi_v1_msgs/srv/ChangeOperationMode", "{}"],
            capture_output=True, text=True, timeout=30)
    except subprocess.TimeoutExpired:
        class _R:
            returncode = 1
            stdout = ""
            stderr = ("engage timed out after 30s -- autonomous mode was not "
                      "available. Relaunch Autoware and re-run.")
        return _R()

def topic_alive(topic: str, timeout: int = 10) -> bool:
    try:
        r = subprocess.run(["ros2", "topic", "echo", "--once", topic],
                           capture_output=True, text=True, timeout=timeout)
        return r.returncode == 0 and bool(r.stdout.strip())
    except subprocess.TimeoutExpired:
        return False


TOPICS = [
    "/planning/scenario_planning/trajectory",
    "/localization/kinematic_state",
    "/planning/mission_planning/goal",
    "/initialpose",
    "/tf",
    "/tf_static",
]


def run(scenario: dict, bag: str, duration: int = 90, settle: int = 5) -> dict:
    out = {"scenario": scenario, "bag": bag, "steps": {}}

    # ros2 bag record -o refuses to write into an existing directory, so a
    # second attempt at the same bag path silently records nothing while the
    # rest of the run proceeds. Measured: a stale bags/demo_clean from an
    # aborted attempt was still present, the recorder failed, and verify_run
    # then read the OLD bag and reported "1 driven point".
    import shutil
    if Path(bag).exists():
        shutil.rmtree(bag, ignore_errors=True)
        print(f"  removed existing bag at {bag}")

    print("  recording ->", bag)
    rec = subprocess.Popen(["ros2", "bag", "record", "-o", bag] + TOPICS,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2)

    try:
        s = scenario["start"]
        print(f"  initial pose ({s['x']:.1f}, {s['y']:.1f})")
        publish_pose(s["x"], s["y"], s["yaw"])
        time.sleep(settle)

        # localisation must converge before a goal will be accepted
        ok = topic_alive("/localization/kinematic_state", 15)
        out["steps"]["localized"] = ok
        if not ok:
            print("  ! no kinematic_state -- pose may be off the map")

        g = scenario["goal"]
        print(f"  goal ({g['x']:.1f}, {g['y']:.1f})")
        publish_goal(g["x"], g["y"], g["yaw"])
        time.sleep(settle)

        ok = topic_alive("/planning/scenario_planning/trajectory", 20)
        out["steps"]["planned"] = ok
        print(f"  trajectory: {'yes' if ok else 'NO -- routing failed'}")

        avail = wait_autonomous_available(60)
        out["steps"]["mode_available"] = avail
        if not avail:
            print("  ! autonomous mode never became available in 60s")
            print("    (routing may have failed, or the stack is already at")
            print("     the goal from a previous run -- relaunch Autoware)")
        r = engage()
        out["steps"]["engaged"] = r.returncode == 0
        print(f"  engage: {'ok' if r.returncode == 0 else r.stderr.strip()[:120]}")

        print(f"  driving for {duration}s ...")
        time.sleep(duration)

    finally:
        rec.terminate()
        rec.wait(timeout=10)
        print("  stopped recording")

    chk = verify_run(bag, scenario)
    out["verification"] = chk
    if chk["ok"]:
        print(f"  verified: drove {chk['driven_length_m']} m, "
              f"start error {chk['start_error_m']} m")
    else:
        print("  !! RUN NOT VERIFIED -- do not use this bag")
        for w in chk["warnings"]:
            print(f"     - {w}")

    return out

def verify_run(bag: str, sc: dict, tol: float = 5.0) -> dict:
    """
    Post-drive sanity check on the RECORDED bag, not on the ROS graph.

    WHY THIS EXISTS. A run on bags/g3_repeat printed
    "trajectory: yes / engage: ok / stopped recording" while the vehicle
    started at the GOAL instead of the start pose and drove 814.61 m
    round-trip on a 393.2 m one-way route. Measured: start (81857.5,
    50439.2) == end (81857.5, 50439.2), the goal coordinates. The stack
    was still engaged and arrived from the previous run, so re-publishing
    the initial pose did not reset it.

    trajectory/engage checks pass on that run because they only ask
    whether a plan exists and the service accepted -- neither verifies
    where the vehicle actually was. This does.
    """
    import importlib.util
    res = {"ok": None, "warnings": []}
    try:
        spec = importlib.util.spec_from_file_location(
            "_fa", str(Path(__file__).parent / "frechet_analysis.py"))
        fa = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(fa)
        pts = fa.ego_path(fa.read_bag(bag, [fa.ODOM])[fa.ODOM])
    except Exception as exc:
        res["warnings"].append(f"could not read bag for verification: {exc}")
        return res

    if len(pts) < 2:
        res["ok"] = False
        res["warnings"].append(f"only {len(pts)} driven points -- vehicle did not move")
        return res

    driven = fa.arc_lengths(pts)[-1]
    res["driven_length_m"] = round(driven, 2)
    res["start_actual"] = [round(pts[0][0], 1), round(pts[0][1], 1)]
    res["end_actual"] = [round(pts[-1][0], 1), round(pts[-1][1], 1)]

    want_s = (float(sc["start"]["x"]), float(sc["start"]["y"]))
    want_g = (float(sc["goal"]["x"]), float(sc["goal"]["y"]))
    d_start = math.dist(pts[0], want_s)
    d_goal = math.dist(pts[-1], want_g)
    res["start_error_m"] = round(d_start, 2)
    res["goal_error_m"] = round(d_goal, 2)

    if d_start > tol:
        res["warnings"].append(
            f"started {d_start:.1f} m from the scenario start pose "
            f"(tolerance {tol} m). If this is close to the GOAL, the stack "
            f"was still engaged from a previous run -- relaunch Autoware "
            f"and re-run. DO NOT use this bag.")
    if d_goal > tol:
        res["warnings"].append(
            f"ended {d_goal:.1f} m from the goal -- route not completed")

    route = float(sc.get("route_length_m", 0) or 0)
    if route and driven > route * 1.5:
        res["warnings"].append(
            f"drove {driven:.0f} m on a {route:.0f} m route -- likely a loop "
            f"or a re-engage, not a single traversal")

    res["ok"] = not res["warnings"]
    return res
# ----------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="GeoShield scenario runner")
    ap.add_argument("--map", help="Lanelet2 .osm (for --pick-route)")
    ap.add_argument("--pick-route", action="store_true", dest="pick")
    ap.add_argument("--min-sep", type=float, default=120.0, dest="min_sep",
                    help="minimum along-route length (m)")
    ap.add_argument("--max-len", type=float, default=400.0, dest="max_len",
                    help="maximum along-route length (m)")
    ap.add_argument("--scenario", help="scenario JSON from --pick-route")
    ap.add_argument("--bag", help="output bag path")
    ap.add_argument("--duration", type=int, default=90)
    ap.add_argument("--out", help="write run report JSON")
    a = ap.parse_args()

    if a.pick:
        if not a.map:
            sys.exit("--pick-route needs --map")
        centres = load_lanelet_centres(a.map)
        succ = load_topology(a.map)
        print(f"road lanelets with metric geometry: {len(centres)}")
        linked = sum(1 for l in centres if succ.get(l))
        print(f"with successors: {linked}")
        sc = pick_route_reachable(centres, succ, a.min_sep, a.max_len)
        if not sc:
            sys.exit("no reachable route -- try --min-sep 60")
        print(f"route is reachable through the graph "
              f"({sc['n_reachable']} lanelets within range)")
        print(json.dumps(sc, indent=2))
        Path("scenario.json").write_text(json.dumps(sc, indent=2))
        print("\nwrote scenario.json")
        return

    if not (a.scenario and a.bag):
        sys.exit("need --scenario and --bag")

    sc = json.loads(Path(a.scenario).read_text())
    print(f"scenario: lanelet {sc['start_lanelet']} -> {sc['goal_lanelet']}"
          f"  ({sc.get('route_length_m', sc.get('separation_m', 0))} m apart)")
    rep = run(sc, a.bag, a.duration)

    if a.out:
        Path(a.out).write_text(json.dumps(rep, indent=2))
        print(f"wrote {a.out}")

    v = rep.get("verification", {})
    if v.get("ok") is False:
        print("\n! Run rejected by post-drive verification. The bag does not")
        print("  match the scenario. Relaunch Autoware (Ctrl-C Terminal 1 and")
        print("  launch again) so the stack starts from a clean state, then")
        print("  re-run. An already-arrived stack does not reset on a new pose.")
        sys.exit(3)

    if not rep["steps"].get("planned"):
        print("\n! No trajectory was produced. Check that the start pose sits on")
        print("  a drivable lanelet and that the goal is reachable from it.")
        sys.exit(2)


if __name__ == "__main__":
    main()