#!/usr/bin/env python3
"""GeoShield map integrity gate.

Runs at the ingest boundary. Verifies a candidate HD map against the last
trusted map, optionally repairs it, and admits or refuses it. Admission is
expressed as an exit code so a launch file can gate the planner on it:

  exit 0   map admitted -- candidate was clean, or was repaired and re-verified
  exit 2   map refused
  exit 1   error

ROS is optional. With --publish, and rclpy importable, the verdict, a
DiagnosticArray and the flagged-lanelet markers are published. Without it the
gate still decides and still writes the admitted map: detection must not
depend on the simulator.

Verification and repair are invoked as subprocesses rather than imported.
differential_verify exposes compare() and structural_diff() but assembles them
in main(); repair_differential keeps its logic in main() entirely. Promoting
both to library functions is open debt.
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ADMIT, REFUSE, ERROR = 0, 2, 1


def run_dv(prior, candidate, report, labels=None):
    """Verify candidate against prior. Returns the parsed report."""
    cmd = [sys.executable, str(HERE / "differential_verify.py"),
           "--previous", str(prior), "--candidate", str(candidate),
           "--report", str(report)]
    if labels:
        cmd += ["--labels", str(labels)]
    subprocess.run(cmd, cwd=HERE, capture_output=True, text=True)
    p = Path(report)
    if not p.exists():
        raise RuntimeError(f"differential_verify wrote no report to {report}")
    return json.loads(p.read_text())


def run_repair(prior, candidate, diff, out, include_suspect, report):
    cmd = [sys.executable, str(HERE / "repair_differential.py"),
           "--candidate", str(candidate), "--prior", str(prior),
           "--diff", str(diff), "--out", str(out), "--report", str(report)]
    if include_suspect:
        cmd.append("--include-suspect")
    r = subprocess.run(cmd, cwd=HERE, capture_output=True, text=True)
    if not Path(out).exists():
        raise RuntimeError(f"repair wrote no map to {out}:\n{r.stderr.strip()}")
    return json.loads(Path(report).read_text()) if Path(report).exists() else {}


def flagged_ids(rep):
    return sorted({c["segment_id"] for c in rep.get("changes", [])
                   if c.get("verdict") in ("REJECT", "SUSPECT")})


def install_map(osm, out_dir, prior):
    """Write an Autoware map directory the planner can be pointed at."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    shutil.copy2(osm, out / "lanelet2_map.osm")
    src = Path(prior).parent
    for extra in ("pointcloud_map.pcd", "map_projector_info.yaml"):
        s, d = src / extra, out / extra
        if s.exists() and not d.exists():
            try:
                d.symlink_to(os.path.relpath(s, out))
            except OSError:
                shutil.copy2(s, d)
    return out


def publish(verdict, detail, map_path, prior, diag_topic, hold):
    """Publish the verdict over ROS. No-op if rclpy is unavailable."""
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from std_msgs.msg import String
        from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    except ImportError:
        print("  (rclpy unavailable; gate decided without publishing)")
        return

    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    rclpy.init()
    node = Node("geoshield_verifier")

    node.create_publisher(String, "/geoshield/verdict", latched).publish(
        String(data=json.dumps({"verdict": verdict, **detail})))

    st = DiagnosticStatus(
        name="geoshield: map_integrity",
        hardware_id="geoshield",
        level=(DiagnosticStatus.OK if verdict == "ACCEPT" else DiagnosticStatus.ERROR),
        message=f"{verdict}: {detail.get('summary', '')}",
        values=[KeyValue(key=k, value=str(v)) for k, v in detail.items()])
    da = DiagnosticArray(status=[st])
    da.header.stamp = node.get_clock().now().to_msg()
    node.create_publisher(DiagnosticArray, diag_topic, latched).publish(da)

    try:
        sys.path.insert(0, str(HERE))
        from publish_flags import load_lanelet_geometry, build_marker_array
        from visualization_msgs.msg import MarkerArray
        rep = detail.get("_report")
        if rep:
            lanelets = load_lanelet_geometry(str(prior))
            node.create_publisher(
                MarkerArray, "/geoshield/flagged_lanelets", latched
            ).publish(build_marker_array(rep, lanelets))
    except Exception as e:                     # markers are cosmetic
        node.get_logger().warn(f"marker publish skipped: {e}")

    if hold > 0:
        node.get_logger().info(f"holding {hold}s for subscribers")
        end = node.get_clock().now().nanoseconds + int(hold * 1e9)
        while node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(node, timeout_sec=0.2)
    else:
        rclpy.spin_once(node, timeout_sec=0.5)   # let TRANSIENT_LOCAL settle
    node.destroy_node()
    rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description="GeoShield map integrity gate")
    ap.add_argument("--candidate", required=True, help="map under test (.osm)")
    ap.add_argument("--prior", required=True, help="last trusted map (.osm)")
    ap.add_argument("--out-map", help="where to write the admitted .osm")
    ap.add_argument("--out-dir", help="also install as an Autoware map directory")
    ap.add_argument("--no-repair", action="store_true",
                    help="refuse a failing map instead of repairing it")
    ap.add_argument("--include-suspect", action="store_true", default=True)
    ap.add_argument("--labels", help="ground truth, for evaluation only")
    ap.add_argument("--report-dir", default="results")
    ap.add_argument("--publish", action="store_true")
    ap.add_argument("--diagnostics-topic", default="/geoshield/diagnostics",
                    help="set to /diagnostics to join Autoware's diagnostic "
                         "graph; off by default so the gate cannot perturb a "
                         "measurement run")
    ap.add_argument("--hold", type=float, default=0.0,
                    help="seconds to keep publishing before exiting")
    a = ap.parse_args()

    # Subprocesses run with cwd=HERE, so a relative report dir would be
    # resolved in two different working directories. Anchor it to the repo.
    rd = Path(a.report_dir)
    if not rd.is_absolute():
        rd = HERE / rd
    rd.mkdir(parents=True, exist_ok=True)
    stem = Path(a.candidate).stem

    print(f"prior     : {a.prior}")
    print(f"candidate : {a.candidate}")

    try:
        rep = run_dv(a.prior, a.candidate, rd / f"gate_{stem}_in.json", a.labels)
    except RuntimeError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return ERROR

    v = rep.get("verdict")
    flagged = flagged_ids(rep)
    print(f"verdict   : {v}  ({len(flagged)} lanelets flagged)")

    admitted, action = None, None
    if v == "ACCEPT":
        admitted, action = a.candidate, "passed unchanged"
    elif a.no_repair:
        action = "refused, repair disabled"
    else:
        rpath = Path(a.out_map or HERE / "data" / f"gate_{stem}_repaired.osm")
        try:
            rr = run_repair(a.prior, a.candidate, rd / f"gate_{stem}_in.json",
                            rpath, a.include_suspect,
                            rd / f"gate_{stem}_repair.json")
        except RuntimeError as e:
            print(f"ERROR: {e}", file=sys.stderr)
            return ERROR
        print(f"repaired  : {rpath}")
        for k in ("lanelets_restored", "nodes_considered", "nodes_restored",
                  "nodes_unchanged", "nodes_missing", "members_removed",
                  "orphan_ways_removed", "orphan_nodes_removed"):
            if k in rr:
                val = rr[k]
                print(f"  {k:<26}: "
                      f"{len(val) if isinstance(val, list) else val}")
        post = run_dv(a.prior, rpath, rd / f"gate_{stem}_out.json")
        if post.get("verdict") == "ACCEPT":
            admitted, action = str(rpath), "repaired and re-verified"
        else:
            action = "refused, repair did not clear verification"

    detail = {"summary": action, "candidate": a.candidate, "prior": a.prior,
              "flagged": len(flagged), "flagged_ids": flagged,
              "admitted": admitted or "", "_report": rep}

    if admitted and a.out_map and Path(admitted) != Path(a.out_map):
        shutil.copy2(admitted, a.out_map)
        admitted = a.out_map
    if admitted and a.out_dir:
        d = install_map(admitted, a.out_dir, a.prior)
        print(f"installed : {d}")

    if a.publish:
        publish("ACCEPT" if admitted else "REJECT", detail,
                admitted, a.prior, a.diagnostics_topic, a.hold)

    print(f"gate      : {'ADMIT' if admitted else 'REFUSE'} -- {action}")
    return ADMIT if admitted else REFUSE


if __name__ == "__main__":
    sys.exit(main())
