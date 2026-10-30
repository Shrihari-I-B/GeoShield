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


# Required: without the pointcloud there is no NDT localisation, so a map
# directory lacking it is one the planner starts on and cannot localise with.
MAP_ASSETS_REQUIRED = ("pointcloud_map.pcd",)
# Copied when present. The Nishi-Shinjuku map carries no map_projector_info
# .yaml and Autoware loads it regardless (verified in the container
# 2026-10-30), so treating it as mandatory would refuse a map that works.
MAP_ASSETS_OPTIONAL = ("map_projector_info.yaml",)
MAP_ASSETS = MAP_ASSETS_REQUIRED + MAP_ASSETS_OPTIONAL


def install_map(osm, out_dir, assets_from):
    """Write an Autoware map directory the planner can be pointed at.

    Returns (out_dir, missing). `assets_from` names the map, or the map
    directory, holding the pointcloud and projector info. In differential
    mode that is the prior, which is itself an installed Autoware map
    directory -- which is the only reason this ever worked. In no-prior mode
    the candidate is a bare .osm whose directory holds neither, so the
    operator has to supply one.

    Neither asset is optional. Without pointcloud_map.pcd there is no NDT
    localization; without map_projector_info.yaml Autoware cannot place the
    map at all. This function used to skip a missing asset silently behind
    `if s.exists()`, so a no-prior install produced a directory containing
    only lanelet2_map.osm, the gate reported ADMIT, the launch file started
    the planner, and the only trace was an unformatted "map is not loaded
    properly" in the launch output. Measured in the container 2026-10-30:
    /map/pointcloud_map_loader never came up.
    """
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    shutil.copy2(osm, out / "lanelet2_map.osm")

    src = None
    if assets_from:
        p = Path(assets_from)
        src = p if p.is_dir() else p.parent

    missing = []
    for extra in MAP_ASSETS:
        d = out / extra
        if d.exists():
            continue
        s = (src / extra) if src else None
        if s is not None and s.exists():
            try:
                d.symlink_to(os.path.relpath(s, out))
            except OSError:
                shutil.copy2(s, d)
        elif extra in MAP_ASSETS_REQUIRED:
            missing.append(extra)
    return out, missing


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


def _noprior_report(a, cand, res, fixes, propose, refuse, gating, path):
    """Write the no-prior decision in the differential report's schema.

    The differential path leaves three JSON reports; the no-prior path left
    only an .osm, so a refusal existed solely as a line in a container log.
    An ingest gate's output is its audit trail, and that has to hold in both
    modes, not just the one with a prior.

    The schema is the differential one on purpose: publish_flags.py reads
    `changes` and keys on segment_id and verdict, so writing gate_<stem>_in
    .json here means the launch file and the marker publisher need no
    mode-specific handling at all.

    previous/candidate are omitted rather than nulled for a finding with no
    repairable value, because publish_flags formats those fields and a None
    would raise inside a process the launch file runs with output="log" --
    the quietest way a demo can fail.
    """
    # Imported here, not at module scope: _no_prior does the same, and the
    # differential path must stay importable on a machine with no sklearn.
    from fusion import TIER1_MINIMAL
    from repair_noprior import APPLY_RULES

    by_fix = {sid: (attr, cur, cons) for sid, attr, cur, cons in fixes}
    by_prop = {sid: (attr, cur, cons) for sid, attr, cur, cons in propose}
    gating_ids = {f.segment_id for f in gating}

    changes = []
    for f in res.findings:
        entry = {
            "segment_id": f.segment_id,
            "rule": f.rule,
            "severity": f.severity,
            "verdict": "REJECT" if f.segment_id in gating_ids else "SUSPECT",
            "reason": f.detail,
        }
        src = by_fix.get(f.segment_id) or by_prop.get(f.segment_id)
        if src:
            attr, cur, cons = src
            entry["field_name"] = attr
            entry["candidate"] = cur   # what this map declares
            entry["previous"] = cons   # what its neighbours agree on
            entry["repair"] = "apply" if f.segment_id in by_fix else "propose"
        else:
            entry["field_name"] = f.rule
            entry["repair"] = "none"
        changes.append(entry)

    rep = {
        "previous": None,
        "previous_is": ("neighbour consensus within this map, NOT a trusted "
                        "prior's attested value -- for a lanelet with one "
                        "neighbour the 'consensus' is that single neighbour"),
        "candidate": a.candidate,
        "mode": "no-prior",
        "verdict": "REJECT" if gating else "ACCEPT",
        "n_previous": None,
        "n_candidate": len(cand),
        "added": [],
        "removed": [],
        "n_accepted": len(cand) - len({c["segment_id"] for c in changes}),
        "n_suspect": len([c for c in changes if c["verdict"] == "SUSPECT"]),
        "n_rejected": len(gating),
        "runs": [],
        "detection": {
            "rules": sorted(TIER1_MINIMAL),
            "gating_rules": sorted(APPLY_RULES),
            "n_gating": len(gating),
            "n_proposed": len(propose),
            "n_unrepairable": len(refuse),
            "calibration": "none -- the gating rule reads no thresholds",
        },
        "changes": changes,
    }
    Path(path).write_text(json.dumps(rep, indent=1, default=str))
    return rep


def _no_prior(a, rd, stem):
    """Gate a map with no trusted prior to diff against.

    Measured on Nishi-Shinjuku, 2026-10-29: only a rule with a clean-map
    false-positive rate of zero can gate here. R5_orphan fires 10 times on the
    CLEAN map and R4_speed_neighbour 15, and with no prior there is no baseline
    to compare those counts against, so a verdict driven by them refuses every
    map including an untampered one. R5_attribute_neighbour fires zero times on
    clean data, so one firing is evidence on its own.

    The gating decision is therefore calibration-free: the only rule that gates
    reads no thresholds. Calibration still runs for the advisory rules, on the
    candidate itself, which is the only map available by definition.

    Coverage is one of six attack types tested: only oneway_flip trips a
    gating rule. speed_spoof is detected at conditional recall 1.000 but
    through R4_speed_neighbour, which is advisory and never refuses a map.
    Geometry and topology are neither detected nor repaired without a prior
    -- that is the cost of not having one, and it is the whole argument for
    keeping one.
    """
    from tier1_rules import calibrate, load_map, run_tier1
    from fusion import TIER1_MINIMAL
    from repair_noprior import APPLY_RULES, plan
    from repair_noprior import apply as apply_fixes

    print(f"prior     : (none -- no-prior mode)")
    print(f"candidate : {a.candidate}")

    cand = load_map(a.candidate)
    res = run_tier1(cand, calibrate(cand), enabled=TIER1_MINIMAL)
    fixes, propose, advisory = plan(cand, res.findings)
    gating = [f for f in res.findings if f.rule in APPLY_RULES]

    print(f"verdict   : {'REJECT' if gating else 'ACCEPT'}  "
          f"({len(gating)} gating, {len(propose) + len(advisory)} advisory)")

    # Deliberately NOT gate_<stem>_in.json: the differential path owns that
    # name, and both modes can run against the same map (reproduce.sh gate
    # does exactly that on the clean map), so sharing it meant the second run
    # silently overwrote the first. An audit trail that overwrites itself is
    # worse than none.
    report = rd / f"gate_{stem}_noprior.json"
    _noprior_report(a, cand, res, fixes, propose, advisory, gating, report)
    print(f"report    : {report}")

    admitted = None
    if not gating:
        admitted = a.out_map or str(Path(a.report_dir) / f"gate_{stem}_asis.osm")
        shutil.copy2(a.candidate, admitted)
        action = "passed: no zero-FPR rule fired"
    elif a.no_repair:
        action = "refused, repair disabled"
    else:
        rpath = Path(a.out_map or HERE / "data" / f"gate_{stem}_repaired.osm")
        n = apply_fixes(a.candidate, str(rpath), fixes)
        post = run_tier1(load_map(str(rpath)), calibrate(cand),
                         enabled=TIER1_MINIMAL)
        if [f for f in post.findings if f.rule in APPLY_RULES]:
            action = "refused, consensus repair did not clear the gating rule"
        else:
            admitted, action = str(rpath), f"repaired ({n} tags) and re-verified"
            print(f"repaired  : {rpath}")
            for sid, attr, old, new in fixes[:8]:
                print(f"    {sid:<20} {attr:<16} {old} -> {new}")

    if propose:
        print(f"proposed  : {len(propose)} correction(s) for review, not applied "
              f"-- that rule's precision does not support writing them")

    if admitted and a.out_dir:
        d, missing = install_map(admitted, a.out_dir,
                                 a.map_assets or a.candidate)
        print(f"installed : {d}")
        if missing:
            # Not a warning. The gate's exit 0 means "the planner may start on
            # this map", and a map without these is one the planner starts on
            # and cannot localise with. Saying ADMIT here is the gate lying
            # about its own output.
            print(f"ERROR: installed map is missing {', '.join(missing)} -- "
                  f"pass --map-assets <dir> naming the directory that holds "
                  f"them", file=sys.stderr)
            print("gate      : ERROR -- installed map is incomplete")
            return ERROR

    print(f"gate      : {'ADMIT' if admitted else 'REFUSE'} -- {action}")
    return ADMIT if admitted else REFUSE


def main():
    ap = argparse.ArgumentParser(description="GeoShield map integrity gate")
    ap.add_argument("--candidate", required=True, help="map under test (.osm)")
    ap.add_argument("--prior", help="last trusted map (.osm). Without it "
                                   "the gate runs in no-prior mode.")
    ap.add_argument("--out-map", help="where to write the admitted .osm")
    ap.add_argument("--out-dir", help="also install as an Autoware map directory")
    ap.add_argument("--map-assets", help="directory (or a map file inside it) "
                    "holding pointcloud_map.pcd and map_projector_info.yaml. "
                    "Differential mode takes them from the prior; in no-prior "
                    "mode the candidate is a bare .osm, so there is nothing to "
                    "infer them from and this must be given when --out-dir is.")
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

    if not a.prior:
        return _no_prior(a, rd, stem)

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
        d, missing = install_map(admitted, a.out_dir, a.map_assets or a.prior)
        print(f"installed : {d}")
        if missing:
            # Should never fire here: the prior IS an installed Autoware map
            # directory, which is the only reason this path always worked
            # while the no-prior one silently half-installed. It fires if
            # --prior names a bare .osm outside a map directory, and the
            # consequence is identical -- a planner that starts and cannot
            # localise.
            print(f"ERROR: installed map is missing {', '.join(missing)} -- "
                  f"pass --map-assets <dir> naming the directory that holds "
                  f"them", file=sys.stderr)
            print("gate      : ERROR -- installed map is incomplete")
            return ERROR

    if a.publish:
        publish("ACCEPT" if admitted else "REJECT", detail,
                admitted, a.prior, a.diagnostics_topic, a.hold)

    print(f"gate      : {'ADMIT' if admitted else 'REFUSE'} -- {action}")
    return ADMIT if admitted else REFUSE


if __name__ == "__main__":
    sys.exit(main())
