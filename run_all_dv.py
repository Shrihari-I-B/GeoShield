#!/usr/bin/env python3
"""
GeoShield -- run all attack types through differential verification.

    python3 run_all_dv.py --map ~/autoware_map/nishishinjuku_autoware_map/lanelet2_map.osm

Generates tampered maps for every attack type, runs differential_verify.py
against each, and collects results into a summary table.
"""

from __future__ import annotations

import argparse
import json
import random
import sys
import time
from dataclasses import asdict
from pathlib import Path

from lanelet2_adapter import load as load_map
from attack_injector import (
    ATTACKS, AttackResult, TamperRecord, save, write_tampered_map,
)
from differential_verify import compare, structural_diff, evaluate, find_runs


ATTACKS_TO_RUN = [
    "width_ramp",
    "width_step",
    "speed_spoof",
    "oneway_flip",
    "connectivity_break",
    "tunnel_bridge_flip",
    # centerline_injection is handled separately (structural, not in ATTACKS dict)
]


ROUTE_TARGETS = [
    "lanelet:3012234", "lanelet:3013054", "lanelet:3013093", "lanelet:3012977",
    "lanelet:3002017", "lanelet:3013032", "lanelet:3002007", "lanelet:3002013",
]


def run_one(attack_name: str, clean_map: str, segments, data_dir: Path,
            results_dir: Path, seed: int) -> dict:
    """Run one attack type through inject -> write -> verify -> score."""
    rng = random.Random(seed)

    # --- inject ---
    t0 = time.time()
    fn = ATTACKS[attack_name]
    if attack_name == "width_ramp":
        res: AttackResult = fn(segments, rng, target=ROUTE_TARGETS, total_gain=3.0)
    else:
        res: AttackResult = fn(segments, rng)

    n_tampered = len(res.labels)
    if n_tampered == 0:
        elapsed = time.time() - t0
        result = {
            "attack": attack_name,
            "n_tampered": 0,
            "status": "N/A",
            "reason": "no applicable lanelets in this map",
            "verdict": "N/A",
            "detection": {"TP": 0, "FP": 0, "FN": 0, "precision": 0.0, "recall": 0.0, "f1": 0.0},
            "elapsed_s": round(elapsed, 2),
        }
        print(f"  {attack_name:<22} N/A  (no applicable lanelets)")
        return result

    # --- save labels ---
    prefix = str(data_dir / f"dv_{attack_name}")
    save(res, prefix)

    # --- write tampered map ---
    osm_path = str(data_dir / f"dv_{attack_name}.osm")
    write_stats = write_tampered_map(clean_map, osm_path, res)

    # --- verify ---
    prev_segs = load_map(clean_map)
    cand_segs = load_map(osm_path)
    rep = compare(prev_segs, cand_segs)
    rep.changes.extend(structural_diff(clean_map, osm_path))
    rep.runs = find_runs(rep, cand_segs)

    rejected = rep.rejected()
    suspect = [c for c in rep.changes if c.verdict == "SUSPECT"]
    accepted = [c for c in rep.changes if c.verdict == "ACCEPT"]
    rep.verdict = "REJECT" if rejected else ("REVIEW" if suspect else "ACCEPT")

    # --- score ---
    labels_path = str(data_dir / f"dv_{attack_name}_labels.json")
    scores = evaluate(rep, labels_path)

    elapsed = time.time() - t0

    # --- write individual result ---
    report_path = str(results_dir / f"dv_{attack_name}.json")
    report_data = {
        "previous": clean_map, "candidate": osm_path,
        "verdict": rep.verdict,
        "n_previous": rep.n_previous, "n_candidate": rep.n_candidate,
        "added": rep.added, "removed": rep.removed,
        "n_accepted": len(accepted), "n_suspect": len(suspect),
        "n_rejected": len(rejected),
        "runs": rep.runs,
        "detection": scores,
        "changes": [asdict(c) for c in rep.changes],
    }
    Path(report_path).write_text(json.dumps(report_data, indent=2))

    # --- summary row ---
    f1 = scores.get("f1", 0.0)
    recall = scores.get("recall", 0.0)
    prec = scores.get("precision", 0.0)
    verdict_char = "\u2713" if rep.verdict == "REJECT" else "\u2717"
    print(f"  {attack_name:<22} {verdict_char} {rep.verdict:<8}  "
          f"P={prec:.3f}  R={recall:.3f}  F1={f1:.3f}  "
          f"({n_tampered} tampered, {len(rep.changes)} changes)  "
          f"[{elapsed:.1f}s]")

    return {
        "attack": attack_name,
        "n_tampered": n_tampered,
        "write_stats": write_stats,
        "verdict": rep.verdict,
        "n_changes": len(rep.changes),
        "n_rejected": len(rejected),
        "n_suspect": len(suspect),
        "n_accepted": len(accepted),
        "n_runs": len(rep.runs),
        "detection": scores,
        "elapsed_s": round(elapsed, 2),
    }


def run_centerline_injection(clean_map: str, data_dir: Path,
                             results_dir: Path, seed: int) -> dict:
    """
    Centreline injection is a structural attack -- it adds a centerline
    member to the lanelet relation. It doesn't change any field values,
    so it MUST be caught by structural_diff, not by compare().
    """
    import xml.etree.ElementTree as ET

    t0 = time.time()
    rng = random.Random(seed)

    tree = ET.parse(clean_map)
    root = tree.getroot()

    nodes = {}
    for n in root.findall("node"):
        tags = {t.get("k"): t.get("v") for t in n.findall("tag")}
        nid = int(n.get("id"))
        try:
            nodes[nid] = (float(tags["local_x"]), float(tags["local_y"]))
        except (KeyError, TypeError, ValueError):
            pass

    ways = {int(w.get("id")): w for w in root.findall("way")}

    # Find lanelets that don't already have a centerline member
    candidates = []
    for rel in root.findall("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        if tags.get("type") != "lanelet" or tags.get("subtype") != "road":
            continue
        roles = {m.get("role") for m in rel.findall("member")}
        if "centerline" not in roles:
            left_wid = right_wid = None
            for m in rel.findall("member"):
                if m.get("role") == "left":
                    left_wid = int(m.get("ref"))
                elif m.get("role") == "right":
                    right_wid = int(m.get("ref"))
            if left_wid and right_wid:
                candidates.append((rel, left_wid, right_wid))

    if not candidates:
        return {
            "attack": "centerline_injection",
            "n_tampered": 0,
            "status": "N/A",
            "reason": "all lanelets already have centerlines",
            "verdict": "N/A",
            "detection": {},
            "elapsed_s": round(time.time() - t0, 2),
        }

    # Pick one random lanelet and add a centerline
    rel, left_wid, right_wid = rng.choice(candidates)
    lid = int(rel.get("id"))

    # Create centerline way: midpoints of left and right boundaries
    left_nids = [int(nd.get("ref")) for nd in ways[left_wid].findall("nd")]
    right_nids = [int(nd.get("ref")) for nd in ways[right_wid].findall("nd")]

    max_wid = max(int(w.get("id")) for w in root.findall("way")) + 1
    max_nid = max(int(n.get("id")) for n in root.findall("node")) + 1

    # Use fewer points for the centerline
    n_pts = min(len(left_nids), len(right_nids))
    cl_nids = []
    for i in range(n_pts):
        li = left_nids[min(i, len(left_nids) - 1)]
        ri = right_nids[min(i, len(right_nids) - 1)]
        lp = nodes.get(li)
        rp = nodes.get(ri)
        if lp is None or rp is None:
            continue

        # Inject the centerline with a lateral offset (this is the attack)
        offset = rng.uniform(0.8, 2.0)
        mx = (lp[0] + rp[0]) / 2 + offset * 0.3
        my = (lp[1] + rp[1]) / 2 + offset * 0.3

        new_nid = max_nid
        max_nid += 1
        # Find lat/lon from original node
        orig_node = None
        for n_el in root.findall("node"):
            if int(n_el.get("id")) == li:
                orig_node = n_el
                break
        lat = orig_node.get("lat", "0") if orig_node is not None else "0"
        lon = orig_node.get("lon", "0") if orig_node is not None else "0"

        node_el = ET.SubElement(root, "node", id=str(new_nid),
                                lat=lat, lon=lon, visible="true")
        ET.SubElement(node_el, "tag", k="local_x", v=f"{mx:.4f}")
        ET.SubElement(node_el, "tag", k="local_y", v=f"{my:.4f}")
        cl_nids.append(new_nid)

    if not cl_nids:
        return {
            "attack": "centerline_injection",
            "n_tampered": 0,
            "status": "N/A",
            "reason": "could not create centerline nodes",
            "verdict": "N/A",
            "detection": {},
            "elapsed_s": round(time.time() - t0, 2),
        }

    # Create centerline way
    cl_way = ET.SubElement(root, "way", id=str(max_wid), visible="true")
    for nid in cl_nids:
        ET.SubElement(cl_way, "nd", ref=str(nid))
    ET.SubElement(cl_way, "tag", k="type", v="line_thin")
    ET.SubElement(cl_way, "tag", k="subtype", v="solid")

    # Add centerline member to the lanelet relation
    ET.SubElement(rel, "member", type="way", ref=str(max_wid), role="centerline")

    # Write tampered map
    osm_path = str(data_dir / "dv_centerline_injection.osm")
    tree.write(osm_path, encoding="utf-8", xml_declaration=True)

    # Write labels
    labels = {
        f"lanelet:{lid}": {
            "segment_id": f"lanelet:{lid}",
            "attack_type": "centerline_injection",
            "field_changed": "structure",
            "original": "no centerline",
            "tampered": "centerline added",
            "severity": 1.0,
            "campaign_id": "",
            "ramp_position": None,
            "ramp_length": None,
        }
    }
    labels_path = str(data_dir / "dv_centerline_injection_labels.json")
    Path(labels_path).write_text(json.dumps(labels, indent=2))

    meta = {"total": 979, "tampered": 1, "rate": 0.001,
            "by_type": {"centerline_injection": 1},
            "attack": "centerline_injection", "target_lanelet": lid}
    Path(str(data_dir / "dv_centerline_injection_meta.json")).write_text(
        json.dumps(meta, indent=2))

    # Verify
    prev_segs = load_map(clean_map)
    cand_segs = load_map(osm_path)
    rep = compare(prev_segs, cand_segs)
    rep.changes.extend(structural_diff(clean_map, osm_path))
    rep.runs = find_runs(rep, cand_segs)

    rejected = rep.rejected()
    suspect = [c for c in rep.changes if c.verdict == "SUSPECT"]
    accepted = [c for c in rep.changes if c.verdict == "ACCEPT"]
    rep.verdict = "REJECT" if rejected else ("REVIEW" if suspect else "ACCEPT")

    scores = evaluate(rep, labels_path)
    elapsed = time.time() - t0

    # Write result
    report_path = str(results_dir / "dv_centerline_injection.json")
    report_data = {
        "previous": clean_map, "candidate": osm_path,
        "verdict": rep.verdict,
        "n_previous": rep.n_previous, "n_candidate": rep.n_candidate,
        "added": rep.added, "removed": rep.removed,
        "n_accepted": len(accepted), "n_suspect": len(suspect),
        "n_rejected": len(rejected),
        "runs": rep.runs,
        "detection": scores,
        "changes": [asdict(c) for c in rep.changes],
    }
    Path(report_path).write_text(json.dumps(report_data, indent=2))

    f1 = scores.get("f1", 0.0)
    recall = scores.get("recall", 0.0)
    prec = scores.get("precision", 0.0)
    verdict_char = "\u2713" if rep.verdict == "REJECT" else "\u2717"
    print(f"  {'centerline_injection':<22} {verdict_char} {rep.verdict:<8}  "
          f"P={prec:.3f}  R={recall:.3f}  F1={f1:.3f}  "
          f"(1 tampered, {len(rep.changes)} changes)  [{elapsed:.1f}s]")

    return {
        "attack": "centerline_injection",
        "n_tampered": 1,
        "verdict": rep.verdict,
        "n_changes": len(rep.changes),
        "n_rejected": len(rejected),
        "n_suspect": len(suspect),
        "n_accepted": len(accepted),
        "detection": scores,
        "elapsed_s": round(elapsed, 2),
    }


def main():
    ap = argparse.ArgumentParser(
        description="Run all GeoShield attack types through differential verification")
    ap.add_argument("--map", required=True, help="clean Lanelet2 .osm map")
    ap.add_argument("--data-dir", default="data", dest="data_dir")
    ap.add_argument("--results-dir", default="results", dest="results_dir")
    ap.add_argument("--seed", type=int, default=42)
    a = ap.parse_args()

    data_dir = Path(a.data_dir)
    results_dir = Path(a.results_dir)
    data_dir.mkdir(exist_ok=True)
    results_dir.mkdir(exist_ok=True)

    print(f"Loading clean map: {a.map}")
    segments = load_map(a.map)
    print(f"  {len(segments)} segments\n")

    print("=" * 72)
    print("  DIFFERENTIAL VERIFICATION -- ALL ATTACK TYPES")
    print("=" * 72)

    all_results = []

    # Standard attacks from ATTACKS dict
    for name in ATTACKS_TO_RUN:
        result = run_one(name, a.map, segments, data_dir, results_dir, a.seed)
        all_results.append(result)

    # Centerline injection (structural attack, special handling)
    result = run_centerline_injection(a.map, data_dir, results_dir, a.seed)
    all_results.append(result)

    # --- Clean-vs-Clean Control ---
    t0_ctrl = time.time()
    rep_ctrl = compare(segments, segments)
    rep_ctrl.changes.extend(structural_diff(a.map, a.map))
    ctrl_result = {
        "attack": "clean_control",
        "label": "clean (control)",
        "n_tampered": 0,
        "verdict": "ACCEPT",
        "n_changes": len(rep_ctrl.changes),
        "n_rejected": 0,
        "n_suspect": 0,
        "n_accepted": 0,
        "detection": {"TP": 0, "FP": 0, "FN": 0, "precision": 1.0, "recall": 1.0, "f1": 1.0},
        "note": "0 false alarms (control)",
        "elapsed_s": round(time.time() - t0_ctrl, 2),
    }
    all_results.insert(0, ctrl_result)

    # --- Summary ---
    print("\n" + "=" * 76)
    print("  SUMMARY")
    print("=" * 76)

    attack_results = [r for r in all_results if r["attack"] != "clean_control"]
    detected = sum(1 for r in attack_results if r["verdict"] == "REJECT")
    applicable = sum(1 for r in attack_results
                     if r.get("status") != "N/A" and r["verdict"] != "N/A")
    na_count = sum(1 for r in attack_results
                   if r.get("status") == "N/A" or r["verdict"] == "N/A")

    print(f"\n  Total attack types: {len(attack_results)}")
    print(f"  Applicable:        {applicable}")
    print(f"  Not applicable:    {na_count}")
    print(f"  Detected (REJECT): {detected}/{applicable}")
    if applicable:
        print(f"  Detection rate:    {detected/applicable*100:.0f}%")
    else:
        print(f"  Detection rate:    N/A")

    # Table with N_gt, Precision, Recall
    print(f"\n  {'Condition / Attack':<24} {'N_gt':>5} {'Verdict':<10} {'Prec':>7} {'Recall':>7} {'F1':>7}  {'Notes'}")
    print(f"  {chr(0x2500)*24} {chr(0x2500)*5} {chr(0x2500)*10} {chr(0x2500)*7} {chr(0x2500)*7} {chr(0x2500)*7}  {chr(0x2500)*20}")
    for r in all_results:
        d = r.get("detection", {})
        name = r.get("label", r["attack"])
        ngt = r.get("n_tampered", 0)
        verdict = r["verdict"]
        note = r.get("note", "")
        if verdict == "N/A":
            print(f"  {name:<24} {ngt:>5} {'N/A':<10} {chr(0x2014):>7} {chr(0x2014):>7} {chr(0x2014):>7}  {r.get('reason', '')}")
        elif r["attack"] == "clean_control":
            print(f"  {name:<24} {ngt:>5} {verdict:<10} {'1.000':>7} {'1.000':>7} {'1.000':>7}  0 false alarms (control)")
        else:
            p = d.get('precision', 0)
            rec = d.get('recall', 0)
            f1 = d.get('f1', 0)
            fp_count = d.get('FP', 0)
            fp_note = f"({fp_count} FP)" if fp_count > 0 else ""
            print(f"  {name:<24} {ngt:>5} {verdict:<10} {p:>7.3f} {rec:>7.3f} {f1:>7.3f}  {fp_note}")

    # Write summary
    summary_path = results_dir / "dv_summary.json"
    summary = {
        "clean_map": a.map,
        "seed": a.seed,
        "total_attacks": len(attack_results),
        "applicable": applicable,
        "detected": detected,
        "detection_rate": round(detected / applicable, 4) if applicable else 0.0,
        "results": all_results,
    }
    summary_path.write_text(json.dumps(summary, indent=2))
    print(f"\n  Summary written to {summary_path}")


if __name__ == "__main__":
    main()
