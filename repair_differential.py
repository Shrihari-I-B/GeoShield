#!/usr/bin/env python3
"""Differential repair: restore rejected lanelets from the trusted prior.

WHY NOT WIDTH-CLAMPING. repair.py corrects a lanelet's MEAN width with a
uniform shift along its left boundary. That inverts a uniform attack. It does
not invert a tapered one: measured 2026-10-26 on route_g3.0c, clamping moved
lanelet 3013032's centre from 1.174 m off clean to 2.228 m off while reporting
the width restored, because the mean converged and the shape did not.

When a trusted prior exists -- which is the supply-chain threat model's whole
premise -- the exact repair is to copy the node coordinates back. Lanelet2 node
IDs are stable across map versions (36936 shared between the clean map and
route_g3.0c), so restoration is a copy, not an estimate. Collateral damage to
neighbours sharing displaced seam nodes is undone in the same pass.

Both coordinate representations are restored: Autoware reads lat/lon, the
adapters read local_x/local_y. See osm_geom for why that matters.
"""
import argparse, json
import xml.etree.ElementTree as ET
from pathlib import Path


def boundary_ways(root):
    """lanelet id -> [way ids] for its left and right boundaries."""
    out = {}
    for rel in root.findall("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        if tags.get("type") != "lanelet":
            continue
        ws = [int(m.get("ref")) for m in rel.findall("member")
              if m.get("role") in ("left", "right")]
        if ws:
            out[int(rel.get("id"))] = ws
    return out


def main():
    ap = argparse.ArgumentParser(description="GeoShield differential repair")
    ap.add_argument("--candidate", required=True, help="map under test")
    ap.add_argument("--prior", required=True, help="last trusted map")
    ap.add_argument("--diff", required=True, help="differential_verify report")
    ap.add_argument("--out", required=True, help="repaired .osm")
    ap.add_argument("--include-suspect", action="store_true")
    ap.add_argument("--report")
    a = ap.parse_args()

    rep = json.loads(Path(a.diff).read_text())
    want = {"REJECT"} | ({"SUSPECT"} if a.include_suspect else set())
    targets = sorted({c["segment_id"] for c in rep.get("changes", [])
                      if c.get("verdict") in want})
    print(f"prior     : {a.prior}")
    print(f"candidate : {a.candidate}")
    print(f"verdict   : {rep.get('verdict')}")
    print(f"lanelets to restore: {len(targets)}")

    ctree = ET.parse(a.candidate); croot = ctree.getroot()
    ptree = ET.parse(a.prior);     proot = ptree.getroot()

    cnodes = {int(n.get("id")): n for n in croot.findall("node")}
    pnodes = {int(n.get("id")): n for n in proot.findall("node")}
    cways = {int(w.get("id")): [int(nd.get("ref")) for nd in w.findall("nd")]
             for w in croot.findall("way")}
    cbw = boundary_ways(croot)

    todo, missing = set(), 0
    for sid in targets:
        lid = int(sid.split(":")[1])
        for wid in cbw.get(lid, []):
            for nid in cways.get(wid, []):
                todo.add(nid)

    restored = unchanged = 0
    for nid in sorted(todo):
        cn, pn = cnodes.get(nid), pnodes.get(nid)
        if cn is None or pn is None:
            missing += 1
            continue
        ct = {t.get("k"): t for t in cn.findall("tag")}
        pt = {t.get("k"): t.get("v") for t in pn.findall("tag")}
        before = (cn.get("lat"), cn.get("lon"),
                  ct["local_x"].get("v") if "local_x" in ct else None,
                  ct["local_y"].get("v") if "local_y" in ct else None)
        cn.set("lat", pn.get("lat")); cn.set("lon", pn.get("lon"))
        for k in ("local_x", "local_y"):
            if k in ct and k in pt:
                ct[k].set("v", pt[k])
        after = (cn.get("lat"), cn.get("lon"),
                 ct["local_x"].get("v") if "local_x" in ct else None,
                 ct["local_y"].get("v") if "local_y" in ct else None)
        if before == after:
            unchanged += 1
        else:
            restored += 1

    # Structure changes. A role present in the candidate but absent from the
    # prior is an injected member -- centerline_attack.py adds an explicit
    # centreline way to lanelets that had none, overriding the centreline
    # Autoware would otherwise compute from the boundaries. Restoring node
    # coordinates cannot undo that; the member itself has to go. The orphaned
    # way and its nodes are left in place: Lanelet2 ignores ways no relation
    # references, and removing them risks breaking ids other relations share.
    rels = {int(r.get("id")): r for r in croot.findall("relation")}
    members_removed, roles_seen = 0, set()
    for ch in rep.get("changes", []):
        if ch.get("field_name") != "structure":
            continue
        if ch.get("verdict") not in want:
            continue
        added = set(ch.get("candidate") or []) - set(ch.get("previous") or [])
        if not added:
            continue
        rel = rels.get(int(ch["segment_id"].split(":")[1]))
        if rel is None:
            continue
        for mem in list(rel.findall("member")):
            if mem.get("role") in added:
                rel.remove(mem)
                members_removed += 1
                roles_seen.add(mem.get("role"))

    ctree.write(a.out, encoding="utf-8", xml_declaration=True)
    print(f"\nwrote {a.out}")
    print(f"  boundary nodes considered : {len(todo)}")
    print(f"  restored from prior       : {restored}")
    print(f"  already matched prior     : {unchanged}")
    print(f"  absent from prior         : {missing}")
    print(f"  injected members removed  : {members_removed}"
          f"{' (' + ', '.join(sorted(roles_seen)) + ')' if roles_seen else ''}")

    if a.report:
        Path(a.report).write_text(json.dumps({
            "policy": "restore_from_prior", "prior": a.prior,
            "candidate": a.candidate, "output": a.out, "diff": a.diff,
            "lanelets_restored": targets,
            "nodes_considered": len(todo), "nodes_restored": restored,
            "nodes_unchanged": unchanged, "nodes_missing": missing,
            "members_removed": members_removed,
            "roles_removed": sorted(roles_seen),
        }, indent=2))
        print(f"  report                    : {a.report}")


if __name__ == "__main__":
    main()
