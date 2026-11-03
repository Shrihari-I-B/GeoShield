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
import argparse, json, sys, uuid
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


# ----------------------------------------------------------------------
# Gated relation-tag restoration (BUG-05)
# ----------------------------------------------------------------------

# Raw OSM/Lanelet2 keys that carry each gated attribute, matching the keys
# lanelet2_adapter.to_segments() actually reads and the comparator's
# _RAW_TAG_KEYS. Aliases are handled consistently so a tamper that moves a
# value between alias keys (e.g. speed_limit -> maxspeed) is still repaired.
_GATED_TAG_ALIASES = {
    "speed_limit": ("speed_limit", "maxspeed"),
    "one_way": ("one_way", "oneway"),
    "tunnel": ("tunnel",),
    "bridge": ("bridge",),
    "lanes": ("lanes",),
    "subtype": ("subtype", "highway"),
}


def relation_tags(rel):
    """relation -> {tag_key: tag_value}."""
    return {t.get("k"): t.get("v") for t in rel.findall("tag")}


def set_tag(rel, k, v):
    """Set tag k=v on rel, creating it if absent. Returns whether it changed."""
    for t in list(rel.findall("tag")):
        if t.get("k") == k:
            if t.get("v") != v:
                t.set("v", v)
                return True
            return False
    t = ET.Element("tag")
    t.set("k", k)
    t.set("v", v)
    rel.append(t)
    return True


def remove_tag_key(rel, k):
    """Remove all tags with key k from rel. Returns list of removed values."""
    removed = []
    for t in list(rel.findall("tag")):
        if t.get("k") == k:
            removed.append(t.get("v"))
            rel.remove(t)
    return removed


def restore_relation_tags(crel, prel):
    """Restore gated relation tags from the trusted prior.

    Policy:
      * If the prior attests a field (any alias key present), the candidate
        must match the prior: missing tags are restored, wrong values are
        corrected, and stray alias keys are removed.
      * If the prior lacks a field entirely, any candidate alias key for it
        is an unauthorized introduction and is removed.
      * Explicit false values (one_way=no, tunnel=no, ...) are preserved.
      * Genuine absence (neither side has the field) is left alone.
      * Unrelated tags on the candidate relation are never touched.

    Returns a list of (field, prior_value, candidate_value, action) repair
    records. prel may be None when the lanelet has no prior relation.
    """
    repairs = []
    ct = relation_tags(crel)
    pt = relation_tags(prel) if prel is not None else {}
    for field, aliases in _GATED_TAG_ALIASES.items():
        prior_val = None
        prior_key = None
        for k in aliases:
            if k in pt:
                prior_val = pt[k]
                prior_key = k
                break
        cand_keys = [k for k in aliases if k in ct]
        if prior_val is not None:
            # Prior attests this field: candidate must normalize to prior_key
            # = prior_val, and every other alias key is removed.
            before = {k: ct.get(k) for k in aliases}
            set_tag(crel, prior_key, prior_val)
            for k in aliases:
                if k != prior_key:
                    remove_tag_key(crel, k)
            after = {k: relation_tags(crel).get(k) for k in aliases}
            if after != before:
                repairs.append((field, prior_val, before, "normalized_to_prior"))
        else:
            for k in cand_keys:
                for v in remove_tag_key(crel, k):
                    repairs.append((field, None, v, "removed_unauthorized"))
    return repairs


# ----------------------------------------------------------------------
# Relation-member restoration (BUG-05)
# ----------------------------------------------------------------------

def relation_members(rel):
    """rel -> list of (type, ref, role) member tuples."""
    return [(m.get("type"), m.get("ref"), m.get("role"))
            for m in rel.findall("member")]


def restore_members(crel, prel):
    """Restore lost members and remove unauthorized added members.

    Compares candidate relation members against the trusted prior.
    Lost members (prior has, candidate lacks) are re-added with their
    original type, reference id and role. Added members (candidate has,
    prior lacks) are removed. Existing matching members are left alone.

    Returns (removed_count, removed_details, added_count, added_details)
    where details are lists of (type, ref, role) tuples.
    """
    cmembers = relation_members(crel)
    pmembers = relation_members(prel) if prel is not None else []
    cmember_set = set(cmembers)
    pmember_set = set(pmembers)

    lost = pmember_set - cmember_set   # re-add these
    added = cmember_set - pmember_set  # remove these

    removed_details = []
    for m in list(crel.findall("member")):
        key = (m.get("type"), m.get("ref"), m.get("role"))
        if key in added:
            crel.remove(m)
            removed_details.append(key)

    restored_details = []
    for (mtype, ref, role) in sorted(lost):
        m = ET.Element("member")
        m.set("type", mtype)
        m.set("ref", str(ref))
        m.set("role", role)
        crel.append(m)
        restored_details.append((mtype, ref, role))

    return len(removed_details), removed_details, len(restored_details), restored_details


# ----------------------------------------------------------------------
# Referenced-geometry restoration (BUG-05)
# ----------------------------------------------------------------------

def _deep_copy(el):
    """Deep-copy an XML element (tag, attributes, children)."""
    new = ET.Element(el.tag, attrib=dict(el.attrib))
    for child in el:
        new.append(_deep_copy(child))
    return new


def restore_referenced_geometry(croot, proot, pnodes_by_id, member_refs):
    """Restore missing ways and nodes referenced by restored members.

    member_refs: list of (type, ref, role) for members being restored.
    Restores any way/node referenced by those members that is absent from the
    candidate, copying it from the trusted prior. Also recurses: restored
    ways pull in their node references.

    Returns (ways_restored, nodes_restored, missing) where missing is a list
    of (type, ref) that could not be restored because they are absent from the
    prior too.
    """
    pways = {w.get("id"): w for w in proot.findall("way")}
    cway_ids = {w.get("id") for w in croot.findall("way")}
    cnode_ids = {n.get("id") for n in croot.findall("node")}

    needed_way_ids = set()
    needed_node_ids = set()
    for (mtype, ref, role) in member_refs:
        ref_i = int(ref) if ref is not None else None
        if mtype == "way" and ref_i is not None:
            needed_way_ids.add(ref_i)
        elif mtype == "node" and ref_i is not None:
            needed_node_ids.add(ref_i)

    ways_restored = 0
    nodes_restored = 0
    missing = []

    # Restore missing ways (and pull in their node references).
    for wid in sorted(needed_way_ids):
        wid_s = str(wid)
        if wid_s in cway_ids:
            continue
        pw = pways.get(wid_s)
        if pw is None:
            missing.append(("way", wid_s))
            continue
        croot.append(_deep_copy(pw))
        cway_ids.add(wid_s)
        ways_restored += 1
        for nd in pw.findall("nd"):
            try:
                needed_node_ids.add(int(nd.get("ref")))
            except (TypeError, ValueError):
                pass

    # Restore missing nodes.
    for nid in sorted(needed_node_ids):
        nid_s = str(nid)
        if nid_s in cnode_ids:
            continue
        pn = pnodes_by_id.get(nid)
        if pn is None:
            missing.append(("node", nid_s))
            continue
        croot.append(_deep_copy(pn))
        cnode_ids.add(nid_s)
        nodes_restored += 1

    return ways_restored, nodes_restored, missing


def main():
    ap = argparse.ArgumentParser(description="GeoShield differential repair")
    ap.add_argument("--candidate", required=True, help="map under test")
    ap.add_argument("--prior", required=True, help="last trusted map")
    ap.add_argument("--diff", required=True, help="differential_verify report")
    ap.add_argument("--out", required=True, help="repaired .osm")
    ap.add_argument("--include-suspect", action="store_true")
    ap.add_argument("--report")
    ap.add_argument("--invocation-id", help="unique ID embedded in repair report")
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
    prior_rels = {int(r.get("id")): r for r in proot.findall("relation")
                  if r.get("id") is not None}

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

    # Tag and relation-member restoration for each flagged lanelet (BUG-05).
    # The trusted prior is the source of truth: restore modified/deleted
    # gated relation tags, restore lost members, remove unauthorized added
    # members, and collect restored member refs so their referenced geometry
    # can be repaired next.
    rels = {int(r.get("id")): r for r in croot.findall("relation")}
    members_removed, roles_seen = 0, set()
    members_restored = 0
    members_restored_refs = []   # (type, ref, role) for geometry restoration
    tag_repairs = 0
    for sid in targets:
        lid = int(sid.split(":")[1])
        crel = rels.get(lid)
        prel = prior_rels.get(lid)
        # Only restore lanelets present in BOTH prior and candidate. Lanelets
        # added in the candidate (prel is None) or removed from it (crel is
        # None) are out of scope for BUG-05 restoration; re-verification keeps
        # them refused.
        if crel is None or prel is None:
            continue
        # ---- restore gated relation tags -------------------------------
        tag_repairs += len(restore_relation_tags(crel, prel))
        # ---- restore relation members ----------------------------------
        nr, rd, ar, ad = restore_members(crel, prel)
        members_removed += nr
        roles_seen.update(r[2] for r in rd)
        members_restored += ar
        members_restored_refs.extend(ad)

    # Restore referenced geometry (ways/nodes) required by restored members.
    # If a required dependency cannot be restored from the prior either, the
    # repair is incomplete -- record it but continue so the gate can fail
    # closed on re-verification rather than admitting a broken map.
    ref_ways_restored, ref_nodes_restored, ref_missing = restore_referenced_geometry(
        croot, proot, pnodes, members_restored_refs)

    # Remove geometry the attack ADDED that nothing references any more.
    # Leaving it was an assumption that Lanelet2 ignores unreferenced ways.
    # Measured 2026-10-26: the repaired centreline map raised MRM
    # comfortable_stop where the clean map did not, with 89 orphaned nodes and
    # 8 orphaned ways the only difference. A repaired map should be as close to
    # the trusted prior as the evidence allows.
    #
    # Only ids ABSENT from the prior are eligible, so this can never delete
    # original geometry -- it restores toward the prior, like everything else
    # in this policy.
    pway_ids = {int(w.get("id")) for w in proot.findall("way")}
    pnode_ids = set(pnodes)
    ways_removed = nodes_removed = 0

    referenced_ways = set()
    for r in croot.findall("relation"):
        for m in r.findall("member"):
            if m.get("type") == "way":
                try:
                    referenced_ways.add(int(m.get("ref")))
                except (TypeError, ValueError):
                    pass
    for w in list(croot.findall("way")):
        wid = int(w.get("id"))
        if wid in pway_ids or wid in referenced_ways:
            continue
        croot.remove(w)
        ways_removed += 1

    referenced_nodes = set()
    for w in croot.findall("way"):
        for nd in w.findall("nd"):
            try:
                referenced_nodes.add(int(nd.get("ref")))
            except (TypeError, ValueError):
                pass
    for r in croot.findall("relation"):
        for m in r.findall("member"):
            if m.get("type") == "node":
                try:
                    referenced_nodes.add(int(m.get("ref")))
                except (TypeError, ValueError):
                    pass
    for n in list(croot.findall("node")):
        nid = int(n.get("id"))
        if nid in pnode_ids or nid in referenced_nodes:
            continue
        croot.remove(n)
        nodes_removed += 1

    ctree.write(a.out, encoding="utf-8", xml_declaration=True)
    print(f"\nwrote {a.out}")
    print(f"  boundary nodes considered : {len(todo)}")
    print(f"  restored from prior       : {restored}")
    print(f"  already matched prior     : {unchanged}")
    print(f"  absent from prior         : {missing}")
    print(f"  relation tags restored    : {tag_repairs}")
    print(f"  injected members removed  : {members_removed}"
          f"{' (' + ', '.join(sorted(roles_seen)) + ')' if roles_seen else ''}")
    print(f"  lost members restored     : {members_restored}")
    print(f"  referenced ways restored  : {ref_ways_restored}")
    print(f"  referenced nodes restored : {ref_nodes_restored}")
    if ref_missing:
        print(f"  referenced geometry MISSING from prior: {ref_missing}")
    print(f"  orphaned ways removed     : {ways_removed}")
    print(f"  orphaned nodes removed    : {nodes_removed}")

    if a.report:
        Path(a.report).write_text(json.dumps({
            "policy": "restore_from_prior", "prior": a.prior,
            "candidate": a.candidate, "output": a.out, "diff": a.diff,
            "lanelets_restored": targets,
            "invocation_id": a.invocation_id,
            "_invocation_id": a.invocation_id,
            "nodes_considered": len(todo), "nodes_restored": restored,
            "nodes_unchanged": unchanged, "nodes_missing": missing,
            "tags_restored": tag_repairs,
            "members_removed": members_removed,
            "members_restored": members_restored,
            "referenced_ways_restored": ref_ways_restored,
            "referenced_nodes_restored": ref_nodes_restored,
            "referenced_geometry_missing": ref_missing,
            "orphan_ways_removed": ways_removed,
            "orphan_nodes_removed": nodes_removed,
            "roles_removed": sorted(roles_seen),
        }, indent=2))
        print(f"  report                    : {a.report}")

    # Fail closed: if a restored member's referenced geometry cannot be
    # reconstructed from the trusted prior, the repaired map would contain
    # dangling references. Refuse admission rather than admitting an
    # incomplete repair (the gate treats non-zero repair exit as ERROR).
    if ref_missing:
        print(f"ERROR: incomplete repair -- referenced geometry absent from prior: "
              f"{ref_missing}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
