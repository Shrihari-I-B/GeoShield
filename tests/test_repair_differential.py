"""
GeoShield -- BUG-05: differential repair regression tests.

Self-contained: synthetic Lanelet2 .osm XML and tmp_path only. No dependency on
data/, bags/, results/, or the external Nishi-Shinjuku dataset.

The end-to-end tests run the real `differential_verify.py` and
`repair_differential.py` subprocesses and inspect actual XML output; the unit
tests call the module's real restoration helpers on parsed XML trees.
"""
from __future__ import annotations

import json
import subprocess
import sys
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

# Import the real repair helpers (not mocked).
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import repair_differential as rd  # noqa: E402

REPO = Path(__file__).resolve().parent.parent
DV = REPO / "differential_verify.py"
RP = REPO / "repair_differential.py"


# ---------------------------------------------------------------------------
# synthetic Lanelet2 XML builders
# ---------------------------------------------------------------------------

def _node(nid, lat, lon, x, y):
    return (
        f'  <node id="{nid}" lat="{lat}" lon="{lon}">\n'
        f'    <tag k="local_x" v="{x}"/>\n'
        f'    <tag k="local_y" v="{y}"/>\n'
        f'  </node>'
    )


def _way(wid, refs):
    nds = "\n".join(f'    <nd ref="{r}"/>' for r in refs)
    return f'  <way id="{wid}">\n{nds}\n  </way>'


def _lanelet(lid, tags, members):
    """members: list of (ref, role) tuples, emitted in order."""
    tag_lines = "\n".join(f'    <tag k="{k}" v="{v}"/>' for k, v in tags.items())
    member_lines = "\n".join(f'    <member type="way" ref="{w}" role="{r}"/>'
                             for w, r in members)
    return (
        f'  <relation id="{lid}">\n'
        f'    <tag k="type" v="lanelet"/>\n'
        f'{tag_lines}\n'
        f'{member_lines}\n'
        f'  </relation>'
    )


def osm_map(nodes, ways, relations):
    body = "\n".join(nodes + ways + relations)
    return f'<?xml version="1.0" encoding="UTF-8"?>\n<osm version="0.6">\n{body}\n</osm>\n'


# Shared geometry for a 3.0 m wide lanelet (metric coords present).
# left way 11: nodes 1,2 at x=0; right way 12: nodes 3,4 at x=3.
_BASE_NODES = [
    _node(1, "35.6900", "139.6900", "0", "0"),
    _node(2, "35.6910", "139.6900", "0", "10"),
    _node(3, "35.6900", "139.6903", "3", "0"),
    _node(4, "35.6910", "139.6903", "3", "10"),
]
_BASE_WAYS = [_way(11, [1, 2]), _way(12, [3, 4])]
_LR_MEMBERS = [(11, "left"), (12, "right")]


def _base_tags(speed_limit="50", one_way="yes", tunnel=None, bridge=None,
               lanes=None, subtype="road"):
    tags = {"speed_limit": speed_limit, "one_way": one_way, "subtype": subtype}
    if tunnel is not None:
        tags["tunnel"] = tunnel
    if bridge is not None:
        tags["bridge"] = bridge
    if lanes is not None:
        tags["lanes"] = str(lanes)
    return tags


def clean_map(**kw):
    """A clean one-lanelet map (left + right boundaries, no centerline)."""
    return osm_map(_BASE_NODES, _BASE_WAYS,
                   [_lanelet(100, _base_tags(**kw), _LR_MEMBERS)])


def map_with_centerline():
    """Clean map plus an explicit centerline member (way 13, nodes 5,6)."""
    cl_nodes = [_node(5, "35.6900", "139.69015", "1.5", "0"),
                _node(6, "35.6910", "139.69015", "1.5", "10")]
    cl_ways = [_way(13, [5, 6])]
    members = _LR_MEMBERS + [(13, "centerline")]
    return (osm_map(_BASE_NODES + cl_nodes, _BASE_WAYS + cl_ways,
                    [_lanelet(100, _base_tags(), members)]),
            cl_nodes, cl_ways)


# ---------------------------------------------------------------------------
# subprocess helpers (real differential_verify / repair_differential)
# ---------------------------------------------------------------------------

def _run(cmd, **kw):
    return subprocess.run(cmd, cwd=str(REPO), capture_output=True, text=True, **kw)


def run_dv(prior, candidate, report, invocation_id=None):
    cmd = [sys.executable, str(DV), "--previous", str(prior), "--candidate",
           str(candidate), "--report", str(report)]
    if invocation_id:
        cmd += ["--invocation-id", str(invocation_id)]
    return _run(cmd)


def run_repair(prior, candidate, diff, out, report, invocation_id=None,
               include_suspect=False):
    cmd = [sys.executable, str(RP), "--candidate", str(candidate), "--prior",
           str(prior), "--diff", str(diff), "--out", str(out),
           "--report", str(report)]
    if invocation_id:
        cmd += ["--invocation-id", str(invocation_id)]
    if include_suspect:
        cmd.append("--include-suspect")
    return _run(cmd)


def read_report(path):
    return json.loads(Path(path).read_text())


def verdict_of(rep):
    return rep.get("verdict")


# ---------------------------------------------------------------------------
# A. Tag-only repair
# ---------------------------------------------------------------------------

class TestTagRepair:
    """Restoring gated relation tags from the trusted prior."""

    def _write(self, tmp_path, name, content):
        p = tmp_path / name
        p.write_text(content)
        return p

    def _tag_cycle(self, tmp_path, prior_kw, candidate_tags, expect_admit=True,
                   invocation_id=None):
        prior = self._write(tmp_path, "prior.osm", clean_map(**prior_kw))
        cand = self._write(tmp_path, "candidate.osm",
                           osm_map(_BASE_NODES, _BASE_WAYS,
                                   [_lanelet(100, candidate_tags, _LR_MEMBERS)]))
        diff = tmp_path / "diff.json"
        out = tmp_path / "repaired.osm"
        rep = tmp_path / "repair.json"
        post = tmp_path / "post.json"

        r1 = run_dv(prior, cand, diff, invocation_id=invocation_id)
        assert r1.returncode in (0, 2), r1.stderr
        assert verdict_of(read_report(diff)) != "ACCEPT", read_report(diff)

        r2 = run_repair(prior, cand, diff, out, rep, invocation_id=invocation_id,
                        include_suspect=True)
        assert r2.returncode == 0, r2.stderr

        # Repaired XML must contain the prior's tag values.
        repaired = ET.parse(out).getroot()
        rel = repaired.find(f'relation[@id="100"]')
        assert rel is not None
        rt = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        prior_tags = _base_tags(**prior_kw)
        for k, v in prior_tags.items():
            assert rt.get(k) == v, f"{k}: {rt.get(k)!r} != {v!r}"

        r3 = run_dv(prior, out, post, invocation_id=invocation_id)
        assert r3.returncode == 0, r3.stderr
        verdict = verdict_of(read_report(post))
        if expect_admit:
            assert verdict == "ACCEPT", verdict
        else:
            assert verdict != "ACCEPT", verdict
        return repaired, rt

    def test_speed_limit_restored(self, tmp_path):
        self._tag_cycle(tmp_path, {}, {"speed_limit": "80", "one_way": "yes",
                                       "subtype": "road"})

    def test_speed_limit_deletion_restored(self, tmp_path):
        prior_tags = _base_tags()
        cand_tags = dict(prior_tags)
        del cand_tags["speed_limit"]
        self._tag_cycle(tmp_path, {}, cand_tags)

    def test_one_way_tampering_restored(self, tmp_path):
        self._tag_cycle(tmp_path, {}, {"speed_limit": "50", "one_way": "no",
                                       "subtype": "road"})

    def test_one_way_deletion_restored(self, tmp_path):
        prior_tags = _base_tags()
        cand_tags = dict(prior_tags)
        del cand_tags["one_way"]
        self._tag_cycle(tmp_path, {}, cand_tags)

    def test_tunnel_restored(self, tmp_path):
        self._tag_cycle(tmp_path, {"tunnel": "yes"},
                        {"speed_limit": "50", "one_way": "yes", "tunnel": "no",
                         "subtype": "road"})

    def test_bridge_restored(self, tmp_path):
        self._tag_cycle(tmp_path, {"bridge": "yes"},
                        {"speed_limit": "50", "one_way": "yes", "bridge": "no",
                         "subtype": "road"})

    def test_lanes_restored(self, tmp_path):
        self._tag_cycle(tmp_path, {"lanes": "2"},
                        {"speed_limit": "50", "one_way": "yes", "lanes": "1",
                         "subtype": "road"})

    def test_highway_class_restored(self, tmp_path):
        self._tag_cycle(tmp_path, {"subtype": "primary"},
                        {"speed_limit": "50", "one_way": "yes", "subtype": "residential"})

    def test_unrelated_tags_preserved(self, tmp_path):
        """Unrelated relation tags survive repair untouched."""
        prior = self._write(tmp_path, "prior.osm", clean_map())
        cand_tags = dict(_base_tags())
        cand_tags["speed_limit"] = "80"
        cand_tags["custom_note"] = "keep-me"
        cand = self._write(tmp_path, "candidate.osm",
                           osm_map(_BASE_NODES, _BASE_WAYS,
                                   [_lanelet(100, cand_tags, _LR_MEMBERS)]))
        diff = tmp_path / "diff.json"
        out = tmp_path / "repaired.osm"
        rep = tmp_path / "repair.json"
        post = tmp_path / "post.json"

        run_dv(prior, cand, diff)
        run_repair(prior, cand, diff, out, rep)
        run_dv(prior, out, post)
        assert verdict_of(read_report(post)) == "ACCEPT"

        repaired = ET.parse(out).getroot()
        rel = repaired.find('relation[@id="100"]')
        rt = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        assert rt["custom_note"] == "keep-me"
        assert rt["speed_limit"] == "50"

    def test_alias_speed_limit_consistency(self, tmp_path):
        """A value moved between speed_limit/maxspeed aliases is repaired."""
        prior = self._write(tmp_path, "prior.osm", clean_map())
        cand_tags = dict(_base_tags())
        del cand_tags["speed_limit"]
        cand_tags["maxspeed"] = "99"
        cand = self._write(tmp_path, "candidate.osm",
                           osm_map(_BASE_NODES, _BASE_WAYS,
                                   [_lanelet(100, cand_tags, _LR_MEMBERS)]))
        diff = tmp_path / "diff.json"
        out = tmp_path / "repaired.osm"
        rep = tmp_path / "repair.json"
        post = tmp_path / "post.json"

        run_dv(prior, cand, diff)
        assert verdict_of(read_report(diff)) != "ACCEPT", read_report(diff)
        run_repair(prior, cand, diff, out, rep, include_suspect=True)
        run_dv(prior, out, post)
        assert verdict_of(read_report(post)) == "ACCEPT"

        repaired = ET.parse(out).getroot()
        rel = repaired.find('relation[@id="100"]')
        rt = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        assert rt["speed_limit"] == "50"
        assert "maxspeed" not in rt


# ---------------------------------------------------------------------------
# B. Relation-member repair
# ---------------------------------------------------------------------------

class TestMemberRepair:
    """Restoring lost relation members and removing unauthorized ones."""

    def _write(self, tmp_path, name, content):
        p = tmp_path / name
        p.write_text(content)
        return p

    def _member_cycle(self, tmp_path, prior_fn, cand_fn, expect_admit=True,
                      invocation_id=None):
        prior = self._write(tmp_path, "prior.osm", prior_fn())
        cand = self._write(tmp_path, "candidate.osm", cand_fn())
        diff = tmp_path / "diff.json"
        out = tmp_path / "repaired.osm"
        rep = tmp_path / "repair.json"
        post = tmp_path / "post.json"

        r1 = run_dv(prior, cand, diff, invocation_id=invocation_id)
        assert r1.returncode == 2, r1.stderr
        assert verdict_of(read_report(diff)) == "REJECT"

        r2 = run_repair(prior, cand, diff, out, rep, invocation_id=invocation_id,
                        include_suspect=True)
        assert r2.returncode == 0, r2.stderr

        r3 = run_dv(prior, out, post, invocation_id=invocation_id)
        assert r3.returncode == 0, r3.stderr
        verdict = verdict_of(read_report(post))
        if expect_admit:
            assert verdict == "ACCEPT", verdict
        else:
            assert verdict != "ACCEPT", verdict
        return out

    def test_added_centerline_removed(self, tmp_path):
        """Candidate adds an explicit centerline member; repair removes it."""
        prior = clean_map()
        cand_map, _, _ = map_with_centerline()
        self._member_cycle(tmp_path, lambda: prior, lambda: cand_map)

    def test_lost_centerline_restored(self, tmp_path):
        """Prior has centerline; candidate lost member+geometry; repair restores."""
        cand_map, cl_nodes, cl_ways = map_with_centerline()
        # Candidate drops the centerline member and its way/nodes entirely.
        cand = osm_map(_BASE_NODES, _BASE_WAYS,
                       [_lanelet(100, _base_tags(), _LR_MEMBERS)])
        prior_fn = lambda: osm_map(_BASE_NODES + cl_nodes, _BASE_WAYS + cl_ways,
                                   [_lanelet(100, _base_tags(),
                                            _LR_MEMBERS + [(13, "centerline")])])
        self._member_cycle(tmp_path, prior_fn, lambda: cand)

    def test_lost_left_boundary_restored(self, tmp_path):
        """Lost left-boundary member is restored from prior."""
        prior = clean_map()
        # Candidate keeps only the right member; left member lost.
        cand = osm_map(_BASE_NODES, _BASE_WAYS,
                       [_lanelet(100, _base_tags(), [(12, "right")])])
        self._member_cycle(tmp_path, lambda: prior, lambda: cand)

    def test_lost_right_boundary_restored(self, tmp_path):
        """Lost right-boundary member is restored from prior."""
        prior = clean_map()
        cand = osm_map(_BASE_NODES, _BASE_WAYS,
                       [_lanelet(100, _base_tags(), [(11, "left")])])
        self._member_cycle(tmp_path, lambda: prior, lambda: cand)

    def test_multiple_lost_members_restored(self, tmp_path):
        """Prior has left+right+centerline; candidate lost all but right."""
        cand_map, cl_nodes, cl_ways = map_with_centerline()
        prior = osm_map(_BASE_NODES + cl_nodes, _BASE_WAYS + cl_ways,
                        [_lanelet(100, _base_tags(),
                                 _LR_MEMBERS + [(13, "centerline")])])
        cand = osm_map(_BASE_NODES, _BASE_WAYS,
                       [_lanelet(100, _base_tags(), [(12, "right")])])
        self._member_cycle(tmp_path, lambda: prior, lambda: cand)

    def test_multiple_added_members_removed(self, tmp_path):
        """Candidate adds two extra members; repair removes both."""
        prior = clean_map()
        extra_nodes = [_node(7, "35.6900", "139.6905", "5", "0"),
                       _node(8, "35.6910", "139.6905", "5", "10")]
        extra_ways = [_way(14, [7, 8]), _way(15, [7, 8])]
        cand = osm_map(_BASE_NODES + extra_nodes, _BASE_WAYS + extra_ways,
                       [_lanelet(100, _base_tags(),
                                 _LR_MEMBERS + [(14, "centerline"), (15, "left")])])
        self._member_cycle(tmp_path, lambda: prior, lambda: cand)

    def test_member_type_ref_role_preserved(self, tmp_path):
        """Restored member keeps original type, ref and role."""
        cand_map, cl_nodes, cl_ways = map_with_centerline()
        prior = osm_map(_BASE_NODES + cl_nodes, _BASE_WAYS + cl_ways,
                        [_lanelet(100, _base_tags(),
                                 _LR_MEMBERS + [(13, "centerline")])])
        cand = osm_map(_BASE_NODES, _BASE_WAYS,
                       [_lanelet(100, _base_tags(), _LR_MEMBERS)])
        out = self._member_cycle(tmp_path, lambda: prior, lambda: cand)
        repaired = ET.parse(out).getroot()
        rel = repaired.find(f'relation[@id="100"]')
        members = {(m.get("type"), m.get("ref"), m.get("role"))
                   for m in rel.findall("member")}
        assert ("way", "13", "centerline") in members


# ---------------------------------------------------------------------------
# C. Referenced geometry
# ---------------------------------------------------------------------------

class TestReferencedGeometry:
    """Restoring ways/nodes required by restored relation members."""

    def _write(self, tmp_path, name, content):
        p = tmp_path / name
        p.write_text(content)
        return p

    def test_lost_centerline_way_and_nodes_restored(self, tmp_path):
        """Candidate lost centerline member + way + nodes; all restored from prior."""
        cand_map, cl_nodes, cl_ways = map_with_centerline()
        prior = self._write(tmp_path, "prior.osm",
                            osm_map(_BASE_NODES + cl_nodes, _BASE_WAYS + cl_ways,
                                    [_lanelet(100, _base_tags(),
                                             _LR_MEMBERS + [(13, "centerline")])]))
        cand = self._write(tmp_path, "candidate.osm",
                           osm_map(_BASE_NODES, _BASE_WAYS,
                                   [_lanelet(100, _base_tags(), _LR_MEMBERS)]))
        diff = tmp_path / "diff.json"
        out = tmp_path / "repaired.osm"
        rep = tmp_path / "repair.json"
        post = tmp_path / "post.json"

        run_dv(prior, cand, diff)
        assert verdict_of(read_report(diff)) != "ACCEPT"
        run_repair(prior, cand, diff, out, rep, include_suspect=True)
        run_dv(prior, out, post)
        assert verdict_of(read_report(post)) == "ACCEPT"

        repaired = ET.parse(out).getroot()
        # member restored
        rel = repaired.find(f'relation[@id="100"]')
        members = {(m.get("type"), m.get("ref"), m.get("role"))
                   for m in rel.findall("member")}
        assert ("way", "13", "centerline") in members
        # way restored
        w = repaired.find(f'way[@id="13"]')
        assert w is not None
        assert [nd.get("ref") for nd in w.findall("nd")] == ["5", "6"]
        # nodes restored with prior coordinates
        n5 = repaired.find(f'node[@id="5"]')
        n6 = repaired.find(f'node[@id="6"]')
        assert n5 is not None and n6 is not None
        assert n5.get("lat") == "35.6900" and n5.get("lon") == "139.69015"
        assert n6.get("lat") == "35.6910" and n6.get("lon") == "139.69015"

    def test_no_dangling_references_after_repair(self, tmp_path):
        """Every member ref in the repaired map resolves to a way/node."""
        cand_map, cl_nodes, cl_ways = map_with_centerline()
        prior = self._write(tmp_path, "prior.osm",
                            osm_map(_BASE_NODES + cl_nodes, _BASE_WAYS + cl_ways,
                                    [_lanelet(100, _base_tags(),
                                             _LR_MEMBERS + [(13, "centerline")])]))
        cand = self._write(tmp_path, "candidate.osm",
                           osm_map(_BASE_NODES, _BASE_WAYS,
                                   [_lanelet(100, _base_tags(), _LR_MEMBERS)]))
        diff = tmp_path / "diff.json"
        out = tmp_path / "repaired.osm"
        rep = tmp_path / "repair.json"

        run_dv(prior, cand, diff)
        run_repair(prior, cand, diff, out, rep, include_suspect=True)

        repaired = ET.parse(out).getroot()
        way_ids = {w.get("id") for w in repaired.findall("way")}
        node_ids = {n.get("id") for n in repaired.findall("node")}
        for rel in repaired.findall("relation"):
            for m in rel.findall("member"):
                if m.get("type") == "way":
                    assert m.get("ref") in way_ids, f"dangling way ref {m.get('ref')}"
                elif m.get("type") == "node":
                    assert m.get("ref") in node_ids, f"dangling node ref {m.get('ref')}"
        for w in repaired.findall("way"):
            for nd in w.findall("nd"):
                assert nd.get("ref") in node_ids, f"dangling nd ref {nd.get('ref')}"

    def test_shared_node_not_duplicated(self, tmp_path):
        """Restoring one lanelet does not duplicate a shared boundary node."""
        # Two lanelets sharing node 2 (end of left way 11 / start of left way 21).
        shared_nodes = [_BASE_NODES[0], _BASE_NODES[1],
                        _node(3, "35.6900", "139.6903", "3", "0"),
                        _node(4, "35.6910", "139.6903", "3", "10"),
                        _node(21, "35.6910", "139.6900", "0", "10"),
                        _node(22, "35.6920", "139.6900", "0", "20")]
        shared_ways = [_way(11, [1, 2]), _way(12, [3, 4]),
                       _way(21, [2, 21])]
        # Lanelet 100 uses left way 21 + right way 12 (shares node 2 with the
        # orphan way 11); way 11 is unreferenced by lanelet 100 in both maps
        # and must be left alone.
        prior = self._write(tmp_path, "prior.osm",
                            osm_map(shared_nodes, shared_ways,
                                    [_lanelet(100, _base_tags(),
                                             [(21, "left"), (12, "right")])]))
        # candidate: lost left member (way 21); only right remains
        cand = self._write(tmp_path, "candidate.osm",
                           osm_map(shared_nodes, [_way(11, [1, 2]), _way(12, [3, 4])],
                                   [_lanelet(100, _base_tags(), [(12, "right")])]))
        diff = tmp_path / "diff.json"
        out = tmp_path / "repaired.osm"
        rep = tmp_path / "repair.json"
        post = tmp_path / "post.json"

        run_dv(prior, cand, diff)
        assert verdict_of(read_report(diff)) != "ACCEPT"
        run_repair(prior, cand, diff, out, rep, include_suspect=True)
        run_dv(prior, out, post)
        assert verdict_of(read_report(post)) == "ACCEPT"

        repaired = ET.parse(out).getroot()
        node2 = repaired.findall(f'node[@id="2"]')
        assert len(node2) == 1, "shared node must not be duplicated"
        rel = repaired.find(f'relation[@id="100"]')
        members = [(m.get("ref"), m.get("role")) for m in rel.findall("member")]
        assert members == [("12", "right"), ("21", "left")], members

    def test_unrelated_lanelet_unchanged(self, tmp_path):
        """An untampered second lanelet is left byte-identical."""
        # Second lanelet 200 with its own geometry, untouched by the attack.
        extra_nodes = [_node(31, "35.6950", "139.6950", "0", "0"),
                       _node(32, "35.6960", "139.6950", "0", "10"),
                       _node(33, "35.6950", "139.6953", "3", "0"),
                       _node(34, "35.6960", "139.6953", "3", "10")]
        extra_ways = [_way(31, [31, 32]), _way(32, [33, 34])]
        def two_lanelets(l100_speed, l200_speed):
            return osm_map(_BASE_NODES + extra_nodes,
                           _BASE_WAYS + extra_ways,
                           [_lanelet(100, _base_tags(speed_limit=l100_speed), _LR_MEMBERS),
                            _lanelet(200, _base_tags(speed_limit=l200_speed),
                                     [(31, "left"), (32, "right")])])

        prior = self._write(tmp_path, "prior.osm", two_lanelets("50", "50"))
        cand = self._write(tmp_path, "candidate.osm", two_lanelets("80", "50"))  # only lanelet 100 tampered; 200 unchanged
        diff = tmp_path / "diff.json"
        out = tmp_path / "repaired.osm"
        rep = tmp_path / "repair.json"
        post = tmp_path / "post.json"

        run_dv(prior, cand, diff)
        assert verdict_of(read_report(diff)) != "ACCEPT"
        run_repair(prior, cand, diff, out, rep, include_suspect=True)
        run_dv(prior, out, post)
        assert verdict_of(read_report(post)) == "ACCEPT"

        repaired = ET.parse(out).getroot()
        rel200 = repaired.find(f'relation[@id="200"]')
        rt = {t.get("k"): t.get("v") for t in rel200.findall("tag")}
        assert rt["speed_limit"] == "50"  # unrelated lanelet unchanged
        rel100 = repaired.find(f'relation[@id="100"]')
        rt100 = {t.get("k"): t.get("v") for t in rel100.findall("tag")}
        assert rt100["speed_limit"] == "50"  # tampered lanelet restored


# ---------------------------------------------------------------------------
# D. Safety and failure behaviour
# ---------------------------------------------------------------------------

class TestSafety:
    """Repair must fail closed on unsafe inputs."""

    def _write(self, tmp_path, name, content):
        p = tmp_path / name
        p.write_text(content)
        return p

    def test_missing_prior(self, tmp_path):
        cand = self._write(tmp_path, "candidate.osm", clean_map())
        diff = tmp_path / "diff.json"
        out = tmp_path / "repaired.osm"
        rep = tmp_path / "repair.json"
        run_dv(tmp_path / "nope.osm", cand, diff)
        r = run_repair(tmp_path / "nope.osm", cand, diff, out, rep)
        assert r.returncode != 0, "repair must fail with missing prior"

    def test_invalid_prior_xml(self, tmp_path):
        cand = self._write(tmp_path, "candidate.osm", clean_map())
        prior = self._write(tmp_path, "prior.osm", "<not>xml<")
        diff = tmp_path / "diff.json"
        out = tmp_path / "repaired.osm"
        rep = tmp_path / "repair.json"
        run_dv(prior, cand, diff)
        r = run_repair(prior, cand, diff, out, rep)
        assert r.returncode != 0, "repair must fail on invalid prior XML"

    def test_invalid_candidate_xml(self, tmp_path):
        prior = self._write(tmp_path, "prior.osm", clean_map())
        cand = self._write(tmp_path, "candidate.osm", "<not>xml<")
        diff = tmp_path / "diff.json"
        out = tmp_path / "repaired.osm"
        rep = tmp_path / "repair.json"
        run_dv(prior, cand, diff)
        r = run_repair(prior, cand, diff, out, rep)
        assert r.returncode != 0, "repair must fail on invalid candidate XML"

    def test_lanelet_addition_unrepairable(self, tmp_path):
        """A lanelet added in the candidate cannot be repaired away by restore."""
        # prior: one lanelet; candidate: two lanelets (added lanelet 200).
        extra_nodes = [_node(31, "35.6950", "139.6950", "0", "0"),
                       _node(32, "35.6960", "139.6950", "0", "10"),
                       _node(33, "35.6950", "139.6953", "3", "0"),
                       _node(34, "35.6960", "139.6953", "3", "10")]
        extra_ways = [_way(31, [31, 32]), _way(32, [33, 34])]
        prior = self._write(tmp_path, "prior.osm", osm_map(_BASE_NODES, _BASE_WAYS,
                            [_lanelet(100, _base_tags(), _LR_MEMBERS)]))
        cand = self._write(tmp_path, "candidate.osm",
                           osm_map(_BASE_NODES + extra_nodes, _BASE_WAYS + extra_ways,
                                   [_lanelet(100, _base_tags(), _LR_MEMBERS),
                                    _lanelet(200, _base_tags(), [(31, "left"), (32, "right")])]))
        diff = tmp_path / "diff.json"
        out = tmp_path / "repaired.osm"
        rep = tmp_path / "repair.json"
        post = tmp_path / "post.json"

        run_dv(prior, cand, diff)
        assert verdict_of(read_report(diff)) == "REJECT"
        run_repair(prior, cand, diff, out, rep, include_suspect=True)
        # Repair does not remove added lanelets (out of scope for BUG-05);
        # re-verification must still refuse.
        run_dv(prior, out, post)
        assert verdict_of(read_report(post)) != "ACCEPT"

    def test_missing_referenced_prior_way_fails(self, tmp_path):
        """Restoring a member whose way is absent from the prior fails closed."""
        # prior: lanelet references centerline way 13, but way 13 absent.
        prior = self._write(tmp_path, "prior.osm",
                            osm_map(_BASE_NODES, _BASE_WAYS,
                                    [_lanelet(100, _base_tags(),
                                             _LR_MEMBERS + [(13, "centerline")])]))
        # candidate: lost the centerline member.
        cand = self._write(tmp_path, "candidate.osm",
                           osm_map(_BASE_NODES, _BASE_WAYS,
                                   [_lanelet(100, _base_tags(), _LR_MEMBERS)]))
        diff = tmp_path / "diff.json"
        out = tmp_path / "repaired.osm"
        rep = tmp_path / "repair.json"

        run_dv(prior, cand, diff)
        assert verdict_of(read_report(diff)) != "ACCEPT"
        r = run_repair(prior, cand, diff, out, rep, include_suspect=True)
        assert r.returncode != 0, "repair must fail when prior lacks referenced way"

    def test_unsupported_element_reference(self, tmp_path):
        """A member referencing a missing node is removed (actively handled)."""
        prior = self._write(tmp_path, "prior.osm", clean_map())
        cand = self._write(tmp_path, "candidate.osm",
                           osm_map(_BASE_NODES, _BASE_WAYS,
                                   [_lanelet(100, _base_tags(),
                                            _LR_MEMBERS + [("999", "node")])]))
        diff = tmp_path / "diff.json"
        out = tmp_path / "repaired.osm"
        rep = tmp_path / "repair.json"
        post = tmp_path / "post.json"
        run_dv(prior, cand, diff)
        assert verdict_of(read_report(diff)) != "ACCEPT"
        # Repair removes the unauthorized bogus member (it is not in the prior).
        r = run_repair(prior, cand, diff, out, rep, include_suspect=True)
        assert r.returncode == 0, r.stderr
        run_dv(prior, out, post)
        assert verdict_of(read_report(post)) == "ACCEPT"
        repaired = ET.parse(out).getroot()
        rel = repaired.find(f'relation[@id="100"]')
        members = {(m.get("type"), m.get("ref"), m.get("role"))
                   for m in rel.findall("member")}
        assert ("node", "999", "node") not in members


# ---------------------------------------------------------------------------
# E. End-to-end re-verification with invocation-ID
# ---------------------------------------------------------------------------

class TestEndToEnd:
    """Full gate cycle: detect → repair → re-verify → admit only on ACCEPT."""

    def _write(self, tmp_path, name, content):
        p = tmp_path / name
        p.write_text(content)
        return p

    def test_full_cycle_admit_on_repaired_accept(self, tmp_path):
        invocation_id = str(uuid.uuid4())
        prior = self._write(tmp_path, "prior.osm", clean_map())
        cand_tags = dict(_base_tags()); cand_tags["speed_limit"] = "80"
        cand = self._write(tmp_path, "candidate.osm",
                           osm_map(_BASE_NODES, _BASE_WAYS,
                                   [_lanelet(100, cand_tags, _LR_MEMBERS)]))
        diff = tmp_path / "diff.json"
        out = tmp_path / "repaired.osm"
        rep = tmp_path / "repair.json"
        post = tmp_path / "post.json"

        r1 = run_dv(prior, cand, diff, invocation_id=invocation_id)
        assert verdict_of(read_report(diff)) != "ACCEPT"
        assert read_report(diff).get("_invocation_id") == invocation_id

        r2 = run_repair(prior, cand, diff, out, rep,
                        invocation_id=invocation_id, include_suspect=True)
        assert r2.returncode == 0, r2.stderr
        rrep = read_report(rep)
        assert rrep.get("_invocation_id") == invocation_id
        assert rrep.get("invocation_id") == invocation_id

        r3 = run_dv(prior, out, post, invocation_id=invocation_id)
        assert r3.returncode == 0, r3.stderr
        assert verdict_of(read_report(post)) == "ACCEPT"
        assert read_report(post).get("_invocation_id") == invocation_id

    def test_repair_report_records_restorations(self, tmp_path):
        invocation_id = str(uuid.uuid4())
        prior = self._write(tmp_path, "prior.osm", clean_map())
        cand = self._write(tmp_path, "candidate.osm",
                           osm_map(_BASE_NODES, _BASE_WAYS,
                                   [_lanelet(100, {"speed_limit": "80",
                                                   "one_way": "yes",
                                                   "subtype": "road"},
                                           _LR_MEMBERS)]))
        diff = tmp_path / "diff.json"
        out = tmp_path / "repaired.osm"
        rep = tmp_path / "repair.json"
        run_dv(prior, cand, diff, invocation_id=invocation_id)
        run_repair(prior, cand, diff, out, rep,
                   invocation_id=invocation_id, include_suspect=True)
        rrep = read_report(rep)
        assert rrep["_invocation_id"] == invocation_id
        assert rrep.get("tags_restored", 0) >= 1  # speed_limit restored
        assert rrep.get("members_removed", 0) >= 0
