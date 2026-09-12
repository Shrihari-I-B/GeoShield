"""
GeoShield -- Tests for attack injector (attack_injector.py).
"""

import random
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
import pytest

from road_segment import RoadSegment
from attack_injector import (
    walk,
    width_ramp,
    width_step,
    speed_spoof,
    oneway_flip,
    connectivity_break,
    write_tampered_map,
    detect_shared_seams,
    AttackResult,
    TamperRecord,
)


def _make_chain(n: int = 5, base_width: float = 3.0):
    segs = []
    for i in range(n):
        sid = f"lanelet:{100 + i}"
        succ = [f"lanelet:{100 + i + 1}"] if i < n - 1 else []
        pred = [f"lanelet:{100 + i - 1}"] if i > 0 else []
        segs.append(
            RoadSegment(
                segment_id=sid,
                source="lanelet2",
                width_m=base_width,
                speed_limit_kph=50.0,
                oneway=True,
                highway_class="road",
                predecessors=pred,
                successors=succ,
                geometry=[(35.0, 139.0 + 0.001 * i), (35.0, 139.0 + 0.001 * (i + 1))],
            )
        )
    return segs


class TestAttackGeneration:
    def test_walk_follows_successors(self):
        segs = _make_chain(5)
        rng = random.Random(42)
        path = walk(segs, "lanelet:100", 3, rng)
        assert path == ["lanelet:100", "lanelet:101", "lanelet:102"]

    def test_width_ramp_creates_monotonic_sequence(self):
        segs = _make_chain(6)
        rng = random.Random(42)
        target_ids = [s.segment_id for s in segs[:4]]
        res = width_ramp(segs, rng, total_gain=2.0, target=target_ids)
        assert len(res.labels) == 4
        # Check that tampered widths increase monotonically along the ramp
        sorted_labels = sorted(res.labels.values(), key=lambda r: r.ramp_position or 0)
        widths = [r.tampered for r in sorted_labels]
        assert all(widths[i] <= widths[i + 1] for i in range(len(widths) - 1))

    def test_width_step_single_tamper(self):
        segs = _make_chain(3)
        rng = random.Random(42)
        res = width_step(segs, rng, delta=1.5)
        assert len(res.labels) == 1
        rec = next(iter(res.labels.values()))
        assert rec.field_changed == "width_m"
        assert rec.tampered == pytest.approx(rec.original + 1.5)

    def test_speed_spoof_changes_speed_limit(self):
        segs = _make_chain(3)
        rng = random.Random(42)
        res = speed_spoof(segs, rng)
        assert len(res.labels) == 1
        rec = next(iter(res.labels.values()))
        assert rec.field_changed == "speed_limit_kph"
        assert rec.tampered != rec.original

    def test_oneway_flip_inverts_direction(self):
        segs = _make_chain(3)
        rng = random.Random(42)
        res = oneway_flip(segs, rng)
        assert len(res.labels) == 1
        rec = next(iter(res.labels.values()))
        assert rec.field_changed == "oneway"
        assert rec.tampered is False
        assert rec.original is True

    def test_connectivity_break_drops_successor(self):
        segs = _make_chain(3)
        rng = random.Random(42)
        res = connectivity_break(segs, rng)
        assert len(res.labels) == 1
        rec = next(iter(res.labels.values()))
        assert rec.field_changed == "successors"
        assert len(rec.tampered) < len(rec.original)


class TestWriteTamperedMap:
    MINI_OSM = """<?xml version="1.0" encoding="UTF-8"?>
<osm version="0.6">
  <node id="1" lat="35.68" lon="139.76">
    <tag k="local_x" v="0.0" />
    <tag k="local_y" v="0.0" />
  </node>
  <node id="2" lat="35.68" lon="139.77">
    <tag k="local_x" v="10.0" />
    <tag k="local_y" v="0.0" />
  </node>
  <node id="3" lat="35.69" lon="139.76">
    <tag k="local_x" v="0.0" />
    <tag k="local_y" v="3.0" />
  </node>
  <node id="4" lat="35.69" lon="139.77">
    <tag k="local_x" v="10.0" />
    <tag k="local_y" v="3.0" />
  </node>
  <way id="10">
    <nd ref="3" />
    <nd ref="4" />
    <tag k="type" v="line_thin" />
  </way>
  <way id="11">
    <nd ref="1" />
    <nd ref="2" />
    <tag k="type" v="line_thin" />
  </way>
  <relation id="100">
    <tag k="type" v="lanelet" />
    <tag k="subtype" v="road" />
    <tag k="speed_limit" v="50" />
    <tag k="one_way" v="yes" />
    <member type="way" ref="10" role="left" />
    <member type="way" ref="11" role="right" />
  </relation>
</osm>"""

    def test_write_tag_tampering(self):
        """Verify speed and oneway tag edits in XML."""
        with tempfile.TemporaryDirectory() as tmpdir:
            p_in = Path(tmpdir) / "in.osm"
            p_out = Path(tmpdir) / "out.osm"
            p_in.write_text(self.MINI_OSM)

            labels = {
                "lanelet:100": TamperRecord(
                    segment_id="lanelet:100",
                    attack_type="speed_spoof",
                    field_changed="speed_limit_kph",
                    original=50.0,
                    tampered=80.0,
                    severity=1.0,
                )
            }
            res = AttackResult(segments=[], labels=labels)
            stats = write_tampered_map(str(p_in), str(p_out), res)

            assert stats["tags"] == 1
            assert stats["skipped"] == 0

            root = ET.parse(p_out).getroot()
            rel = root.find(".//relation[@id='100']")
            tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
            assert tags["speed_limit"] == "80.0"

    def test_write_width_geometry_shift(self):
        """Verify left boundary way nodes are physically shifted."""
        with tempfile.TemporaryDirectory() as tmpdir:
            p_in = Path(tmpdir) / "in.osm"
            p_out = Path(tmpdir) / "out.osm"
            p_in.write_text(self.MINI_OSM)

            labels = {
                "lanelet:100": TamperRecord(
                    segment_id="lanelet:100",
                    attack_type="width_step",
                    field_changed="width_m",
                    original=3.0,
                    tampered=4.5,
                    severity=1.0,
                )
            }
            res = AttackResult(segments=[], labels=labels)
            stats = write_tampered_map(str(p_in), str(p_out), res)

            assert stats["geometry"] == 1

            # Check that node 3 or 4 (left way 10) was displaced along normal
            root = ET.parse(p_out).getroot()
            node3 = root.find(".//node[@id='3']")
            tags = {t.get("k"): t.get("v") for t in node3.findall("tag")}
            # Left boundary is from (0,3) to (10,3). Normal points along +y:
            # tx=10, ty=0 -> nx=-0/10=0, ny=10/10=1.
            # Shift = +1.5m -> y becomes 3.0 + 1.5 = 4.5
            assert float(tags["local_y"]) == pytest.approx(4.5, abs=1e-3)


class TestSeamAndFixtureRegression:
    def test_route_g3_shared_seams_regression(self):
        """
        route_g3.0 targets must report exactly 7 doubly-claimed left-boundary
        nodes out of 178 unique left-boundary nodes.
        Guards against regression where seam nodes between consecutive lanelets
        are doubly-displaced during width ramp injection.
        """
        map_path = Path.home() / "autoware_map/nishishinjuku_autoware_map/lanelet2_map.osm"
        labels_path = Path("data/route_g3.0_labels.json")
        if not map_path.exists() or not labels_path.exists():
            pytest.skip("Clean map or route_g3.0 fixture not found")

        import json
        from collections import Counter

        labels = json.loads(labels_path.read_text())
        target_ids = list(labels.keys())
        assert len(target_ids) == 8

        root = ET.parse(str(map_path)).getroot()
        ways = {int(w.get("id")): [int(nd.get("ref")) for nd in w.findall("nd")]
                for w in root.findall("way")}
        rels = {int(r.get("id")): r for r in root.findall("relation")}

        all_left = []
        for tid_str in target_ids:
            tid = int(tid_str.split(":")[1])
            rel = rels[tid]
            for m in rel.findall("member"):
                if m.get("role") == "left":
                    wid = int(m.get("ref"))
                    all_left.extend(ways[wid])

        counts = Counter(all_left)
        assert len(counts) == 178, f"Expected 178 unique left-boundary nodes, got {len(counts)}"

        seams = detect_shared_seams(str(map_path), target_ids)
        assert len(seams) == 7, f"Expected exactly 7 doubly-claimed seam nodes, got {len(seams)}"
        for nid, count in seams.items():
            assert count == 2, f"Expected node {nid} to have count 2, got {count}"

    def test_real_osm_pipeline_load(self):
        """Touch the real pipeline with actual .osm fixtures."""
        clean_path = Path.home() / "autoware_map/nishishinjuku_autoware_map/lanelet2_map.osm"
        tampered_path = Path("data/route_g3.0.osm")
        if not clean_path.exists() or not tampered_path.exists():
            pytest.skip("Real map fixtures not available")

        from lanelet2_adapter import load as load_map
        clean_segs = load_map(str(clean_path))
        tampered_segs = load_map(str(tampered_path))

        assert len(clean_segs) == 979
        assert len(tampered_segs) == 979

        by_clean = {s.segment_id: s for s in clean_segs}
        by_tampered = {s.segment_id: s for s in tampered_segs}

        # Targeted lanelet 3002013 should be noticeably wider in the real map
        assert by_tampered["lanelet:3002013"].width_m > by_clean["lanelet:3002013"].width_m + 3.0

