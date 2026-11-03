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

    # Two lanelets: 100 -> 101 (successor), for connectivity_break test.
    # Lanelet 100: left way 10 (nodes 3,4), right way 11 (nodes 1,2)
    # Lanelet 101: left way 20 (nodes 4,6), right way 21 (nodes 2,7)
    # Shared boundaries: way 10 end (node 4) = way 20 start (node 4)
    #                   way 11 end (node 2) = way 21 start (node 2)
    MINI_OSM_SUCCESSORS = """<?xml version="1.0" encoding="UTF-8"?>
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
  <node id="5" lat="35.70" lon="139.76">
    <tag k="local_x" v="0.0" />
    <tag k="local_y" v="6.0" />
  </node>
  <node id="6" lat="35.70" lon="139.77">
    <tag k="local_x" v="10.0" />
    <tag k="local_y" v="6.0" />
  </node>
  <node id="7" lat="35.71" lon="139.76">
    <tag k="local_x" v="0.0" />
    <tag k="local_y" v="9.0" />
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
  <way id="20">
    <nd ref="4" />
    <nd ref="6" />
    <tag k="type" v="line_thin" />
  </way>
  <way id="21">
    <nd ref="2" />
    <nd ref="7" />
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
  <relation id="101">
    <tag k="type" v="lanelet" />
    <tag k="subtype" v="road" />
    <tag k="speed_limit" v="50" />
    <tag k="one_way" v="yes" />
    <member type="way" ref="20" role="left" />
    <member type="way" ref="21" role="right" />
  </relation>
</osm>"""

    # WIDTH_TEST_OSM: 13 lanelets × 6 nodes = 78 nodes with valid
    # local_x/local_y + lat/lon (exceeds the 50-point calibration threshold).
    # Each way has 3 nodes (2 segments) so the middle node gets displaced
    # (taper goes to zero at endpoints, max in middle).
    # Each lanelet has its own left/right boundary ways (no sharing) so every
    # node is unique and contributes to the calibration count.
    WIDTH_TEST_OSM = """<?xml version="1.0" encoding="UTF-8"?>
<osm version="0.6">
""" + "\n".join([
        # 13 lanelets × 6 nodes each = 78 nodes (ids 1-78)
        # Each lanelet: left way has nodes (6i-5, 6i-4, 6i-3), right way has (6i-2, 6i-1, 6i)
        "  <node id=\"{}\" lat=\"{}\" lon=\"{}\">\n    <tag k=\"local_x\" v=\"{}\" />\n    <tag k=\"local_y\" v=\"{}\" />\n  </node>".format(
            n,
            35.68 + (n - 1) * 0.001,
            139.76 + (n - 1) * 0.001,
            (n - 1) * 2.0,
            (n - 1) * 2.0
        )
        for n in range(1, 79)
    ]) + "\n" + "\n".join([
        # 13 left ways (ids 101, 103, 105... = 99+2i), each with 3 nodes
        "  <way id=\"{}\">\n    <nd ref=\"{}\" />\n    <nd ref=\"{}\" />\n    <nd ref=\"{}\" />\n    <tag k=\"type\" v=\"line_thin\" />\n  </way>".format(
            99 + 2*i,
            6*i - 5,  # first node of left way
            6*i - 4,  # middle node of left way (gets displaced)
            6*i - 3   # last node of left way
        )
        for i in range(1, 14)
    ]) + "\n" + "\n".join([
        # 13 right ways (ids 102, 104, 106... = 100+2i), each with 3 nodes
        "  <way id=\"{}\">\n    <nd ref=\"{}\" />\n    <nd ref=\"{}\" />\n    <nd ref=\"{}\" />\n    <tag k=\"type\" v=\"line_thin\" />\n  </way>".format(
            100 + 2*i,
            6*i - 2,  # first node of right way
            6*i - 1,  # middle node
            6*i       # last node
        )
        for i in range(1, 14)
    ]) + "\n" + "\n".join([
        # 13 lanelet relations (ids 100-112)
        "  <relation id=\"{}\">\n    <tag k=\"type\" v=\"lanelet\" />\n    <tag k=\"subtype\" v=\"road\" />\n    <tag k=\"speed_limit\" v=\"50\" />\n    <tag k=\"one_way\" v=\"yes\" />\n    <member type=\"way\" ref=\"{}\" role=\"left\" />\n    <member type=\"way\" ref=\"{}\" role=\"right\" />\n  </relation>".format(
            100 + i - 1,
            99 + 2*i,
            100 + 2*i
        )
        for i in range(1, 14)
    ]) + "\n</osm>\n"

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
            # Use WIDTH_TEST_OSM which has 3-node ways so the middle node
            # gets displaced (taper goes to zero at endpoints, max in middle).
            p_in.write_text(self.WIDTH_TEST_OSM)

            # Lanelet 100 uses left way 101 (nodes 1,2,3). Middle node is 2.
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
            assert stats["tags"] == 0
            assert stats["skipped"] == 0

            # Check that middle node of left way (node 2) was displaced along normal
            root = ET.parse(p_out).getroot()
            # Left way 101: nodes 1->2->3 in local coords (0,0) -> (2,2) -> (4,4)
            # Tangent = (4,4)-(0,0) = (4,4), normal = (-1,1)/sqrt(2)
            # Shift = +1.5m, nseg=2, middle node (i=1) gets f = min(1/2, 1-1/2) = 0.5
            # Displacement = 0.5 * 1.5 * (-1,1)/sqrt(2) = 0.75/sqrt(2) ≈ 0.53
            node2 = root.find(".//node[@id='2']")
            tags = {t.get("k"): t.get("v") for t in node2.findall("tag")}
            import math
            disp = 0.75 / math.sqrt(2)
            expected_x = 2.0 - disp
            expected_y = 2.0 + disp
            assert float(tags["local_x"]) == pytest.approx(expected_x, abs=1e-2)
            assert float(tags["local_y"]) == pytest.approx(expected_y, abs=1e-2)

    # --- New regression tests for BUG-01 calibration fix ---

    def test_write_width_fails_on_insufficient_points(self):
        """Width attack on map with <50 valid points must fail closed."""
        with tempfile.TemporaryDirectory() as tmpdir:
            p_in = Path(tmpdir) / "in.osm"
            p_out = Path(tmpdir) / "out.osm"
            p_in.write_text(self.MINI_OSM)  # only 4 valid calibration points

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

            # Must raise SystemExit (calibration guard)
            with pytest.raises(SystemExit) as exc:
                write_tampered_map(str(p_in), str(p_out), res)
            assert "cannot calibrate" in str(exc.value)

    def test_write_width_succeeds_on_sufficient_points(self):
        """Width attack on map with ≥50 valid points must succeed."""
        with tempfile.TemporaryDirectory() as tmpdir:
            p_in = Path(tmpdir) / "in.osm"
            p_out = Path(tmpdir) / "out.osm"
            p_in.write_text(self.WIDTH_TEST_OSM)  # 78 valid calibration points

            # Lanelet 100 uses left way 101 (nodes 1,2,3). Middle node is 2.
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
            assert stats["tags"] == 0
            assert stats["skipped"] == 0

            # Verify the middle node of left way (node 2) was displaced
            root = ET.parse(p_out).getroot()
            node2 = root.find(".//node[@id='2']")
            tags = {t.get("k"): t.get("v") for t in node2.findall("tag")}
            import math
            disp = 0.75 / math.sqrt(2)
            expected_x = 2.0 - disp
            expected_y = 2.0 + disp
            assert float(tags["local_x"]) == pytest.approx(expected_x, abs=1e-2)
            assert float(tags["local_y"]) == pytest.approx(expected_y, abs=1e-2)

    def test_write_successors_no_calibration(self):
        """Successors attack must not require calibration (clones lat/lon)."""
        with tempfile.TemporaryDirectory() as tmpdir:
            p_in = Path(tmpdir) / "in.osm"
            p_out = Path(tmpdir) / "out.osm"
            # Use fixture with two lanelets where 100 -> 101 is a valid successor
            p_in.write_text(self.MINI_OSM_SUCCESSORS)

            labels = {
                "lanelet:100": TamperRecord(
                    segment_id="lanelet:100",
                    attack_type="connectivity_break",
                    field_changed="successors",
                    original=["lanelet:101"],
                    tampered=[],
                    severity=1.0,
                )
            }
            res = AttackResult(segments=[], labels=labels)

            # Should succeed without calibration (clones lat/lon directly)
            stats = write_tampered_map(str(p_in), str(p_out), res)

            assert stats["geometry"] == 1
            assert stats["tags"] == 0
            assert stats["skipped"] == 0

    def test_write_tag_tampering_no_calibration(self):
        """Tag-only attacks (speed, oneway) must not require calibration."""
        with tempfile.TemporaryDirectory() as tmpdir:
            p_in = Path(tmpdir) / "in.osm"
            p_out = Path(tmpdir) / "out.osm"
            p_in.write_text(self.MINI_OSM)  # only 4 valid points

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

            # Should succeed without calibration
            stats = write_tampered_map(str(p_in), str(p_out), res)

            assert stats["tags"] == 1
            assert stats["geometry"] == 0
            assert stats["skipped"] == 0

            root = ET.parse(p_out).getroot()
            rel = root.find(".//relation[@id='100']")
            tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
            assert tags["speed_limit"] == "80.0"


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

