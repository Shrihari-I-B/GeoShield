"""
GeoShield -- Tests for differential verification (differential_verify.py).
"""

import json
import tempfile
from pathlib import Path
import pytest

from road_segment import RoadSegment
from differential_verify import (
    compare,
    structural_diff,
    find_runs,
    evaluate,
    DiffReport,
    Change,
)


def _make_seg(sid: str, width_m=3.0, speed_kph=50.0, oneway=True, successors=None,
              raw_tags=None, tunnel=None, bridge=None, lanes=None,
              highway_class=None):
    return RoadSegment(
        segment_id=sid,
        source="lanelet2",
        width_m=width_m,
        speed_limit_kph=speed_kph,
        oneway=oneway,
        tunnel=tunnel,
        bridge=bridge,
        lanes=lanes,
        highway_class=highway_class,
        successors=successors or [],
        raw_tags=raw_tags or {},
    )


_ALL_ATTRS = ("width_m", "speed_limit_kph", "oneway", "tunnel", "bridge",
              "lanes", "highway_class")


def _seg_with(sid: str, field_name: str, value, raw_tags=None):
    """Segment with every attribute absent except the one under test.

    This makes "both values absent" distinguishable from "attribute removed".
    """
    kwargs = {name: None for name in _ALL_ATTRS}
    kwargs[field_name] = value
    return RoadSegment(segment_id=sid, source="lanelet2",
                       raw_tags=raw_tags or {}, **kwargs)


class TestDifferentialComparison:
    def test_clean_vs_clean(self):
        """Comparing identical map versions must yield zero changes and ACCEPT."""
        segs_a = [_make_seg(f"l_{i}", width_m=3.2, speed_kph=40.0) for i in range(5)]
        segs_b = [_make_seg(f"l_{i}", width_m=3.2, speed_kph=40.0) for i in range(5)]

        rep = compare(segs_a, segs_b)
        assert rep.n_previous == 5
        assert rep.n_candidate == 5
        assert len(rep.changes) == 0
        assert len(rep.added) == 0
        assert len(rep.removed) == 0
        assert len(rep.rejected()) == 0
        assert len(rep.flagged_ids()) == 0

    def test_width_noise_ignored(self):
        """Width change <= 0.10 m (survey noise) should not produce a change."""
        a = [_make_seg("l1", width_m=3.0)]
        b = [_make_seg("l1", width_m=3.08)]  # delta 0.08m <= 0.10m
        rep = compare(a, b)
        assert len(rep.changes) == 0

    def test_width_suspect_and_reject_thresholds(self):
        """Width change between 5% and 15% is SUSPECT; >= 15% is REJECT."""
        # 10% change: 3.0 -> 3.3m
        a = [_make_seg("l1", width_m=3.0)]
        b = [_make_seg("l1", width_m=3.30)]
        rep_suspect = compare(a, b)
        assert len(rep_suspect.changes) == 1
        assert rep_suspect.changes[0].verdict == "SUSPECT"
        assert rep_suspect.changes[0].field_name == "width_m"

        # 30% change: 3.0 -> 3.9m
        c = [_make_seg("l1", width_m=3.90)]
        rep_reject = compare(a, c)
        assert len(rep_reject.changes) == 1
        assert rep_reject.changes[0].verdict == "REJECT"
        assert len(rep_reject.rejected()) == 1

    def test_speed_limit_thresholds(self):
        """Speed limit changes < 5 km/h ignored; 5-14 km/h SUSPECT; >= 15 km/h REJECT."""
        a = [_make_seg("l1", speed_kph=50.0)]

        # +3 km/h -> ignored
        rep_ignored = compare(a, [_make_seg("l1", speed_kph=53.0)])
        assert len(rep_ignored.changes) == 0

        # +10 km/h -> SUSPECT
        rep_suspect = compare(a, [_make_seg("l1", speed_kph=60.0)])
        assert len(rep_suspect.changes) == 1
        assert rep_suspect.changes[0].verdict == "SUSPECT"

        # +20 km/h -> REJECT
        rep_reject = compare(a, [_make_seg("l1", speed_kph=70.0)])
        assert len(rep_reject.changes) == 1
        assert rep_reject.changes[0].verdict == "REJECT"

    def test_oneway_reversal(self):
        """Direction reversal must trigger immediate REJECT."""
        a = [_make_seg("l1", oneway=True)]
        b = [_make_seg("l1", oneway=False)]
        rep = compare(a, b)
        assert len(rep.changes) == 1
        assert rep.changes[0].verdict == "REJECT"
        assert rep.changes[0].field_name == "oneway"

    def test_connectivity_break(self):
        """Successor dropped or altered must trigger REJECT."""
        a = [_make_seg("l1", successors=["l2", "l3"])]
        b = [_make_seg("l1", successors=["l2"])]
        rep = compare(a, b)
        assert len(rep.changes) == 1
        assert rep.changes[0].verdict == "REJECT"
        assert rep.changes[0].field_name == "successors"


class TestCoordinatedRuns:
    def test_find_runs_groups_monotonic_ramp(self):
        """find_runs must group connected flagged lanelets and detect monotonic trend."""
        # Setup l1 -> l2 -> l3
        segs_cand = [
            _make_seg("l1", width_m=4.0, successors=["l2"]),
            _make_seg("l2", width_m=4.5, successors=["l3"]),
            _make_seg("l3", width_m=5.0, successors=[]),
        ]
        # Previous map was 3.0 m for all
        segs_prev = [
            _make_seg("l1", width_m=3.0, successors=["l2"]),
            _make_seg("l2", width_m=3.0, successors=["l3"]),
            _make_seg("l3", width_m=3.0, successors=[]),
        ]

        rep = compare(segs_prev, segs_cand)
        assert len(rep.flagged_ids()) == 3

        runs = find_runs(rep, segs_cand)
        assert len(runs) == 1
        r = runs[0]
        assert r["lanelets"] == ["l1", "l2", "l3"]
        assert r["length"] == 3
        assert r["monotonic"] is True
        assert r["total_width_change"] == pytest.approx(1.0)


class TestStructuralDiff:
    def test_centerline_injection_detected(self):
        """Centerline injection adds a centerline role where none existed before -> REJECT."""
        xml_clean = """<?xml version="1.0" encoding="UTF-8"?>
<osm version="0.6">
  <relation id="100">
    <tag k="type" v="lanelet" />
    <member type="way" ref="1" role="left" />
    <member type="way" ref="2" role="right" />
  </relation>
</osm>"""

        xml_attack = """<?xml version="1.0" encoding="UTF-8"?>
<osm version="0.6">
  <relation id="100">
    <tag k="type" v="lanelet" />
    <member type="way" ref="1" role="left" />
    <member type="way" ref="2" role="right" />
    <member type="way" ref="3" role="centerline" />
  </relation>
</osm>"""

        with tempfile.TemporaryDirectory() as tmpdir:
            p_clean = Path(tmpdir) / "clean.osm"
            p_attack = Path(tmpdir) / "attack.osm"
            p_clean.write_text(xml_clean)
            p_attack.write_text(xml_attack)

            changes = structural_diff(str(p_clean), str(p_attack))
            assert len(changes) == 1
            ch = changes[0]
            assert ch.segment_id == "lanelet:100"
            assert ch.field_name == "structure"
            assert ch.verdict == "REJECT"
            assert "explicit centreline ADDED" in ch.reason


class TestEvaluationScoring:
    def test_evaluate_perfect_match(self):
        rep = DiffReport()
        rep.changes = [
            Change("l1", "width_m", 3.0, 4.0, 1.0, 0.33, verdict="REJECT"),
            Change("l2", "width_m", 3.0, 4.0, 1.0, 0.33, verdict="REJECT"),
        ]

        with tempfile.TemporaryDirectory() as tmpdir:
            labels_path = Path(tmpdir) / "labels.json"
            labels_path.write_text(json.dumps({"l1": {"tampered": True}, "l2": {"tampered": True}}))

            metrics = evaluate(rep, str(labels_path))
            assert metrics["TP"] == 2
            assert metrics["FP"] == 0
            assert metrics["FN"] == 0
            assert metrics["precision"] == 1.0
            assert metrics["recall"] == 1.0
            assert metrics["f1"] == 1.0


class TestLaneletPresence:
    """BUG-03: added/removed lanelets must be explicit REJECT findings."""

    def test_clean_vs_clean_still_accepts(self):
        a = [_seg_with("lanelet:1", "width_m", 3.0)]
        b = [_seg_with("lanelet:1", "width_m", 3.0)]
        rep = compare(a, b)
        assert rep.changes == []
        assert rep.added == []
        assert rep.removed == []
        assert rep.decide() == "ACCEPT"

    def test_one_added_lanelet_rejected(self):
        a = [_seg_with("lanelet:1", "width_m", 3.0)]
        b = [_seg_with("lanelet:1", "width_m", 3.0),
             _seg_with("lanelet:2", "width_m", 3.0)]
        rep = compare(a, b)

        assert rep.added == ["lanelet:2"]
        assert rep.removed == []
        assert len(rep.changes) == 1
        ch = rep.changes[0]
        assert ch.segment_id == "lanelet:2"
        assert ch.field_name == "lanelet_presence"
        assert ch.verdict == "REJECT"
        assert "addition" in ch.reason
        assert rep.rejected() == [ch]
        assert rep.decide() == "REJECT"

    def test_one_removed_lanelet_rejected(self):
        a = [_seg_with("lanelet:1", "width_m", 3.0),
             _seg_with("lanelet:2", "width_m", 3.0)]
        b = [_seg_with("lanelet:1", "width_m", 3.0)]
        rep = compare(a, b)

        assert rep.added == []
        assert rep.removed == ["lanelet:2"]
        assert len(rep.changes) == 1
        ch = rep.changes[0]
        assert ch.segment_id == "lanelet:2"
        assert ch.field_name == "lanelet_presence"
        assert ch.verdict == "REJECT"
        assert "removal" in ch.reason
        assert rep.rejected() == [ch]
        assert rep.decide() == "REJECT"

    def test_multiple_added_lanelets_rejected(self):
        a = [_seg_with("lanelet:1", "width_m", 3.0)]
        b = [_seg_with("lanelet:1", "width_m", 3.0),
             _seg_with("lanelet:2", "width_m", 3.0),
             _seg_with("lanelet:3", "width_m", 3.0),
             _seg_with("lanelet:4", "width_m", 3.0)]
        rep = compare(a, b)

        assert rep.added == ["lanelet:2", "lanelet:3", "lanelet:4"]
        assert rep.removed == []
        assert len(rep.changes) == 3
        assert {c.segment_id for c in rep.changes} == {
            "lanelet:2", "lanelet:3", "lanelet:4"}
        assert all(c.verdict == "REJECT" for c in rep.changes)
        assert all(c.field_name == "lanelet_presence" for c in rep.changes)
        assert rep.decide() == "REJECT"

    def test_multiple_removed_lanelets_rejected(self):
        a = [_seg_with("lanelet:1", "width_m", 3.0),
             _seg_with("lanelet:2", "width_m", 3.0),
             _seg_with("lanelet:3", "width_m", 3.0)]
        b = [_seg_with("lanelet:1", "width_m", 3.0)]
        rep = compare(a, b)

        assert rep.added == []
        assert rep.removed == ["lanelet:2", "lanelet:3"]
        assert len(rep.changes) == 2
        assert all(c.verdict == "REJECT" for c in rep.changes)
        assert rep.decide() == "REJECT"

    def test_added_and_removed_lanelets_rejected(self):
        a = [_seg_with("lanelet:1", "width_m", 3.0),
             _seg_with("lanelet:2", "width_m", 3.0)]
        b = [_seg_with("lanelet:1", "width_m", 3.0),
             _seg_with("lanelet:3", "width_m", 3.0)]
        rep = compare(a, b)

        assert rep.added == ["lanelet:3"]
        assert rep.removed == ["lanelet:2"]
        assert len(rep.changes) == 2
        assert {c.field_name for c in rep.changes} == {"lanelet_presence"}
        assert {c.segment_id for c in rep.changes} == {"lanelet:2", "lanelet:3"}
        assert all(c.verdict == "REJECT" for c in rep.changes)
        assert rep.decide() == "REJECT"

    def test_presence_and_width_change_both_reported(self):
        a = [_make_seg("lanelet:1", width_m=3.0)]
        b = [_make_seg("lanelet:1", width_m=4.0),
             _make_seg("lanelet:2", width_m=3.0)]
        rep = compare(a, b)

        assert {c.field_name for c in rep.changes} == {
            "width_m", "lanelet_presence"}
        assert all(c.verdict == "REJECT" for c in rep.changes)
        assert rep.flagged_ids() == {"lanelet:1", "lanelet:2"}
        assert rep.decide() == "REJECT"


class TestAttributePresence:
    """BUG-04: presence transitions and previously-uncompared attributes."""

    @pytest.mark.parametrize("field_name,previous,candidate,expected", [
        # width: existing value thresholds preserved; presence -> REVIEW
        ("width_m", 3.0, 3.0, None),
        ("width_m", 3.0, 3.3, "SUSPECT"),
        ("width_m", 3.0, 3.9, "REJECT"),
        ("width_m", 3.0, None, "SUSPECT"),
        ("width_m", None, 3.0, "SUSPECT"),
        ("width_m", None, None, None),
        # speed: existing thresholds preserved; lost limit -> REJECT
        ("speed_limit_kph", 50.0, 50.0, None),
        ("speed_limit_kph", 50.0, 60.0, "SUSPECT"),
        ("speed_limit_kph", 50.0, 70.0, "REJECT"),
        ("speed_limit_kph", 50.0, None, "REJECT"),
        ("speed_limit_kph", None, 50.0, "SUSPECT"),
        ("speed_limit_kph", None, None, None),
        # one-way: reversal as before; lost restriction -> REJECT
        ("oneway", True, True, None),
        ("oneway", True, False, "REJECT"),
        ("oneway", True, None, "REJECT"),
        ("oneway", None, True, "SUSPECT"),
        ("oneway", None, None, None),
        # tunnel
        ("tunnel", False, False, None),
        ("tunnel", False, True, "SUSPECT"),
        ("tunnel", True, None, "SUSPECT"),
        ("tunnel", None, True, "SUSPECT"),
        ("tunnel", None, None, None),
        # bridge
        ("bridge", False, False, None),
        ("bridge", False, True, "SUSPECT"),
        ("bridge", True, None, "SUSPECT"),
        ("bridge", None, True, "SUSPECT"),
        ("bridge", None, None, None),
        # lanes
        ("lanes", 1, 1, None),
        ("lanes", 1, 3, "SUSPECT"),
        ("lanes", 2, None, "SUSPECT"),
        ("lanes", None, 2, "SUSPECT"),
        ("lanes", None, None, None),
        # highway class / subtype
        ("highway_class", "residential", "residential", None),
        ("highway_class", "residential", "primary", "SUSPECT"),
        ("highway_class", "primary", None, "SUSPECT"),
        ("highway_class", None, "primary", "SUSPECT"),
        ("highway_class", None, None, None),
    ])
    def test_attribute_transitions(self, field_name, previous, candidate, expected):
        rep = compare([_seg_with("lanelet:1", field_name, previous)],
                      [_seg_with("lanelet:1", field_name, candidate)])

        if expected is None:
            assert rep.changes == []
            assert rep.decide() == "ACCEPT"
            return

        assert len(rep.changes) == 1
        ch = rep.changes[0]
        assert ch.segment_id == "lanelet:1"
        assert ch.field_name == field_name
        assert ch.verdict == expected
        assert ch.reason
        assert rep.decide() == ("REJECT" if expected == "REJECT" else "REVIEW")

    def test_removed_speed_limit_rejected(self):
        rep = compare([_seg_with("lanelet:1", "speed_limit_kph", 50.0)],
                      [_seg_with("lanelet:1", "speed_limit_kph", None)])
        ch = rep.changes[0]
        assert ch.field_name == "speed_limit_kph"
        assert ch.verdict == "REJECT"
        assert "REMOVED" in ch.reason
        assert rep.decide() == "REJECT"

    def test_removed_oneway_rejected(self):
        rep = compare([_seg_with("lanelet:1", "oneway", True)],
                      [_seg_with("lanelet:1", "oneway", None)])
        ch = rep.changes[0]
        assert ch.field_name == "oneway"
        assert ch.verdict == "REJECT"
        assert "REMOVED" in ch.reason
        assert rep.decide() == "REJECT"

    def test_width_becoming_missing_is_review(self):
        rep = compare([_seg_with("lanelet:1", "width_m", 3.0)],
                      [_seg_with("lanelet:1", "width_m", None)])
        ch = rep.changes[0]
        assert ch.field_name == "width_m"
        assert ch.verdict == "SUSPECT"
        assert rep.decide() == "REVIEW"

    def test_introduced_tunnel_is_review(self):
        rep = compare([_seg_with("lanelet:1", "tunnel", None)],
                      [_seg_with("lanelet:1", "tunnel", True)])
        ch = rep.changes[0]
        assert ch.field_name == "tunnel"
        assert ch.verdict == "SUSPECT"
        assert rep.decide() == "REVIEW"

    def test_introduced_bridge_is_review(self):
        rep = compare([_seg_with("lanelet:1", "bridge", None)],
                      [_seg_with("lanelet:1", "bridge", True)])
        ch = rep.changes[0]
        assert ch.field_name == "bridge"
        assert ch.verdict == "SUSPECT"
        assert rep.decide() == "REVIEW"

    def test_lane_count_change_is_review(self):
        rep = compare([_seg_with("lanelet:1", "lanes", 1)],
                      [_seg_with("lanelet:1", "lanes", 3)])
        ch = rep.changes[0]
        assert ch.field_name == "lanes"
        assert ch.verdict == "SUSPECT"
        assert rep.decide() == "REVIEW"

    def test_highway_class_change_is_review(self):
        rep = compare([_seg_with("lanelet:1", "highway_class", "residential")],
                      [_seg_with("lanelet:1", "highway_class", "primary")])
        ch = rep.changes[0]
        assert ch.field_name == "highway_class"
        assert ch.verdict == "SUSPECT"
        assert rep.decide() == "REVIEW"

    def test_unparsable_speed_tag_reported_as_parser_limitation(self):
        """A tag that is present but unparsable is REVIEW, not a verified removal."""
        prev = _seg_with("lanelet:1", "speed_limit_kph", 50.0,
                         raw_tags={"speed_limit": "50"})
        cand = _seg_with("lanelet:1", "speed_limit_kph", None,
                         raw_tags={"speed_limit": "bogus"})
        rep = compare([prev], [cand])

        ch = rep.changes[0]
        assert ch.field_name == "speed_limit_kph"
        assert ch.verdict == "SUSPECT"
        assert "parser limitation" in ch.reason
        assert rep.decide() == "REVIEW"

    def test_genuine_tag_removal_is_rejected_for_speed(self):
        """The tag is gone from the source element: this is a removal, not a parse failure."""
        prev = _seg_with("lanelet:1", "speed_limit_kph", 50.0,
                         raw_tags={"speed_limit": "50"})
        cand = _seg_with("lanelet:1", "speed_limit_kph", None,
                         raw_tags={"subtype": "road"})
        rep = compare([prev], [cand])

        ch = rep.changes[0]
        assert ch.verdict == "REJECT"
        assert "REMOVED" in ch.reason
        assert rep.decide() == "REJECT"
