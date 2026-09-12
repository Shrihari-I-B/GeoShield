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


def _make_seg(sid: str, width_m=3.0, speed_kph=50.0, oneway=True, successors=None, raw_tags=None):
    return RoadSegment(
        segment_id=sid,
        source="lanelet2",
        width_m=width_m,
        speed_limit_kph=speed_kph,
        oneway=oneway,
        successors=successors or [],
        raw_tags=raw_tags or {},
    )


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
