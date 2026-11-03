#!/usr/bin/env python3
"""
Regression tests for BUG-02: run_all_dv.py centerline_injection prefix NameError.
Self-contained, no external data dependencies.
"""

import json
import tempfile
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).parent.parent))

import run_all_dv


# Minimal valid Lanelet2 OSM with one lanelet (no centerline) for testing.
# Based on test_injector.py's MINI_OSM.
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


def test_run_centerline_injection_no_nameerror():
    """Centerline injection executes without NameError."""
    with tempfile.TemporaryDirectory() as tmpdir:
        clean_map = Path(tmpdir) / "clean.osm"
        clean_map.write_text(MINI_OSM)
        data_dir = Path(tmpdir) / "data"
        results_dir = Path(tmpdir) / "results"
        data_dir.mkdir()
        results_dir.mkdir()

        # Should not raise NameError
        result = run_all_dv.run_centerline_injection(
            str(clean_map), data_dir, results_dir, seed=42
        )

        # Basic structure checks
        assert result["attack"] == "centerline_injection"
        assert "verdict" in result
        assert "detection" in result
        assert "elapsed_s" in result


def test_run_centerline_injection_outputs():
    """All expected centerline output files are generated."""
    with tempfile.TemporaryDirectory() as tmpdir:
        clean_map = Path(tmpdir) / "clean.osm"
        clean_map.write_text(MINI_OSM)
        data_dir = Path(tmpdir) / "data"
        results_dir = Path(tmpdir) / "results"
        data_dir.mkdir()
        results_dir.mkdir()

        run_all_dv.run_centerline_injection(
            str(clean_map), data_dir, results_dir, seed=42
        )

        # Check the four output files exist
        expected_files = [
            data_dir / "dv_centerline_injection.osm",
            data_dir / "dv_centerline_injection_labels.json",
            data_dir / "dv_centerline_injection_meta.json",
            results_dir / "dv_centerline_injection.json",
        ]
        for f in expected_files:
            assert f.exists(), f"Missing expected output: {f}"


def test_run_centerline_injection_result_structure():
    """Result JSON has expected structure and verdict."""
    with tempfile.TemporaryDirectory() as tmpdir:
        clean_map = Path(tmpdir) / "clean.osm"
        clean_map.write_text(MINI_OSM)
        data_dir = Path(tmpdir) / "data"
        results_dir = Path(tmpdir) / "results"
        data_dir.mkdir()
        results_dir.mkdir()

        run_all_dv.run_centerline_injection(
            str(clean_map), data_dir, results_dir, seed=42
        )

        result_path = results_dir / "dv_centerline_injection.json"
        with open(result_path) as f:
            data = json.load(f)

        # Required fields
        assert data["verdict"] in ("REJECT", "REVIEW", "ACCEPT")
        assert "detection" in data
        assert "changes" in data
        assert data["candidate"] == str(data_dir / "dv_centerline_injection.osm")


def test_run_all_dv_smoke():
    """Full experiment runner smoke test with minimal map."""
    with tempfile.TemporaryDirectory() as tmpdir:
        clean_map = Path(tmpdir) / "clean.osm"
        clean_map.write_text(MINI_OSM)
        data_dir = Path(tmpdir) / "data"
        results_dir = Path(tmpdir) / "results"
        data_dir.mkdir()
        results_dir.mkdir()

        # Run the main logic but with only one attack to keep it fast.
        # We'll monkey-patch ATTACKS_TO_RUN to just one attack.
        original_attacks = run_all_dv.ATTACKS_TO_RUN
        run_all_dv.ATTACKS_TO_RUN = ["width_step"]  # single attack that works on our map
        try:
            # We need to call main() but it uses argparse. Instead, we replicate the main logic
            # with our directories and seed.
            from run_all_dv import main as run_all_dv_main
            import argparse
            import sys

            # Simulate command line arguments
            sys.argv = ["run_all_dv.py", "--map", str(clean_map),
                        "--data-dir", str(data_dir),
                        "--results-dir", str(results_dir),
                        "--seed", "42"]
            run_all_dv_main()
        finally:
            run_all_dv.ATTACKS_TO_RUN = original_attacks

        # Check summary exists and has expected number of entries (clean_control + 1 attack + centerline = 3)
        summary_path = results_dir / "dv_summary.json"
        assert summary_path.exists(), "Summary file not created"

        with open(summary_path) as f:
            summary = json.load(f)

        # Should have results list
        assert "results" in summary
        results = summary["results"]
        # Expect: clean_control + width_step + centerline_injection = 3
        # But note: width_step may be N/A if no applicable lanelets? Our map has one lanelet, width_step should work.
        # centerline_injection should work.
        # So at least 3 entries.
        assert len(results) >= 3, f"Expected at least 3 results, got {len(results)}"

        # Verify each entry has required fields
        for r in results:
            assert "attack" in r
            assert "verdict" in r

        # Check that centerline_injection is present
        attacks = [r["attack"] for r in results]
        assert "centerline_injection" in attacks
        assert "clean_control" in attacks


if __name__ == "__main__":
    # Allow running directly for quick debugging
    test_run_centerline_injection_no_nameerror()
    print("test_run_centerline_injection_no_nameerror passed")
    test_run_centerline_injection_outputs()
    print("test_run_centerline_injection_outputs passed")
    test_run_centerline_injection_result_structure()
    print("test_run_centerline_injection_result_structure passed")
    test_run_all_dv_smoke()
    print("test_run_all_dv_smoke passed")
    print("All tests passed!")