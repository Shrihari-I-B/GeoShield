"""
GeoShield — BUG-09: subprocess output freshness and return-code validation.

Self-contained regression tests. No dependency on data/, bags/, results/,
or the external Nishi-Shinjuku dataset. Child subprocesses are mocked via
monkeypatch on geoshield_verifier.subprocess.run, which writes synthetic
report JSON to the paths the real child scripts would use.
"""
from __future__ import annotations

import json
import subprocess
import sys
import uuid
from pathlib import Path

import pytest

from geoshield_verifier import run_dv, run_repair, ADMIT, REFUSE, ERROR
import geoshield_verifier as gv


def _make_report(verdict: str, invocation_id: str) -> str:
    """Build a synthetic differential_verify report JSON string."""
    return json.dumps({
        "verdict": verdict,
        "_invocation_id": invocation_id,
        "n_previous": 1,
        "n_candidate": 1,
        "added": [],
        "removed": [],
        "changes": [],
    })


def _make_repair_report(invocation_id: str) -> str:
    """Build a synthetic repair_differential report JSON string."""
    return json.dumps({
        "_invocation_id": invocation_id,
        "lanelets_restored": [],
        "nodes_considered": 0,
        "nodes_restored": 0,
        "nodes_unchanged": 0,
        "nodes_missing": 0,
        "members_removed": 0,
        "orphan_ways_removed": 0,
        "orphan_nodes_removed": 0,
    })


def _find_arg(cmd, flag):
    """Return the value following *flag* in a subprocess cmd list, or None."""
    for i, arg in enumerate(cmd):
        if arg == flag and i + 1 < len(cmd):
            return Path(cmd[i + 1])
    return None


def _mock_runner(monkeypatch, return_code=0, report_content=None,
                 write_report=True, missing_report=False, malformed=False,
                 empty_report=False, out_map_content=None, write_out_map=False):
    """Monkeypatch geoshield_verifier.subprocess.run to simulate a child.

    The mock inspects the cmd list for --report and --out flags, writes a
    synthetic report (and optionally an output map) to those paths, and
    returns a CompletedProcess with *return_code*.
    """
    def mock_run(cmd, **kwargs):
        report_path = _find_arg(cmd, "--report")
        out_path = _find_arg(cmd, "--out")

        if write_out_map and out_path:
            out_path.write_text(out_map_content if out_map_content is not None
                                else "<repaired/>")

        if write_report and report_path is not None:
            if missing_report:
                pass  # do not write anything
            elif malformed:
                report_path.write_text("{bad json!!!")
            elif empty_report:
                report_path.write_text("")
            elif report_content is not None:
                report_path.write_text(report_content)

        return subprocess.CompletedProcess(cmd, return_code, stderr="")

    monkeypatch.setattr(gv.subprocess, "run", mock_run)
    return mock_run


# ===========================================================================
# Differential verification freshness tests
# ===========================================================================


class TestDvFreshness:
    """Freshness and return-code validation for run_dv()."""

    def test_fresh_accept_exit0(self, tmp_path, monkeypatch):
        """1. Fresh ACCEPT report, exit 0."""
        inv_id = str(uuid.uuid4())
        report_path = tmp_path / "diff_in.json"
        _mock_runner(monkeypatch, return_code=0,
                     report_content=_make_report("ACCEPT", inv_id))
        rep = run_dv("prior.osm", "candidate.osm", report_path,
                     invocation_id=inv_id)
        assert rep is not None
        assert rep["verdict"] == "ACCEPT"

    def test_fresh_review_exit0(self, tmp_path, monkeypatch):
        """2. Fresh REVIEW report, exit 0."""
        inv_id = str(uuid.uuid4())
        report_path = tmp_path / "diff_in.json"
        _mock_runner(monkeypatch, return_code=0,
                     report_content=_make_report("REVIEW", inv_id))
        rep = run_dv("prior.osm", "candidate.osm", report_path,
                     invocation_id=inv_id)
        assert rep is not None
        assert rep["verdict"] == "REVIEW"

    def test_fresh_reject_exit2(self, tmp_path, monkeypatch):
        """3. Fresh REJECT report, exit 2."""
        inv_id = str(uuid.uuid4())
        report_path = tmp_path / "diff_in.json"
        _mock_runner(monkeypatch, return_code=2,
                     report_content=_make_report("REJECT", inv_id))
        rep = run_dv("prior.osm", "candidate.osm", report_path,
                     invocation_id=inv_id)
        assert rep is not None
        assert rep["verdict"] == "REJECT"

    def test_exit1_with_stale_accept_report_refuses(self, tmp_path, monkeypatch):
        """4. Exit 1 with an old ACCEPT report still present → RuntimeError."""
        inv_id = str(uuid.uuid4())
        report_path = tmp_path / "diff_in.json"
        report_path.write_text(_make_report("ACCEPT", "stale-id-12345"))
        _mock_runner(monkeypatch, return_code=1, write_report=False)
        try:
            run_dv("prior.osm", "candidate.osm", report_path,
                   invocation_id=inv_id)
            pytest.fail("RuntimeError expected — exit 1 + stale ACCEPT must refuse")
        except RuntimeError as e:
            assert "unexpected" in str(e) or "does not match" in str(e)

    def test_missing_report(self, tmp_path, monkeypatch):
        """5. Missing report → RuntimeError."""
        inv_id = str(uuid.uuid4())
        report_path = tmp_path / "nonexistent.json"
        _mock_runner(monkeypatch, return_code=0, missing_report=True)
        with pytest.raises(RuntimeError, match="wrote no report"):
            run_dv("prior.osm", "candidate.osm", report_path,
                   invocation_id=inv_id)

    def test_malformed_json(self, tmp_path, monkeypatch):
        """6. Malformed JSON report → RuntimeError."""
        inv_id = str(uuid.uuid4())
        report_path = tmp_path / "diff_in.json"
        _mock_runner(monkeypatch, return_code=0, malformed=True)
        with pytest.raises(RuntimeError):
            run_dv("prior.osm", "candidate.osm", report_path,
                   invocation_id=inv_id)

    def test_empty_json(self, tmp_path, monkeypatch):
        """7. Empty JSON report → RuntimeError."""
        inv_id = str(uuid.uuid4())
        report_path = tmp_path / "diff_in.json"
        _mock_runner(monkeypatch, return_code=0, empty_report=True)
        with pytest.raises(RuntimeError):
            run_dv("prior.osm", "candidate.osm", report_path,
                   invocation_id=inv_id)

    def test_missing_invocation_id(self, tmp_path, monkeypatch):
        """8. Report with no _invocation_id → RuntimeError."""
        inv_id = str(uuid.uuid4())
        report_path = tmp_path / "diff_in.json"
        report = {"verdict": "ACCEPT", "n_previous": 1, "n_candidate": 1,
                  "added": [], "removed": [], "changes": []}
        _mock_runner(monkeypatch, return_code=0,
                     report_content=json.dumps(report))
        with pytest.raises(RuntimeError, match="does not match"):
            run_dv("prior.osm", "candidate.osm", report_path,
                   invocation_id=inv_id)

    def test_incorrect_invocation_id(self, tmp_path, monkeypatch):
        """9. Report with wrong _invocation_id → RuntimeError."""
        inv_id = str(uuid.uuid4())
        old_id = str(uuid.uuid4())
        assert old_id != inv_id
        report_path = tmp_path / "diff_in.json"
        _mock_runner(monkeypatch, return_code=0,
                     report_content=_make_report("ACCEPT", old_id))
        with pytest.raises(RuntimeError, match="does not match"):
            run_dv("prior.osm", "candidate.osm", report_path,
                   invocation_id=inv_id)

    def test_returncode_verdict_disagreement(self, tmp_path, monkeypatch):
        """10. Exit-code/verdict disagreement → RuntimeError."""
        inv_id = str(uuid.uuid4())
        report_path = tmp_path / "diff_in.json"
        _mock_runner(monkeypatch, return_code=2,
                     report_content=_make_report("ACCEPT", inv_id))
        with pytest.raises(RuntimeError, match="REJECT code 2"):
            run_dv("prior.osm", "candidate.osm", report_path,
                   invocation_id=inv_id)

    def test_two_invocations_same_candidate(self, tmp_path, monkeypatch):
        """11. Two invocations same candidate filename; both succeed with fresh IDs."""
        inv_id_1 = str(uuid.uuid4())
        inv_id_2 = str(uuid.uuid4())
        assert inv_id_1 != inv_id_2
        report_path = tmp_path / "gate_candidate_in.json"

        _mock_runner(monkeypatch, return_code=0,
                     report_content=_make_report("ACCEPT", inv_id_1))
        rep1 = run_dv("prior.osm", "candidate.osm", report_path,
                      invocation_id=inv_id_1)
        assert rep1["verdict"] == "ACCEPT"

        _mock_runner(monkeypatch, return_code=0,
                     report_content=_make_report("ACCEPT", inv_id_2))
        rep2 = run_dv("prior.osm", "candidate.osm", report_path,
                      invocation_id=inv_id_2)
        assert rep2["verdict"] == "ACCEPT"


# ===========================================================================
# Repair freshness tests
# ===========================================================================


class TestRepairFreshness:
    """Freshness and return-code validation for run_repair()."""

    def test_successful_repair_exit0_fresh_output(self, tmp_path, monkeypatch):
        """1. Successful repair producing fresh output."""
        inv_id = str(uuid.uuid4())
        report_path = tmp_path / "repair.json"
        out_map = tmp_path / "repaired.osm"
        diff_report = tmp_path / "diff_in.json"
        _mock_runner(
            monkeypatch,
            return_code=0,
            report_content=_make_repair_report(inv_id),
            write_out_map=True,
            out_map_content='<osm version="0.6"/>',
        )
        rep = run_repair("prior.osm", "candidate.osm", diff_report, out_map,
                         True, report_path, invocation_id=inv_id)
        assert rep is not None
        assert rep["_invocation_id"] == inv_id
        assert out_map.exists()

    def test_failed_repair_old_map_present(self, tmp_path, monkeypatch):
        """2. Failed repair with an old repaired map already present."""
        inv_id = str(uuid.uuid4())
        report_path = tmp_path / "repair.json"
        out_map = tmp_path / "repaired.osm"
        diff_report = tmp_path / "diff_in.json"
        out_map.write_text("<osm><old/>")

        _mock_runner(monkeypatch, return_code=1, write_report=False)
        try:
            run_repair("prior.osm", "candidate.osm", diff_report, out_map,
                       True, report_path, invocation_id=inv_id)
            pytest.fail("RuntimeError expected — failed repair must refuse")
        except RuntimeError as e:
            assert "exited with code 1" in str(e)

    def test_successful_exit_no_output_map(self, tmp_path, monkeypatch):
        """3. Successful exit (0) but no output map written."""
        inv_id = str(uuid.uuid4())
        report_path = tmp_path / "repair.json"
        out_map = tmp_path / "repaired.osm"
        diff_report = tmp_path / "diff_in.json"

        _mock_runner(
            monkeypatch,
            return_code=0,
            report_content=_make_repair_report(inv_id),
            write_out_map=False,
        )
        rep = run_repair("prior.osm", "candidate.osm", diff_report, out_map,
                         True, report_path, invocation_id=inv_id)
        assert rep is not None
        assert not out_map.exists()

    def test_empty_output_map(self, tmp_path, monkeypatch):
        """4. Empty output map file."""
        inv_id = str(uuid.uuid4())
        report_path = tmp_path / "repair.json"
        out_map = tmp_path / "repaired.osm"
        diff_report = tmp_path / "diff_in.json"

        _mock_runner(
            monkeypatch,
            return_code=0,
            report_content=_make_repair_report(inv_id),
            write_out_map=True,
            out_map_content="",
        )
        rep = run_repair("prior.osm", "candidate.osm", diff_report, out_map,
                         True, report_path, invocation_id=inv_id)
        assert rep is not None
        assert out_map.read_text() == ""

    def test_unreadable_output_map(self, tmp_path, monkeypatch):
        """5. Unreadable output map (permissions)."""
        inv_id = str(uuid.uuid4())
        report_path = tmp_path / "repair.json"
        out_map = tmp_path / "repaired.osm"
        diff_report = tmp_path / "diff_in.json"

        _mock_runner(
            monkeypatch,
            return_code=0,
            report_content=_make_repair_report(inv_id),
            write_out_map=True,
            out_map_content="<osm/>",
        )
        rep = run_repair("prior.osm", "candidate.osm", diff_report, out_map,
                         True, report_path, invocation_id=inv_id)
        assert rep is not None
        out_map.chmod(0o000)
        try:
            content = out_map.read_text()
            pytest.skip("Running as root — permission test not enforced")
        except PermissionError:
            pass
        finally:
            out_map.chmod(0o644)

    def test_nonzero_exit_partial_output(self, tmp_path, monkeypatch):
        """6. Non-zero repair exit with partial output."""
        inv_id = str(uuid.uuid4())
        report_path = tmp_path / "repair.json"
        out_map = tmp_path / "repaired.osm"
        diff_report = tmp_path / "diff_in.json"

        out_map.write_text("<osm><partial/>")
        report_path.write_text(_make_repair_report("old-id"))

        _mock_runner(monkeypatch, return_code=1, write_report=False)
        try:
            run_repair("prior.osm", "candidate.osm", diff_report, out_map,
                       True, report_path, invocation_id=inv_id)
            pytest.fail("RuntimeError expected — non-zero repair exit must fail")
        except RuntimeError as e:
            assert "exited with code 1" in str(e)

    def test_successful_repair_followed_by_accept(self, tmp_path, monkeypatch):
        """7. Successful repair followed by fresh post-repair ACCEPT."""
        inv_id = str(uuid.uuid4())
        diff_report = tmp_path / "diff_in.json"
        report_path = tmp_path / "repair.json"
        out_map = tmp_path / "repaired.osm"
        post_report = tmp_path / "post_dv.json"

        _mock_runner(monkeypatch, return_code=2,
                     report_content=_make_report("REJECT", inv_id))
        dvrep = run_dv("prior.osm", "candidate.osm", diff_report,
                       invocation_id=inv_id)
        assert dvrep["verdict"] == "REJECT"

        _mock_runner(monkeypatch, return_code=0,
                     report_content=_make_repair_report(inv_id),
                     write_out_map=True,
                     out_map_content='<osm version="0.6"/>')
        rep = run_repair("prior.osm", "candidate.osm", diff_report, out_map,
                         True, report_path, invocation_id=inv_id)
        assert rep["_invocation_id"] == inv_id

        _mock_runner(monkeypatch, return_code=0,
                     report_content=_make_report("ACCEPT", inv_id))
        postrep = run_dv("prior.osm", out_map, post_report,
                         invocation_id=inv_id)
        assert postrep["verdict"] == "ACCEPT"

    def test_successful_repair_followed_by_review(self, tmp_path, monkeypatch):
        """8. Successful repair followed by post-repair REVIEW."""
        inv_id = str(uuid.uuid4())
        diff_report = tmp_path / "diff_in.json"
        report_path = tmp_path / "repair.json"
        out_map = tmp_path / "repaired.osm"
        post_report = tmp_path / "post_dv.json"

        _mock_runner(monkeypatch, return_code=2,
                     report_content=_make_report("REJECT", inv_id))
        dvrep = run_dv("prior.osm", "candidate.osm", diff_report,
                       invocation_id=inv_id)
        assert dvrep["verdict"] == "REJECT"

        _mock_runner(monkeypatch, return_code=0,
                     report_content=_make_repair_report(inv_id),
                     write_out_map=True,
                     out_map_content='<osm version="0.6"/>')
        rep = run_repair("prior.osm", "candidate.osm", diff_report, out_map,
                         True, report_path, invocation_id=inv_id)
        assert rep["_invocation_id"] == inv_id

        _mock_runner(monkeypatch, return_code=0,
                     report_content=_make_report("REVIEW", inv_id))
        postrep = run_dv("prior.osm", out_map, post_report,
                         invocation_id=inv_id)
        assert postrep["verdict"] == "REVIEW"

    def test_successful_repair_followed_by_reject(self, tmp_path, monkeypatch):
        """9. Successful repair followed by post-repair REJECT."""
        inv_id = str(uuid.uuid4())
        diff_report = tmp_path / "diff_in.json"
        report_path = tmp_path / "repair.json"
        out_map = tmp_path / "repaired.osm"
        post_report = tmp_path / "post_dv.json"

        _mock_runner(monkeypatch, return_code=2,
                     report_content=_make_report("REJECT", inv_id))
        dvrep = run_dv("prior.osm", "candidate.osm", diff_report,
                       invocation_id=inv_id)
        assert dvrep["verdict"] == "REJECT"

        _mock_runner(monkeypatch, return_code=0,
                     report_content=_make_repair_report(inv_id),
                     write_out_map=True,
                     out_map_content='<osm version="0.6"/>')
        rep = run_repair("prior.osm", "candidate.osm", diff_report, out_map,
                         True, report_path, invocation_id=inv_id)
        assert rep["_invocation_id"] == inv_id

        _mock_runner(monkeypatch, return_code=2,
                     report_content=_make_report("REJECT", inv_id))
        postrep = run_dv("prior.osm", out_map, post_report,
                         invocation_id=inv_id)
        assert postrep["verdict"] == "REJECT"


# ===========================================================================
# Gate-level admission/refusal tests
# ===========================================================================


class TestGateAdmissionRefusal:
    """Gate-level tests verifying admission and refusal behaviours."""

    def _run_gate(self, monkeypatch, tmp_path, dv_return_code=0,
                  dv_verdict="ACCEPT", repair_return_code=0,
                  post_dv_return_code=0, post_dv_verdict="ACCEPT",
                  invocation_id=None, no_repair=False):
        """Invoke geoshield_verifier.main() with mocked subprocess via sys.argv."""
        inv_id = invocation_id or str(uuid.uuid4())
        report_dir = tmp_path / "results"
        candidate = tmp_path / "candidate.osm"
        prior = tmp_path / "prior.osm"
        candidate.write_text("<osm/>")
        prior.write_text("<osm/>")

        def mock_run(cmd, **kwargs):
            report_path = _find_arg(cmd, "--report")
            out_path = _find_arg(cmd, "--out")
            # Extract --invocation-id from the cmd so the synthetic report
            # matches what run_dv/run_repair expect.
            inv_id_from_cmd = None
            for i, arg in enumerate(cmd):
                if arg == "--invocation-id" and i + 1 < len(cmd):
                    inv_id_from_cmd = cmd[i + 1]
                    break
            effective_inv_id = inv_id_from_cmd or inv_id
            if "differential_verify.py" in cmd[1]:
                if report_path:
                    if dv_return_code == 2:
                        report_path.write_text(_make_report("REJECT", effective_inv_id))
                    else:
                        report_path.write_text(_make_report(dv_verdict, effective_inv_id))
                return subprocess.CompletedProcess(cmd, dv_return_code,
                                                   stderr="")
            elif "repair_differential.py" in cmd[1]:
                if out_path:
                    out_path.write_text("<osm version='0.6'/>")
                if report_path:
                    report_path.write_text(_make_repair_report(effective_inv_id))
                return subprocess.CompletedProcess(cmd, repair_return_code,
                                                   stderr="")
            return subprocess.CompletedProcess(cmd, 0, stderr="")

        monkeypatch.setattr(gv.subprocess, "run", mock_run)

        old_argv = sys.argv
        sys.argv = ["geoshield_verifier.py", "--candidate", str(candidate),
                    "--prior", str(prior), "--report-dir", str(report_dir)]
        if no_repair:
            sys.argv.append("--no-repair")
        try:
            return gv.main()
        finally:
            sys.argv = old_argv

    def test_differential_execution_error_causes_refusal(self, tmp_path,
                                                        monkeypatch):
        """Gate: differential execution error → ERROR (exit 1)."""
        rc = self._run_gate(monkeypatch, tmp_path, dv_return_code=1)
        assert rc == ERROR

    def test_invalid_report_causes_refusal(self, tmp_path, monkeypatch):
        """Gate: invalid/malformed report → ERROR."""
        report_dir = tmp_path / "results"
        candidate = tmp_path / "candidate.osm"
        prior = tmp_path / "prior.osm"
        candidate.write_text("<osm/>")
        prior.write_text("<osm/>")

        def mock_run(cmd, **kwargs):
            report_path = _find_arg(cmd, "--report")
            if report_path:
                report_path.write_text("{bad json!!!")
            return subprocess.CompletedProcess(cmd, 0, stderr="")

        monkeypatch.setattr(gv.subprocess, "run", mock_run)

        old_argv = sys.argv
        sys.argv = ["geoshield_verifier.py", "--candidate", str(candidate),
                    "--prior", str(prior), "--report-dir", str(report_dir)]
        try:
            rc = gv.main()
        finally:
            sys.argv = old_argv
        assert rc == ERROR

    def test_review_never_permits_admission(self, tmp_path, monkeypatch):
        """Gate: REVIEW verdict never permits admission."""
        rc = self._run_gate(monkeypatch, tmp_path, dv_verdict="REVIEW",
                            no_repair=True)
        assert rc == REFUSE

    def test_reject_never_permits_admission(self, tmp_path, monkeypatch):
        """Gate: REJECT verdict never permits admission."""
        rc = self._run_gate(monkeypatch, tmp_path, dv_return_code=2,
                            dv_verdict="REJECT", no_repair=True)
        assert rc == REFUSE

    def test_failed_repairs_never_reuse_old_repaired_maps(self, tmp_path,
                                                           monkeypatch):
        """Gate: failed repairs never reuse old repaired maps."""
        inv_id = str(uuid.uuid4())
        report_dir = tmp_path / "results"
        candidate = tmp_path / "candidate.osm"
        prior = tmp_path / "prior.osm"
        repaired = tmp_path / "repaired.osm"
        candidate.write_text("<osm/>")
        prior.write_text("<osm/>")
        repaired.write_text("<osm><old/>")

        def mock_run(cmd, **kwargs):
            report_path = _find_arg(cmd, "--report")
            if "differential_verify.py" in cmd[1]:
                report_path.write_text(_make_report("REJECT", inv_id))
                return subprocess.CompletedProcess(cmd, 2, stderr="")
            elif "repair_differential.py" in cmd[1]:
                return subprocess.CompletedProcess(cmd, 1, stderr="fail")
            return subprocess.CompletedProcess(cmd, 0, stderr="")

        monkeypatch.setattr(gv.subprocess, "run", mock_run)

        old_argv = sys.argv
        sys.argv = ["geoshield_verifier.py", "--candidate", str(candidate),
                    "--prior", str(prior), "--report-dir", str(report_dir),
                    "--out-map", str(repaired)]
        try:
            rc = gv.main()
        finally:
            sys.argv = old_argv
        assert rc == ERROR

    def test_fresh_accept_still_permits_admission(self, tmp_path, monkeypatch):
        """Gate: fresh ACCEPT verdict still permits admission."""
        rc = self._run_gate(monkeypatch, tmp_path, dv_verdict="ACCEPT")
        assert rc == ADMIT
