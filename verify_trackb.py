#!/usr/bin/env python3
"""Re-derive the Track B safety figures from the bags that produced them.

Every results/fr_*.json records the two bags it read, so each published d_Fe
can be re-derived without re-recording anything. `reproduce.sh drive` asks
for fresh rb_* recordings, which is correct for a from-scratch reproduction
but costs seven 150-second Autoware launches; this checks the same claims
against the archive in seconds, which is what a demonstration needs.

A figure whose bag is present but whose value does not re-derive is a
failure. A figure whose bag is absent is reported separately: the bags are
gitignored, so absence means "not on this machine", not "wrong".
"""
import argparse, glob, json, os, subprocess, sys, tempfile


def label(path):
    return os.path.basename(path)[3:].rsplit('_', 1)[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--glob", default="results/fr_*.json")
    ap.add_argument("--tolerance", type=float, default=0.001,
                    help="metres; 1 mm by default, far below the 0.0408 m "
                         "run-to-run noise floor")
    a = ap.parse_args()

    reports = sorted(glob.glob(a.glob))
    if not reports:
        print(f"  no reports matched {a.glob}")
        return 1

    ok = fails = absent = 0
    print(f"  {'claim':<34} {'published':>10} {'re-derived':>11} {'delta':>8}")
    for r in reports:
        d = json.load(open(r))
        cb, tb, want = d.get("clean_bag"), d.get("tampered_bag"), d.get("d_Fe")
        if not (cb and tb and want is not None):
            print(f"  {label(r):<34} {'-':>10} {'-':>11} {'':>8}  NO BAGS RECORDED")
            fails += 1
            continue
        if not (os.path.isdir(cb) and os.path.isdir(tb)):
            print(f"  {label(r):<34} {want:>10.4f} {'-':>11} {'':>8}  bag absent")
            absent += 1
            continue
        out = tempfile.NamedTemporaryFile(suffix=".json", delete=False).name
        cmd = ["python3", "frechet_analysis.py", "--clean", cb, "--tampered", tb,
               "--build", d.get("autoware_build", "unspecified"), "--out", out]
        if not d.get("truncated", True):
            cmd.append("--no-truncate")
        p = subprocess.run(cmd, capture_output=True, text=True)
        if p.returncode != 0:
            print(f"  {label(r):<34} {want:>10.4f} {'ERROR':>11} {'':>8}  "
                  f"{p.stderr.strip().splitlines()[-1][:34] if p.stderr.strip() else ''}")
            fails += 1
            os.unlink(out)
            continue
        got = json.load(open(out)).get("d_Fe")
        os.unlink(out)
        delta = abs(got - want)
        good = delta <= a.tolerance
        print(f"  {label(r):<34} {want:>10.4f} {got:>11.4f} {delta:>8.4f}  "
              f"{'ok' if good else 'MISMATCH'}")
        ok += good
        fails += not good

    print()
    print(f"  {ok} re-derived, {fails} failed, {absent} bag(s) not on this machine")
    if absent and not fails:
        print("  Bags are gitignored. Record them with: bash reproduce.sh drive")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
