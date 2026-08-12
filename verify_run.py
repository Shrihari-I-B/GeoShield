"""Verify that injected ramps are genuinely connected chains."""
import sys, json
sys.path.insert(0,'.')
from lanelet2_adapter import load

segs = load(sys.argv[1])
by = {s.segment_id: s for s in segs}
labels = json.load(open(sys.argv[2]))
recs = labels if "segment_id" in next(iter(labels.values()), {}) else labels.get("labels", labels)

runs = {}
for sid, r in recs.items():
    if r.get("ramp_position") is not None:
        runs.setdefault(r.get("campaign_id",""), []).append((r["ramp_position"], sid))

for cid, items in runs.items():
    items.sort()
    ids = [s for _, s in items]
    print(f"\nramp {cid or '(single)'}  length {len(ids)}")
    ok = True
    for i in range(len(ids)-1):
        connected = ids[i+1] in by[ids[i]].successors
        ok &= connected
        w0, w1 = by[ids[i]].width_m, by[ids[i+1]].width_m
        print(f"  {ids[i]:<18} -> {ids[i+1]:<18} connected={connected}")
    print(f"  CHAIN VALID: {ok}")
    print("  widths:", [round(by[s].width_m,2) for s in ids])