from __future__ import annotations
import json, math, sys, importlib.util
from pathlib import Path

def _load(name, path):
    s = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(s); s.loader.exec_module(m); return m

fa  = _load("fa",  "frechet_analysis.py")
ad  = _load("l2a", "lanelet2_adapter.py")

BAG_C, BAG_T = "data/bags/clean_0.50.0", "data/bags/g3_0.50.0_v2"
MAP, LABELS  = "data/route_g3.0.osm", "data/route_g3.0_labels.json"

ce = fa.ego_path(fa.read_bag(BAG_C, [fa.ODOM])[fa.ODOM])
te = fa.ego_path(fa.read_bag(BAG_T, [fa.ODOM])[fa.ODOM])
cu, tu, _ = fa.truncate_common(ce, te)

raw  = max(min(math.dist(p, q) for q in tu) for p in cu)
a, b = fa.resample(cu, 400), fa.resample(tu, 400)
res  = max(min(math.dist(p, q) for q in b) for p in a)
print(f"peak lateral, raw points  : {raw:.4f} m   <- compare_runs.py style")
print(f"peak lateral, resampled   : {res:.4f} m   <- frechet_analysis.py style")

segs   = {s.segment_id: s for s in ad.load(MAP)}
labels = json.load(open(LABELS))
print(f"\ntampered lanelets: {len(labels)}   map segments: {len(segs)}")

def nearest(seg, path):
    d = []
    for kx, ky in (("_cl_start_x","_cl_start_y"), ("_cl_end_x","_cl_end_y")):
        x, y = seg.raw_tags.get(kx), seg.raw_tags.get(ky)
        if x is not None:
            d.append(min(math.dist((x, y), p) for p in path))
    return min(d) if d else float("nan")

allmin = min(nearest(s, cu) for s in segs.values() if "_cl_start_x" in s.raw_tags)
print(f"nearest map lanelet to driven path: {allmin:.2f} m  (frame check)")

print(f"\n{'lanelet':<22}{'orig':>8}{'tamp':>8}{'delta%':>9}{'dist_m':>10}  on_route")
on = 0
for lid, info in labels.items():
    s = segs.get(lid)
    if s is None:
        print(f"{lid:<22}{'--- not found in map ---':>35}"); continue
    o, t = info.get("original"), info.get("tampered")
    pct  = (t - o) / o * 100 if o else float("nan")
    d    = nearest(s, cu)
    hit  = d < 15.0
    on  += hit
    print(f"{lid:<22}{o:>8.3f}{t:>8.3f}{pct:>8.1f}%{d:>10.2f}  {'YES' if hit else 'no'}")
print(f"\non route: {on}/{len(labels)}")
