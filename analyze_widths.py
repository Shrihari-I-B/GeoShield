import sys, statistics, json
sys.path.insert(0,'.')
from lanelet2_adapter import load

segs = load(sys.argv[1])
by = {s.segment_id: s for s in segs}

def pctl(v,p):
    v=sorted(v); return v[min(int(p/100*len(v)), len(v)-1)]

for label, keep in [("ALL subtypes", lambda s: True),
                    ("road only",   lambda s: s.highway_class=="road")]:
    sel=[s for s in segs if keep(s) and s.width_m]
    abs_d, rel_d, grad = [], [], []
    for s in sel:
        for sid in s.successors:
            t=by.get(sid)
            if not t or not t.width_m or not keep(t): continue
            d=abs(s.width_m-t.width_m)
            abs_d.append(d)
            rel_d.append(d/s.width_m)
            L=max(s.length_m,1.0)
            grad.append(d/L)
    if not abs_d: continue
    print(f"\n=== {label}  ({len(sel)} lanelets, {len(abs_d)} pairs) ===")
    print(f"{'metric':<22}{'median':>9}{'p90':>9}{'p95':>9}{'p99':>9}")
    for name,v,f in [("abs delta [m]",abs_d,"{:.3f}"),
                     ("rel delta [frac]",rel_d,"{:.3f}"),
                     ("gradient [m/m]",grad,"{:.4f}")]:
        print(f"{name:<22}"+"".join(f"{f.format(pctl(v,p)):>9}" for p in (50,90,95,99)))

# what would Sato's attack look like in gradient terms?
roads=[s for s in segs if s.highway_class=="road" and s.width_m]
L=statistics.median(s.length_m for s in roads)
print(f"\nmedian road lanelet length: {L:.1f} m")
print(f"Sato +0.5 m over that length -> gradient {0.5/L:.4f} m/m")
print(f"Sato +0.5 m relative to 3.0 m lane -> rel delta 0.167")