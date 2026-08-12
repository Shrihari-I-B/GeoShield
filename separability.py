"""Where do clean chain trends sit vs attacked chain trends?"""
import sys, json, statistics
sys.path.insert(0,'.')
from lanelet2_adapter import load
from tier1_rules import _chains, _chain_trend

clean = load(sys.argv[1])
byc = {s.segment_id: s for s in clean}
tam = load(sys.argv[2])
byt = {s.segment_id: s for s in tam}
truth = set(json.load(open(sys.argv[3])).get("labels", json.load(open(sys.argv[3]))).keys())

def pctl(v,p):
    v=sorted(v); return v[min(int(p/100*len(v)),len(v)-1)] if v else None

for L in (3,4,5,6,7):
    ct=[t for t in (_chain_trend(c,byc) for c in _chains(clean,byc,L)) if t is not None]
    att, cln = [], []
    for c in _chains(tam,byt,L):
        t=_chain_trend(c,byt)
        if t is None: continue
        (att if any(x.segment_id in truth for x in c) else cln).append(t)
    if not att: 
        print(f"L={L}: no attacked chains found"); continue
    print(f"\nL={L}  clean chains {len(ct)}  attacked chains {len(att)}")
    print(f"  clean   p90 {pctl(ct,90):.3f}  p99 {pctl(ct,99):.3f}  p99.5 {pctl(ct,99.5):.3f}  max {max(ct):.3f}")
    print(f"  attack  min {min(att):.3f}  median {statistics.median(att):.3f}  max {max(att):.3f}")
    # best achievable separation
    best=None
    for thr in [x/100 for x in range(1,200)]:
        tp=sum(1 for t in att if t>thr); fn=len(att)-tp
        fp=sum(1 for t in cln if t>thr); 
        if tp+fp==0: continue
        p=tp/(tp+fp); r=tp/(tp+fn) if tp+fn else 0
        f=2*p*r/(p+r) if p+r else 0
        if best is None or f>best[3]: best=(thr,p,r,f)
    if best: print(f"  BEST thr {best[0]:.2f} -> precision {best[1]:.3f} recall {best[2]:.3f} F1 {best[3]:.3f}")
