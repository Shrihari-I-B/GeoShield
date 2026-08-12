"""How connected is the real map, really?"""
import sys, json
sys.path.insert(0,'.')
from lanelet2_adapter import load
sys.argv=[sys.argv[0]]+sys.argv[1:]
segs=load(sys.argv[1])
by={s.segment_id:s for s in segs}
roads=[s for s in segs if s.highway_class=="road"]
print(f"total {len(segs)}, road {len(roads)}")

from collections import Counter
c=Counter(len(s.successors) for s in roads)
print("successor count distribution:", dict(sorted(c.items())))
c2=Counter(len(s.predecessors) for s in roads)
print("predecessor count distribution:", dict(sorted(c2.items())))

# how many road lanelets have a turn_direction tag (junction marker)?
j=sum(1 for s in roads if 'turn_direction' in s.raw_tags)
print(f"lanelets with turn_direction tag: {j} ({100*j/len(roads):.1f}%)")

# longest chain reachable if we ALLOW forks (pick first successor)
def chain_len(s0, maxn=20):
    seen={s0.segment_id}; cur=s0; n=1
    while n<maxn:
        nxt=[by[t] for t in cur.successors if t in by and by[t].highway_class=="road"
             and by[t].segment_id not in seen]
        if not nxt: break
        cur=nxt[0]; seen.add(cur.segment_id); n+=1
    return n
lens=[chain_len(s) for s in roads]
print(f"chain length allowing forks: median {sorted(lens)[len(lens)//2]}, max {max(lens)}")

# strict (single successor only)
def strict_len(s0, maxn=20):
    seen={s0.segment_id}; cur=s0; n=1
    while n<maxn:
        nxt=[by[t] for t in cur.successors if t in by and by[t].highway_class=="road"
             and by[t].segment_id not in seen]
        if len(nxt)!=1: break
        cur=nxt[0]; seen.add(cur.segment_id); n+=1
    return n
slens=[strict_len(s) for s in roads]
print(f"chain length strict (no forks): median {sorted(slens)[len(slens)//2]}, max {max(slens)}")
print(f"  chains reaching length>=5 strict: {sum(1 for x in slens if x>=5)}")
print(f"  chains reaching length>=5 w/forks: {sum(1 for x in lens if x>=5)}")