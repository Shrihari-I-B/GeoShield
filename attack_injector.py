#!/usr/bin/env python3
"""
GeoShield -- Phase 2: attack injector.

Corrupts a clean map and records exactly what it changed, giving exact,
zero-cost ground truth. This is the dual-use module: an attack tool built to
evaluate a defence.

    python3 attack_injector.py --lanelet2 map.osm --campaign --out data/
    python3 attack_injector.py --lanelet2 map.osm --attack width_ramp --seed 7

WHY RAMPS, NOT SINGLE EDITS
---------------------------
The Phase 1 width analysis on Nishi-Shinjuku measured honest lanelet-to-lanelet
width variation at p90 = 0.775 m absolute / 0.219 relative / 0.0412 m/m
gradient. Sato et al.'s smallest effective attack (+0.5 m) sits BELOW all three
90th percentiles. A single-lanelet edit is therefore statistically invisible.

Sato's own results close the loop from the other side: an abrupt single-lanelet
change to 5.0 m produced an infeasible plan and the vehicle simply stopped --
a failed attack.

So the only attack that is both *effective* and *stealthy* is a sustained
monotonic ramp spread across consecutive lanelets. `width_ramp` is the primary
attack of this project; `width_step` exists only as the naive baseline to
demonstrate why it fails.

STEALTH AND LEAKAGE
-------------------
Every tamper parameter is randomised. If the injector always wrote
maxspeed=80, a model would learn "80 means attack" -- memorising the injector
instead of learning implausibility, scoring well on your test set and failing
on anything real. Ranges, directions and lengths are all sampled.
"""

from __future__ import annotations

import argparse
import copy
import json
import xml.etree.ElementTree as ET
import random
import sys
from dataclasses import dataclass, asdict, field
from typing import Callable, Optional

from road_segment import RoadSegment


# ----------------------------------------------------------------------
# labels
# ----------------------------------------------------------------------

@dataclass
class TamperRecord:
    """Ground truth for one modified segment."""
    segment_id: str
    attack_type: str
    field_changed: str
    original: object
    tampered: object
    severity: float                 # normalised 0-1, attack-relative
    campaign_id: str = ""
    ramp_position: Optional[int] = None      # index within a ramp, 0-based
    ramp_length: Optional[int] = None


@dataclass
class AttackResult:
    segments: list[RoadSegment]
    labels: dict[str, TamperRecord] = field(default_factory=dict)
    meta: dict = field(default_factory=dict)

    def label_vector(self) -> dict[str, int]:
        """{segment_id: 1 if tampered else 0} for every segment."""
        return {s.segment_id: int(s.segment_id in self.labels) for s in self.segments}

    def summary(self) -> dict:
        n = len(self.segments)
        k = len(self.labels)
        by_type: dict[str, int] = {}
        for r in self.labels.values():
            by_type[r.attack_type] = by_type.get(r.attack_type, 0) + 1
        return {"total": n, "tampered": k,
                "rate": round(k / n, 4) if n else 0.0,
                "by_type": by_type, **self.meta}


# ----------------------------------------------------------------------
# route sampling -- ramps need consecutive segments
# ----------------------------------------------------------------------

def walk(segments: list[RoadSegment], start_id: str, n: int,
         rng: random.Random) -> list[str]:
    """Follow successors from start_id for up to n hops."""
    by = {s.segment_id: s for s in segments}
    path, cur, seen = [start_id], start_id, {start_id}
    while len(path) < n:
        nxt = [x for x in by[cur].successors if x in by and x not in seen]
        if not nxt:
            break
        cur = rng.choice(nxt)
        path.append(cur)
        seen.add(cur)
    return path


def sample_run(segments: list[RoadSegment], n: int, rng: random.Random,
               predicate: Callable[[RoadSegment], bool] = lambda s: True,
               tries: int = 200) -> Optional[list[str]]:
    """Find a chain of n consecutive segments all satisfying `predicate`."""
    cands = [s for s in segments if predicate(s) and s.successors]
    if not cands:
        return None
    by = {s.segment_id: s for s in segments}
    for _ in range(tries):
        run = walk(segments, rng.choice(cands).segment_id, n, rng)
        if len(run) == n and all(predicate(by[i]) for i in run):
            return run
    return None


def _is_road(s: RoadSegment) -> bool:
    return s.highway_class in ("road", None) or s.source == "osm"


# ----------------------------------------------------------------------
# attacks
# ----------------------------------------------------------------------

def width_ramp(segments, rng, *, total_gain=None, n=None,
               campaign="", target=None) -> AttackResult:
    """
    PRIMARY ATTACK. Widen a run of consecutive lanelets monotonically.

    Each step stays inside honest per-segment variation, but the cumulative
    displacement is large enough to move the planned trajectory. This is the
    attack the whole detection design has to catch.

    TARGETED MODE (`target`): tamper a specific list of lanelet ids -- the
    victim's actual route -- instead of sampling one at random.

    WHY THIS MATTERS. Random sampling picked 6 lanelets out of 979 while the
    simulated route used 8, so the overlap was empty and the measured
    trajectory deviation was d_Fe = 0.083 m, i.e. the simulator's own
    run-to-run noise floor. An attacker does not sample uniformly: they know
    which road the target will drive and tamper THAT. Targeting is both the
    realistic threat model and the only way to measure end-to-end impact.
    """
    segs = copy.deepcopy(segments)
    by = {s.segment_id: s for s in segs}

    if target:
        run = [t for t in target if t in by and by[t].width_m is not None]
        if not run:
            return AttackResult(segs, {}, {"attack": "width_ramp",
                                           "status": "no target lanelet found"})
        n = len(run)
    else:
        n = n or rng.randint(4, 9)
        run = sample_run(segs, n, rng, lambda s: _is_road(s) and s.width_m)
        if run is None:
            return AttackResult(segs, {}, {"attack": "width_ramp",
                                           "status": "no run found"})
    total_gain = total_gain or rng.uniform(1.2, 3.0)     # metres, randomised

    labels = {}
    for i, sid in enumerate(run):
        s = by[sid]
        frac = (i + 1) / n                       # linear ramp
        delta = total_gain * frac
        orig = s.width_m
        s.width_m = round(orig + delta, 3)
        s.raw_tags = dict(s.raw_tags)
        s.raw_tags["_tampered_width"] = True
        labels[sid] = TamperRecord(
            segment_id=sid, attack_type="width_ramp", field_changed="width_m",
            original=orig, tampered=s.width_m,
            severity=min(delta / 3.0, 1.0), campaign_id=campaign,
            ramp_position=i, ramp_length=n)

    return AttackResult(segs, labels, {
        "attack": "width_ramp", "run_length": n,
        "targeted": bool(target),
        "total_gain_m": round(total_gain, 3),
        "per_step_m": round(total_gain / n, 3)})


def width_step(segments, rng, *, delta=None, campaign="") -> AttackResult:
    """
    NAIVE BASELINE. One lanelet, one abrupt change.

    Kept deliberately: Phase 1 showed it is below the p90 of honest variation
    (undetectable), and Sato showed abrupt changes produce infeasible plans
    (ineffective). Including it lets you *demonstrate* both claims rather than
    assert them.
    """
    segs = copy.deepcopy(segments)
    cands = [s for s in segs if _is_road(s) and s.width_m]
    if not cands:
        return AttackResult(segs, {}, {"attack": "width_step", "status": "none"})

    s = rng.choice(cands)
    delta = delta if delta is not None else rng.uniform(0.5, 2.0)
    orig = s.width_m
    s.width_m = round(orig + delta, 3)
    return AttackResult(segs, {s.segment_id: TamperRecord(
        s.segment_id, "width_step", "width_m", orig, s.width_m,
        min(delta / 3.0, 1.0), campaign)}, {
        "attack": "width_step", "delta_m": round(delta, 3)})


def speed_spoof(segments, rng, *, campaign="") -> AttackResult:
    """
    Attribute poisoning. Randomised in BOTH directions and magnitude.

    Raising a limit causes unsafe acceleration; lowering one causes unexpected
    braking. Both matter, and randomising direction stops the model learning a
    one-sided signature.
    """
    segs = copy.deepcopy(segments)
    cands = [s for s in segs if s.speed_limit_kph]
    if not cands:
        return AttackResult(segs, {}, {"attack": "speed_spoof", "status": "none"})

    s = rng.choice(cands)
    orig = s.speed_limit_kph
    factor = rng.choice([rng.uniform(1.5, 2.8), rng.uniform(0.35, 0.7)])
    s.speed_limit_kph = round(orig * factor, 1)
    return AttackResult(segs, {s.segment_id: TamperRecord(
        s.segment_id, "speed_spoof", "speed_limit_kph", orig, s.speed_limit_kph,
        min(abs(factor - 1), 1.0), campaign)}, {
        "attack": "speed_spoof", "factor": round(factor, 2)})


def oneway_flip(segments, rng, *, campaign="") -> AttackResult:
    """Topological manipulation: reverse a direction constraint."""
    segs = copy.deepcopy(segments)
    cands = [s for s in segs if s.oneway is not None]
    if not cands:
        return AttackResult(segs, {}, {"attack": "oneway_flip", "status": "none"})

    s = rng.choice(cands)
    orig = s.oneway
    s.oneway = not orig
    return AttackResult(segs, {s.segment_id: TamperRecord(
        s.segment_id, "oneway_flip", "oneway", orig, s.oneway, 1.0, campaign)},
        {"attack": "oneway_flip"})


def tunnel_bridge_flip(segments, rng, *, campaign="") -> AttackResult:
    """Semantic label flip. OSM only -- Lanelet2 has no tunnel/bridge concept."""
    segs = copy.deepcopy(segments)
    cands = [s for s in segs if s.tunnel or s.bridge]
    if not cands:
        return AttackResult(segs, {}, {"attack": "tunnel_bridge_flip", "status": "none"})

    s = rng.choice(cands)
    was = "tunnel" if s.tunnel else "bridge"
    s.tunnel, s.bridge = (not s.tunnel, not s.bridge)
    return AttackResult(segs, {s.segment_id: TamperRecord(
        s.segment_id, "tunnel_bridge_flip", "tunnel/bridge", was,
        "bridge" if was == "tunnel" else "tunnel", 1.0, campaign)},
        {"attack": "tunnel_bridge_flip"})


def connectivity_break(segments, rng, *, campaign="") -> AttackResult:
    """Sever a graph edge, making a route segment unreachable."""
    segs = copy.deepcopy(segments)
    cands = [s for s in segs if s.successors]
    if not cands:
        return AttackResult(segs, {}, {"attack": "connectivity_break", "status": "none"})

    s = rng.choice(cands)
    orig = list(s.successors)
    dropped = rng.choice(orig)
    s.successors = [x for x in orig if x != dropped]
    return AttackResult(segs, {s.segment_id: TamperRecord(
        s.segment_id, "connectivity_break", "successors", orig, s.successors,
        1.0, campaign)}, {"attack": "connectivity_break", "dropped": dropped})


ATTACKS: dict[str, Callable] = {
    "width_ramp": width_ramp,
    "width_step": width_step,
    "speed_spoof": speed_spoof,
    "oneway_flip": oneway_flip,
    "tunnel_bridge_flip": tunnel_bridge_flip,
    "connectivity_break": connectivity_break,
}


# ----------------------------------------------------------------------
# campaigns -- many attacks over one map, to a stated budget
# ----------------------------------------------------------------------

def campaign(segments: list[RoadSegment], rng: random.Random,
             budget: float = 0.03,
             mix: Optional[dict[str, float]] = None) -> AttackResult:
    """
    Apply attacks until `budget` (fraction of segments) is tampered.

    BUDGET IS A DECLARED ASSUMPTION, NOT A FREE PARAMETER. It sets the class
    imbalance, which in turn sets the Isolation Forest contamination value and
    determines which evaluation metrics are honest. 3% reflects a targeted
    adversary hitting a specific corridor, not blanket corruption -- state this
    in the report and vary it in a sensitivity experiment.
    """
    mix = mix or {"width_ramp": 0.40, "speed_spoof": 0.20, "oneway_flip": 0.15,
                  "width_step": 0.10, "connectivity_break": 0.10,
                  "tunnel_bridge_flip": 0.05}

    cur = AttackResult(copy.deepcopy(segments), {}, {})
    target = int(len(segments) * budget)
    names, weights = list(mix), list(mix.values())
    applied, guard = [], 0

    while len(cur.labels) < target and guard < 500:
        guard += 1
        name = rng.choices(names, weights=weights)[0]
        cid = f"c{len(applied):03d}"
        res = ATTACKS[name](cur.segments, rng, campaign=cid)
        if not res.labels:
            continue
        # don't double-tamper a segment: keeps labels unambiguous
        if any(sid in cur.labels for sid in res.labels):
            continue
        cur.segments = res.segments
        cur.labels.update(res.labels)
        applied.append({"id": cid, **res.meta})

    cur.meta = {"budget": budget, "attacks_applied": len(applied),
                "campaign_log": applied}
    return cur


# ----------------------------------------------------------------------

def save(res: AttackResult, prefix: str) -> None:
    with open(f"{prefix}_segments.json", "w") as fh:
        json.dump([s.to_dict() for s in res.segments], fh, indent=2)
    with open(f"{prefix}_labels.json", "w") as fh:
        json.dump({k: asdict(v) for k, v in res.labels.items()}, fh, indent=2)
    with open(f"{prefix}_meta.json", "w") as fh:
        json.dump(res.summary(), fh, indent=2)


# ----------------------------------------------------------------------
# writing a genuinely tampered Lanelet2 map, for the simulator
# ----------------------------------------------------------------------

def write_tampered_map(in_path: str, out_path: str,
                       res: "AttackResult") -> dict:
    """
    Apply the tamper records to real Lanelet2 XML.

    Width is GEOMETRY, not a tag: to widen a lanelet you must physically move
    its boundary nodes. We displace the LEFT boundary outward along the local
    normal, which is also what a Vector Map Builder edit looks like in practice
    and leaves a left/right asymmetry that Tier 1 can exploit.

    Attribute attacks (speed, oneway) are simple tag rewrites on the relation.
    """
    tree = ET.parse(in_path)
    root = tree.getroot()

    nodes = {int(n.get("id")): n for n in root.findall("node")}
    ways = {int(w.get("id")): [int(nd.get("ref")) for nd in w.findall("nd")]
            for w in root.findall("way")}
    rels = {int(r.get("id")): r for r in root.findall("relation")}

    left_of = {}
    for rid, rel in rels.items():
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        if tags.get("type") != "lanelet":
            continue
        for mem in rel.findall("member"):
            if mem.get("role") == "left":
                left_of[rid] = int(mem.get("ref"))

    def xy(nid):
        n = nodes.get(nid)
        if n is None:
            return None
        d = {t.get("k"): t.get("v") for t in n.findall("tag")}
        try:
            return float(d["local_x"]), float(d["local_y"])
        except (KeyError, TypeError, ValueError):
            return None

    def set_xy(nid, x, y):
        for t in nodes[nid].findall("tag"):
            if t.get("k") == "local_x":
                t.set("v", f"{x:.4f}")
            elif t.get("k") == "local_y":
                t.set("v", f"{y:.4f}")

    def set_tag(rel, k, v):
        for t in rel.findall("tag"):
            if t.get("k") == k:
                t.set("v", str(v))
                return
        ET.SubElement(rel, "tag", k=k, v=str(v))

    stats = {"geometry": 0, "tags": 0, "skipped": 0}

    for sid, rec in res.labels.items():
        if not sid.startswith("lanelet:"):
            stats["skipped"] += 1
            continue
        lid = int(sid.split(":")[1])
        rel = rels.get(lid)
        if rel is None:
            stats["skipped"] += 1
            continue

        if rec.field_changed == "width_m":
            wid = left_of.get(lid)
            if wid is None or wid not in ways:
                stats["skipped"] += 1
                continue
            shift = float(rec.tampered) - float(rec.original)
            pts = ways[wid]
            for i, nid in enumerate(pts):
                p = xy(nid)
                if p is None:
                    continue
                q = xy(pts[min(i + 1, len(pts) - 1)]) or p
                r = xy(pts[max(i - 1, 0)]) or p
                tx, ty = q[0] - r[0], q[1] - r[1]
                mag = (tx * tx + ty * ty) ** 0.5
                if mag < 1e-9:
                    continue
                nx, ny = -ty / mag, tx / mag        # left-hand normal
                set_xy(nid, p[0] + nx * shift, p[1] + ny * shift)
            stats["geometry"] += 1

        elif rec.field_changed == "speed_limit_kph":
            set_tag(rel, "speed_limit", rec.tampered)
            stats["tags"] += 1
        elif rec.field_changed == "oneway":
            set_tag(rel, "one_way", "yes" if rec.tampered else "no")
            stats["tags"] += 1
        else:
            stats["skipped"] += 1

    tree.write(out_path, encoding="utf-8", xml_declaration=True)
    return stats


def main() -> None:
    ap = argparse.ArgumentParser(description="GeoShield attack injector")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--lanelet2", help="Lanelet2 .osm file")
    src.add_argument("--osm-area", help="built-in Overpass area name")
    ap.add_argument("--attack", choices=sorted(ATTACKS), help="single attack")
    ap.add_argument("--campaign", action="store_true", help="mixed campaign")
    ap.add_argument("--budget", type=float, default=0.03)
    ap.add_argument("--seed", type=int, default=42, help="reproducibility")
    ap.add_argument("--target", help="comma-separated lanelet ids to tamper "
                    "(e.g. the route the victim will drive)")
    ap.add_argument("--total-gain", type=float, default=None, dest="total_gain",
                    help="total width gain in metres across the ramp")
    ap.add_argument("--out", help="output prefix")
    ap.add_argument("--out-map", dest="out_map",
                    help="write a tampered Lanelet2 .osm (needs --lanelet2)")
    a = ap.parse_args()

    rng = random.Random(a.seed)

    if a.lanelet2:
        from lanelet2_adapter import load as l2load
        segs = l2load(a.lanelet2)
    else:
        from osm_adapter import AREAS, load as osmload
        segs = osmload(AREAS[a.osm_area])
    print(f"loaded {len(segs)} clean segments", file=sys.stderr)

    if a.campaign:
        res = campaign(segs, rng, a.budget)
    elif a.attack:
        kw = {}
        if a.target:
            ids = [t.strip() for t in a.target.split(",") if t.strip()]
            # accept bare numbers or full "lanelet:NNN" ids
            kw["target"] = [t if t.startswith("lanelet:") else f"lanelet:{t}"
                            for t in ids]
        if a.total_gain is not None:
            kw["total_gain"] = a.total_gain
        try:
            res = ATTACKS[a.attack](segs, rng, **kw)
        except TypeError:
            # attacks other than width_ramp do not accept these
            res = ATTACKS[a.attack](segs, rng)
    else:
        ap.error("give --attack or --campaign")

    s = res.summary()
    print(f"\n=== Injection (seed {a.seed}) ===")
    print(f"segments   : {s['total']}")
    print(f"tampered   : {s['tampered']}  ({100*s['rate']:.2f}%)")
    if s["by_type"]:
        print("\nby attack type:")
        for k, v in sorted(s["by_type"].items(), key=lambda kv: -kv[1]):
            print(f"  {k:<22}{v:>5}")

    ex = list(res.labels.values())[:5]
    if ex:
        print("\nsample modifications:")
        for r in ex:
            pos = f"  [ramp {r.ramp_position+1}/{r.ramp_length}]" if r.ramp_position is not None else ""
            print(f"  {r.segment_id:<18}{r.field_changed:<16}"
                  f"{r.original} -> {r.tampered}{pos}")

    if a.out_map:
        if not a.lanelet2:
            ap.error("--out-map requires --lanelet2")
        st = write_tampered_map(a.lanelet2, a.out_map, res)
        print(f"\nmap -> {a.out_map}")
        print(f"  geometry modified : {st['geometry']} lanelets")
        print(f"  tags modified     : {st['tags']} lanelets")
        print(f"  skipped           : {st['skipped']}")

    if a.out:
        save(res, a.out)
        print(f"\nwrote {a.out}_segments.json / _labels.json / _meta.json", file=sys.stderr)


if __name__ == "__main__":
    main()