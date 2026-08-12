#!/usr/bin/env python3
"""
GeoShield -- Track A adapter: Overpass API -> RoadSegment.

Run the Phase 0 attribute-density check (this is the go/no-go for the whole
cross-verification design):

    python3 osm_adapter.py --area nishi-shinjuku --density

Other built-in areas: munich-centre, berlin-mitte, sf-soma
Or pass your own:  --bbox SOUTH,WEST,NORTH,EAST

Save the parsed segments for later phases:

    python3 osm_adapter.py --area nishi-shinjuku --out segments.json

Only dependency is `requests`.
"""

from __future__ import annotations

import argparse
import json
import re
import hashlib
import sys
import time
from pathlib import Path
from typing import Optional

from road_segment import RoadSegment, Provenance, summarise

OVERPASS_ENDPOINTS = [
    "https://overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
    "https://overpass.osm.ch/api/interpreter",
    "https://overpass.private.coffee/api/interpreter",
]

# Overpass returns 406 Not Acceptable to clients with a generic user agent.
# Identify the tool properly -- this is also required by OSM's usage policy.
HEADERS = {
    "User-Agent": "GeoShield/0.1 (final-year research project; contact via GitHub)",
    "Accept": "application/json",
}

CACHE_DIR = Path.home() / ".cache" / "geoshield" / "overpass"

# south, west, north, east
AREAS = {
    "nishi-shinjuku": (35.6850, 139.6890, 35.7010, 139.7020),  # matches the AWSIM map
    "munich-centre":  (48.1300, 11.5500, 48.1500, 11.5900),
    "berlin-mitte":   (52.5100, 13.3800, 52.5250, 13.4100),
    "sf-soma":        (37.7700, -122.4150, 37.7850, -122.3900),
}

# Drivable roads only. Footpaths and cycleways have no speed limits to tamper
# with and would inflate the "missing attribute" counts misleadingly.
DRIVABLE = ("motorway|trunk|primary|secondary|tertiary|unclassified|residential"
            "|living_street|service|motorway_link|trunk_link|primary_link"
            "|secondary_link|tertiary_link")

QUERY = """
[out:json][timeout:{timeout}];
way["highway"~"^({classes})$"]({s},{w},{n},{e});
out meta geom;
"""


# ----------------------------------------------------------------------
# tag parsing -- OSM tags are free text, so every parser must fail to None
# ----------------------------------------------------------------------

_MPH = re.compile(r"^\s*([\d.]+)\s*mph\s*$", re.I)
_KPH = re.compile(r"^\s*([\d.]+)\s*(km/h|kph|kmh)?\s*$", re.I)


def parse_maxspeed(v: Optional[str]) -> Optional[float]:
    """'50' -> 50.0 | '30 mph' -> 48.3 | 'DE:urban' -> None (implicit, not stated)."""
    if not v:
        return None
    if m := _MPH.match(v):
        return round(float(m.group(1)) * 1.609344, 1)
    if m := _KPH.match(v):
        return float(m.group(1))
    return None  # implicit/zone values are deliberately not resolved here


def parse_oneway(tags: dict) -> Optional[bool]:
    v = tags.get("oneway")
    if v in ("yes", "true", "1", "-1"):
        return True
    if v in ("no", "false", "0"):
        return False
    if v is None and tags.get("junction") in ("roundabout", "circular"):
        return True  # implied by OSM convention
    return None


def parse_bool_tag(tags: dict, key: str) -> Optional[bool]:
    v = tags.get(key)
    if v is None:
        return None
    return v not in ("no", "false", "0")


def parse_int(v: Optional[str]) -> Optional[int]:
    try:
        return int(float(v))
    except (TypeError, ValueError):
        return None


def parse_float(v: Optional[str]) -> Optional[float]:
    try:
        return float(re.sub(r"[^\d.]", "", v))
    except (TypeError, ValueError, AttributeError):
        return None


# ----------------------------------------------------------------------
# fetch + convert
# ----------------------------------------------------------------------

def build_query(bbox, timeout: int = 180, simple: bool = False) -> str:
    s, w, n, e = bbox
    classes = ".*" if simple else DRIVABLE
    return QUERY.format(timeout=timeout, classes=classes, s=s, w=w, n=n, e=e)


def fetch(bbox, timeout: int = 180, use_cache: bool = True,
          simple: bool = False) -> dict:
    """Query Overpass with retries, endpoint failover and an on-disk cache."""
    import requests

    q = build_query(bbox, timeout, simple)
    key = hashlib.sha256(q.encode()).hexdigest()[:16]
    cache = CACHE_DIR / f"{key}.json"

    if use_cache and cache.exists():
        print(f"  cache hit: {cache}", file=sys.stderr)
        return json.loads(cache.read_text())

    last = None
    for attempt in range(3):
        for url in OVERPASS_ENDPOINTS:
            try:
                print(f"  [try {attempt+1}] {url}", file=sys.stderr)
                r = requests.post(url, data={"data": q}, headers=HEADERS,
                                  timeout=timeout + 60)
                if r.status_code in (429, 504):        # busy / timed out
                    print(f"    busy ({r.status_code})", file=sys.stderr)
                    last = f"{r.status_code} from {url}"
                    continue
                r.raise_for_status()
                data = r.json()
                if use_cache:
                    CACHE_DIR.mkdir(parents=True, exist_ok=True)
                    cache.write_text(json.dumps(data))
                    print(f"  cached -> {cache}", file=sys.stderr)
                return data
            except Exception as exc:                    # noqa: BLE001
                last = exc
                print(f"    failed: {exc}", file=sys.stderr)
        wait = 5 * (2 ** attempt)                       # 5s, 10s, 20s
        print(f"  all endpoints busy, waiting {wait}s ...", file=sys.stderr)
        time.sleep(wait)

    raise RuntimeError(
        f"Overpass unreachable after 3 rounds; last error: {last}\n"
        f"Overpass is often busy at peak times -- retry in a few minutes, or run\n"
        f"  python3 osm_adapter.py --area <area> --dry-run\n"
        f"and paste the printed query into https://overpass-turbo.eu"
    )


def element_to_segment(el: dict) -> Optional[RoadSegment]:
    if el.get("type") != "way" or "geometry" not in el:
        return None
    tags = el.get("tags", {}) or {}
    geom = [(p["lat"], p["lon"]) for p in el["geometry"]]
    if len(geom) < 2:
        return None

    return RoadSegment(
        segment_id=f"osm:way:{el['id']}",
        source="osm",
        geometry=geom,
        speed_limit_kph=parse_maxspeed(tags.get("maxspeed")),
        oneway=parse_oneway(tags),
        tunnel=parse_bool_tag(tags, "tunnel"),
        bridge=parse_bool_tag(tags, "bridge"),
        lanes=parse_int(tags.get("lanes")),
        width_m=parse_float(tags.get("width")),
        highway_class=tags.get("highway"),
        provenance=Provenance(
            version=el.get("version"),
            timestamp=el.get("timestamp"),
            changeset=el.get("changeset"),
            user=el.get("user"),
            uid=el.get("uid"),
        ),
        raw_tags=tags,
    )


def load(bbox, **kw) -> list[RoadSegment]:
    data = fetch(bbox, **kw)
    segs = [s for s in (element_to_segment(e) for e in data.get("elements", [])) if s]
    return segs


# ----------------------------------------------------------------------

def print_density(segments: list[RoadSegment], label: str) -> None:
    rep = summarise(segments)
    print(f"\n=== Attribute density: {label} ===")
    print(f"drivable ways : {rep['count']}")
    print(f"total length  : {rep['total_km']} km\n")
    print(f"{'attribute':<18}{'present':>9}{'pct':>9}   verdict")
    print("-" * 56)
    for k in ("speed_limit_kph", "oneway", "tunnel", "bridge",
              "lanes", "width_m", "highway_class"):
        st = rep[k]
        pct = st["pct"]
        verdict = "good" if pct >= 60 else ("usable" if pct >= 25 else "TOO SPARSE")
        print(f"{k:<18}{st['present']:>9}{pct:>8}%   {verdict}")

    ms = rep["speed_limit_kph"]["pct"]
    ow = rep["oneway"]["pct"]
    print()
    if ms >= 25 and ow >= 40:
        print("VERDICT: cross-verification is viable here. Proceed to Phase 1.")
    else:
        print("VERDICT: too sparse for speed/oneway cross-checks in this box.")
        print("         Try a European city, or lean harder on the")
        print("         self-consistency rules that need no external witness.")


def main() -> None:
    ap = argparse.ArgumentParser(description="GeoShield Overpass adapter")
    ap.add_argument("--area", choices=sorted(AREAS), help="built-in bounding box")
    ap.add_argument("--bbox", help="SOUTH,WEST,NORTH,EAST")
    ap.add_argument("--density", action="store_true", help="print the Phase 0 report")
    ap.add_argument("--out", help="write parsed segments to this JSON file")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the Overpass query and exit (paste into overpass-turbo.eu)")
    ap.add_argument("--no-cache", action="store_true", help="ignore the on-disk cache")
    ap.add_argument("--simple", action="store_true",
                    help="fetch ALL highway types, not just drivable ones")
    a = ap.parse_args()

    if a.bbox:
        bbox, label = tuple(float(x) for x in a.bbox.split(",")), a.bbox
    elif a.area:
        bbox, label = AREAS[a.area], a.area
    else:
        ap.error("give --area or --bbox")

    if a.dry_run:
        print(build_query(bbox, simple=a.simple))
        return

    print(f"fetching {label} {bbox}", file=sys.stderr)
    segs = load(bbox, use_cache=not a.no_cache, simple=a.simple)
    print(f"parsed {len(segs)} drivable segments", file=sys.stderr)

    if a.density or not a.out:
        print_density(segs, label)
    if a.out:
        with open(a.out, "w") as fh:
            json.dump([s.to_dict() for s in segs], fh, indent=2)
        print(f"\nwrote {a.out}", file=sys.stderr)


if __name__ == "__main__":
    main()
