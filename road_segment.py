"""
GeoShield -- common road segment schema.

Both the OSM adapter (Track A) and the Lanelet2 adapter (Track B) normalise
into this single record, so the detection core is written once and never needs
to know which map format it is looking at.

Fields that a given source cannot supply are left as None. Detectors must treat
None as "unknown", never as a value -- the `missing()` helper exists so that
missingness itself can be handed to the ML layer as a feature.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, asdict
from typing import Optional, Sequence

EARTH_RADIUS_M = 6_371_000.0


def haversine_m(a: tuple[float, float], b: tuple[float, float]) -> float:
    """Great-circle distance in metres between two (lat, lon) pairs."""
    lat1, lon1 = math.radians(a[0]), math.radians(a[1])
    lat2, lon2 = math.radians(b[0]), math.radians(b[1])
    dlat, dlon = lat2 - lat1, lon2 - lon1
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(h))


def _bearing(a: tuple[float, float], b: tuple[float, float]) -> float:
    """Initial bearing in radians from point a to point b."""
    lat1, lat2 = math.radians(a[0]), math.radians(b[0])
    dlon = math.radians(b[1] - a[1])
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    return math.atan2(x, y)


@dataclass
class Provenance:
    """
    Edit history of the source element.

    Only OSM supplies this (via Overpass `out meta`). It is what lets GeoShield
    decide *which side is lying* when the HD map and OSM disagree: a way edited
    three days ago by a low-history account is a suspect witness, whereas one
    untouched for four years is a credible one.
    """

    version: Optional[int] = None
    timestamp: Optional[str] = None       # ISO 8601
    changeset: Optional[int] = None
    user: Optional[str] = None
    uid: Optional[int] = None


@dataclass
class RoadSegment:
    # --- identity ---
    segment_id: str
    source: str                            # "osm" | "lanelet2"

    # --- geometry: ordered (lat, lon) vertices ---
    geometry: list[tuple[float, float]] = field(default_factory=list)

    # --- semantic attributes (None == unknown, NOT a default) ---
    speed_limit_kph: Optional[float] = None
    oneway: Optional[bool] = None
    tunnel: Optional[bool] = None
    bridge: Optional[bool] = None
    lanes: Optional[int] = None
    width_m: Optional[float] = None        # Lanelet2 supplies directly; OSM rarely
    highway_class: Optional[str] = None    # "residential", "primary", ...

    # --- topology: filled in by the graph builder, not the adapter ---
    predecessors: list[str] = field(default_factory=list)
    successors: list[str] = field(default_factory=list)

    # --- audit trail ---
    provenance: Provenance = field(default_factory=Provenance)
    raw_tags: dict = field(default_factory=dict)

    # ------------------------------------------------------------------
    # derived geometry
    # ------------------------------------------------------------------

    @property
    def length_m(self) -> float:
        if len(self.geometry) < 2:
            return 0.0
        return sum(
            haversine_m(self.geometry[i], self.geometry[i + 1])
            for i in range(len(self.geometry) - 1)
        )

    @property
    def mean_curvature(self) -> Optional[float]:
        """
        Mean absolute heading change per metre (rad/m).

        Used by the Tier-1 speed-plausibility rule: a segment declaring 80 km/h
        through a curve whose implied lateral acceleration exceeds ~3 m/s^2 is
        not physically drivable, whatever its tags claim.
        """
        if len(self.geometry) < 3:
            return None
        total_turn, total_len = 0.0, 0.0
        for i in range(len(self.geometry) - 2):
            b1 = _bearing(self.geometry[i], self.geometry[i + 1])
            b2 = _bearing(self.geometry[i + 1], self.geometry[i + 2])
            d = (b2 - b1 + math.pi) % (2 * math.pi) - math.pi
            total_turn += abs(d)
            total_len += haversine_m(self.geometry[i], self.geometry[i + 1])
        return (total_turn / total_len) if total_len > 0 else None

    @property
    def implied_radius_m(self) -> Optional[float]:
        c = self.mean_curvature
        return (1.0 / c) if c and c > 1e-9 else None

    def max_safe_speed_kph(self, a_lat_max: float = 3.0) -> Optional[float]:
        """Speed at which lateral acceleration hits a_lat_max on this curve."""
        r = self.implied_radius_m
        if r is None:
            return None
        return math.sqrt(a_lat_max * r) * 3.6

    # ------------------------------------------------------------------
    # helpers for the ML layer
    # ------------------------------------------------------------------

    def missing(self) -> dict[str, bool]:
        """Missingness indicators -- fed to the model alongside the values."""
        return {
            f"missing_{k}": getattr(self, k) is None
            for k in ("speed_limit_kph", "oneway", "tunnel", "bridge",
                      "lanes", "width_m", "highway_class")
        }

    def to_dict(self) -> dict:
        d = asdict(self)
        d["length_m"] = self.length_m
        d["mean_curvature"] = self.mean_curvature
        d.update(self.missing())
        return d


def summarise(segments: Sequence[RoadSegment]) -> dict:
    """Attribute-density report -- the Phase 0 go/no-go check."""
    n = len(segments)
    if n == 0:
        return {"count": 0}
    keys = ["speed_limit_kph", "oneway", "tunnel", "bridge",
            "lanes", "width_m", "highway_class"]
    out = {"count": n, "total_km": round(sum(s.length_m for s in segments) / 1000, 2)}
    for k in keys:
        present = sum(1 for s in segments if getattr(s, k) is not None)
        out[k] = {"present": present, "pct": round(100 * present / n, 1)}
    return out
