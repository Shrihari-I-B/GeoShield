"""Shared Lanelet2 node-geometry helpers for anything that writes a map.

WHY THIS MODULE EXISTS. Autoware's map loader projects from lat/lon (MGRS by
default; these maps ship no map_projector_info.yaml) and never reads
local_x/local_y. A writer that displaces only the metric tags produces a file
the simulator treats as geometrically clean.

Measured 2026-10-19 on route_g3.0.osm: 36936 nodes compared, 0 differing in
lat/lon, 178 differing in local_x/local_y. The vehicle deviated 0.0266 m
against a 0.0400 m run-to-run noise floor -- every Frechet figure from a
written map until that date compared a clean run against a clean run. The same
defect was found independently in repair.py, where it would have shown
GeoShield failing to repair a map it had in fact repaired correctly.

Every map writer must obtain its setter from make_setter(). Do not hand-write
local_x/local_y again.
"""


def calibrate(nodes):
    """Least-squares fit of (local_x, local_y) -> (lat, lon) from this map.

    Fitted rather than assumed, so it holds for whatever projection the map
    uses, in either axis order. Over a map this size it is linear to well under
    a millimetre across the few metres any displacement spans; the reported
    residual is the absolute fit error over the whole map and is not the error
    on a delta. Returns None if the map lacks usable node pairs.
    """
    import numpy as np
    A, B = [], []
    for n in nodes.values():
        if n.get("lat") is None or n.get("lon") is None:
            continue
        d = {t.get("k"): t.get("v") for t in n.findall("tag")}
        try:
            x, y = float(d["local_x"]), float(d["local_y"])
        except (KeyError, TypeError, ValueError):
            continue
        A.append((1.0, x, y))
        B.append((float(n.get("lat")), float(n.get("lon"))))
    if len(A) < 50:
        return None
    A, B = np.array(A), np.array(B)
    coef, *_ = np.linalg.lstsq(A, B, rcond=None)
    res = np.abs(A @ coef - B).max(axis=0)
    return {"a0": coef[0, 0], "a1": coef[1, 0], "a2": coef[2, 0],
            "b0": coef[0, 1], "b1": coef[1, 1], "b2": coef[2, 1],
            "res_lat": float(res[0]), "res_lon": float(res[1])}


def to_latlon(cal, x, y):
    """Absolute (lat, lon) for a metric position.

    For writers that CREATE nodes rather than displace existing ones.
    centerline_attack.py stamped every node it created with lat="0" lon="0",
    which places it ~10000 km from the map; Autoware projects from lat/lon, so
    such a map cannot load as intended. Use this instead.
    """
    if cal is None:
        raise ValueError("no calibration available for this map")
    return (cal["a0"] + cal["a1"] * x + cal["a2"] * y,
            cal["b0"] + cal["b1"] * x + cal["b2"] * y)


def make_setter(nodes, xy, cal):
    """Return set_xy(nid, x, y) that writes local_x/local_y AND lat/lon.

    The lat/lon delta is derived from the node's own current position on every
    call, so repeated or iterative displacement (repair.py damps over several
    passes) accumulates identically in both representations.
    """
    def set_xy(nid, x, y):
        n = nodes[nid]
        old = xy(nid)
        for t in n.findall("tag"):
            if t.get("k") == "local_x":
                t.set("v", f"{x:.4f}")
            elif t.get("k") == "local_y":
                t.set("v", f"{y:.4f}")
        if old is None or cal is None:
            return
        dx, dy = x - old[0], y - old[1]
        n.set("lat", f"{float(n.get('lat')) + cal['a1']*dx + cal['a2']*dy:.12f}")
        n.set("lon", f"{float(n.get('lon')) + cal['b1']*dx + cal['b2']*dy:.12f}")
    return set_xy
