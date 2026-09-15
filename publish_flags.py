#!/usr/bin/env python3
"""
GeoShield -- publish 3D RViz markers for flagged lanelets.

    python3 publish_flags.py \
        --report results/diff_g3.0.json \
        --map ~/autoware_map/nishishinjuku_autoware_map/lanelet2_map.osm \
        --topic /geoshield/flagged_lanelets

Reads the differential verification JSON report and map XML, then publishes a
visualization_msgs/msg/MarkerArray in the 'map' frame with TRANSIENT_LOCAL
durability so RViz receives them regardless of when it subscribes.

Markers published per flagged lanelet:
  1. Vertical 3D pillar (Red for REJECT, Orange for SUSPECT)
  2. 3D Boundary lines highlighting the corridor
  3. Floating text banner with lanelet ID, verdict, and measured delta
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


def load_lanelet_geometry(map_path: str):
    """Parse 3D nodes and lanelet left/right boundary paths from XML."""
    root = ET.parse(map_path).getroot()
    nodes = {}
    for n in root.findall("node"):
        tags = {t.get("k"): t.get("v") for t in n.findall("tag")}
        try:
            x = float(tags["local_x"])
            y = float(tags["local_y"])
            z = float(tags.get("ele", 0.0))
            nodes[int(n.get("id"))] = (x, y, z)
        except (KeyError, TypeError, ValueError):
            pass

    ways = {
        int(w.get("id")): [int(nd.get("ref")) for nd in w.findall("nd")]
        for w in root.findall("way")
    }

    lanelets = {}
    for rel in root.findall("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        if tags.get("type") != "lanelet":
            continue
        lid = int(rel.get("id"))
        left_pts = []
        right_pts = []
        for mem in rel.findall("member"):
            role = mem.get("role")
            ref = int(mem.get("ref"))
            if role == "left" and ref in ways:
                left_pts = [nodes[nid] for nid in ways[ref] if nid in nodes]
            elif role == "right" and ref in ways:
                right_pts = [nodes[nid] for nid in ways[ref] if nid in nodes]

        if left_pts and right_pts:
            all_pts = left_pts + right_pts
            cx = sum(p[0] for p in all_pts) / len(all_pts)
            cy = sum(p[1] for p in all_pts) / len(all_pts)
            cz = sum(p[2] for p in all_pts) / len(all_pts)
            lanelets[lid] = {
                "left": left_pts,
                "right": right_pts,
                "center": (cx, cy, cz),
            }
    return lanelets


def build_marker_array(report_data: dict, lanelets: dict) -> MarkerArray:
    """Construct visualization_msgs/MarkerArray for all flagged lanelets."""
    markers = MarkerArray()
    changes = report_data.get("changes", [])
    if not changes:
        return markers

    # Group changes by lanelet id
    by_lid = {}
    for ch in changes:
        sid = ch.get("segment_id", "")
        if not sid.startswith("lanelet:"):
            continue
        try:
            lid = int(sid.split(":")[1])
        except ValueError:
            continue
        by_lid.setdefault(lid, []).append(ch)

    now = rclpy.time.Time().to_msg()
    marker_id = 0

    for lid, ch_list in sorted(by_lid.items()):
        if lid not in lanelets:
            continue

        geom = lanelets[lid]
        cx, cy, cz = geom["center"]

        # Worst verdict
        is_reject = any(c.get("verdict") == "REJECT" for c in ch_list)
        verdict = "REJECT" if is_reject else "SUSPECT"

        if is_reject:
            # Vibrant Red
            color = ColorRGBA(r=0.95, g=0.15, b=0.15, a=0.85)
            color_solid = ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0)
            color_wire = ColorRGBA(r=1.0, g=0.3, b=0.3, a=0.95)
        else:
            # Vibrant Amber / Orange
            color = ColorRGBA(r=0.98, g=0.60, b=0.05, a=0.80)
            color_solid = ColorRGBA(r=1.0, g=0.65, b=0.1, a=1.0)
            color_wire = ColorRGBA(r=1.0, g=0.7, b=0.2, a=0.95)

        # ------------------------------------------------------------------
        # 1. Vertical 3D Pillar (Marker.CYLINDER)
        # ------------------------------------------------------------------
        pillar = Marker()
        pillar.header.frame_id = "map"
        pillar.header.stamp = now
        pillar.ns = "geoshield_pillars"
        pillar.id = marker_id
        marker_id += 1
        pillar.type = Marker.CYLINDER
        pillar.action = Marker.ADD
        pillar.pose.position.x = cx
        pillar.pose.position.y = cy
        pillar.pose.position.z = cz + 4.0  # Center height of 8m cylinder
        pillar.pose.orientation.w = 1.0
        pillar.scale.x = 2.5
        pillar.scale.y = 2.5
        pillar.scale.z = 8.0
        pillar.color = color
        markers.markers.append(pillar)

        # ------------------------------------------------------------------
        # 2. Left and Right Boundary Lines (Marker.LINE_STRIP)
        # ------------------------------------------------------------------
        for side, pts in [("left", geom["left"]), ("right", geom["right"])]:
            line = Marker()
            line.header.frame_id = "map"
            line.header.stamp = now
            line.ns = f"geoshield_boundary_{side}"
            line.id = marker_id
            marker_id += 1
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = 0.45  # line width (m)
            line.color = color_wire
            for p in pts:
                pt = Point()
                pt.x = p[0]
                pt.y = p[1]
                pt.z = p[2] + 0.15  # slightly above ground to prevent z-fighting
                line.points.append(pt)
            markers.markers.append(line)

        # ------------------------------------------------------------------
        # 3. Floating 3D Text Banner (Marker.TEXT_VIEW_FACING)
        # ------------------------------------------------------------------
        # Summarize changes on this lanelet
        width_ch = next((c for c in ch_list if c.get("field_name") == "width_m"), None)
        cl_ch = next((c for c in ch_list if c.get("field_name") == "centreline"), None)

        text_lines = [f"LANELET {lid} [{verdict}]"]
        if width_ch:
            d = width_ch.get("delta", 0.0)
            rel = width_ch.get("relative", 0.0)
            w_orig = width_ch.get("previous", "?")
            w_new = width_ch.get("candidate", "?")
            text_lines.append(f"Width: {w_orig}m -> {w_new}m ({d:+.2f}m, {rel*100:+.0f}%)")
        if cl_ch:
            shift = cl_ch.get("delta", 0.0)
            text_lines.append(f"Centreline shift: {shift:.2f}m")
        if not width_ch and not cl_ch:
            other = ch_list[0]
            text_lines.append(f"{other.get('field_name')}: {other.get('verdict')}")

        text_marker = Marker()
        text_marker.header.frame_id = "map"
        text_marker.header.stamp = now
        text_marker.ns = "geoshield_labels"
        text_marker.id = marker_id
        marker_id += 1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = cx
        text_marker.pose.position.y = cy
        # Stagger label height so dense clusters do not overlap. Measured:
        # nine flagged lanelets fall within a ~200 m stretch at the start of
        # the route, and at a fixed height their TEXT_VIEW_FACING banners
        # render on top of each other and become unreadable. Cycling through
        # four tiers separates them vertically while keeping every label
        # above its own pillar.
        _tier = (len(markers.markers) // 3) % 4
        text_marker.pose.position.z = cz + 9.5 + _tier * 4.0
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 1.4  # Text height
        text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.98)
        text_marker.text = "\n".join(text_lines)
        markers.markers.append(text_marker)

    return markers


class FlagPublisher(Node):
    def __init__(self, topic: str, markers: MarkerArray, rate_hz: float = 1.0):
        super().__init__("geoshield_flag_publisher")
        # Transient Local QoS: late-joining subscribers (like RViz) receive the message
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(MarkerArray, topic, qos)
        self.markers = markers
        self.rate_hz = rate_hz
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_callback)
        # Publish immediately once
        self.pub.publish(self.markers)
        self.count = 1

    def publish_callback(self):
        self.pub.publish(self.markers)
        self.count += 1


def main():
    ap = argparse.ArgumentParser(description="GeoShield RViz Flag Publisher")
    ap.add_argument("--report", default="results/diff_g3.0.json",
                    help="Differential verification report JSON")
    ap.add_argument("--map", default=os.path.expanduser(
                    "~/autoware_map/nishishinjuku_autoware_map/lanelet2_map.osm"),
                    help="Lanelet2 .osm file")
    ap.add_argument("--topic", default="/geoshield/flagged_lanelets",
                    help="MarkerArray topic name")
    ap.add_argument("--duration", type=float, default=120.0,
                    help="Publish duration in seconds (0 for indefinite)")
    ap.add_argument("--rate", type=float, default=2.0,
                    help="Publish rate in Hz")
    ap.add_argument("--once", action="store_true",
                    help="Publish once and exit")
    a = ap.parse_args()

    report_path = Path(a.report)
    if not report_path.exists():
        # Fallback search if path is relative
        cand = Path("/geoshield") / a.report
        if cand.exists():
            report_path = cand
        else:
            sys.exit(f"ERROR: Report file {a.report} not found")

    map_path = Path(a.map)
    if not map_path.exists():
        # Fallback search in /autoware_map
        cand = Path("/autoware_map/nishishinjuku_autoware_map/lanelet2_map.osm")
        if cand.exists():
            map_path = cand
        else:
            sys.exit(f"ERROR: Map file {a.map} not found")

    print(f"[GeoShield] Loading report from: {report_path}")
    report_data = json.loads(report_path.read_text())

    print(f"[GeoShield] Loading map geometry from: {map_path}")
    lanelets = load_lanelet_geometry(str(map_path))

    rclpy.init()
    markers = build_marker_array(report_data, lanelets)

    n_flagged = len(set(m.pose.position.x for m in markers.markers if m.ns == "geoshield_pillars"))
    print(f"[GeoShield] Built {len(markers.markers)} markers for {n_flagged} flagged lanelets.")
    print(f"[GeoShield] Publishing to {a.topic} (TRANSIENT_LOCAL durability)")
    print(f"            RED = REJECT, ORANGE = SUSPECT")

    node = FlagPublisher(a.topic, markers, a.rate)

    if a.once:
        time.sleep(0.5)
        node.destroy_node()
        rclpy.shutdown()
        print("[GeoShield] Published markers successfully.")
        return

    start_time = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
            if a.duration > 0 and (time.time() - start_time) > a.duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("[GeoShield] Finished publishing.")


if __name__ == "__main__":
    main()
