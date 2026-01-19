# Copyright 2025 Eemil Kulmala, University of Oulu
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from collections import deque

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from tractor_safety_system_interfaces.msg import FusedDetection, FusedDetectionArray


class TrackViz(Node):

    def __init__(self):
        super().__init__("track_viz")
        # Tracked detections (output of tracker)
        self.sub_tracks = self.create_subscription(
            FusedDetectionArray, "/tracked_detections", self.on_track, 50
        )
        # Fused detections (output of fusion node, pre-tracking)
        self.sub_fused = self.create_subscription(
            FusedDetection, "/fused_detections", self.on_fused, 50
        )
        self.pub = self.create_publisher(MarkerArray, "/tracked_markers", 10)

        # params
        self.declare_parameter("dot_size", 0.6)  # meters
        self.declare_parameter("trail_len", 30)  # points kept per track
        self.declare_parameter("ttl_sec", 1.5)  # delete if no update in this time
        self.declare_parameter("keepalive_hz", 2.0)  # republish unchanged at this rate
        # Fused detections (pre-tracks) visualization
        self.declare_parameter("show_fused", True)  # show fused detections as points
        self.declare_parameter("fused_ttl_sec", 0.5)  # how long to keep fused points

        self.dot_size = float(self.get_parameter("dot_size").value)
        self.trail_len = int(self.get_parameter("trail_len").value)
        self.ttl = float(self.get_parameter("ttl_sec").value)
        self.keepalive_dt = 1.0 / max(
            0.1, float(self.get_parameter("keepalive_hz").value)
        )
        self.show_fused = bool(self.get_parameter("show_fused").value)
        self.fused_ttl = float(self.get_parameter("fused_ttl_sec").value)

        # state
        self.tracks = {}  # tid -> dict(x,y,z,frame,last,trail: deque[Point], last_pub)
        self.id_map = {}  # tid -> stable int id for markers
        self.next_id = 1

        # Simple buffer of recent fused detections (pre-tracks)
        # each entry: dict(x,y,z,frame,t)
        self.fused_points = []

        self.timer = self.create_timer(0.1, self.on_timer)  # 10 Hz

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _get_id(self, tid: str) -> int:
        if tid not in self.id_map:
            self.id_map[tid] = self.next_id
            self.next_id += 1
        return self.id_map[tid]

    def _color_for(self, tid: str):
        # deterministic pastel-ish color from string
        h = abs(hash(tid)) % (256 * 256 * 256)
        r = ((h >> 16) & 255) / 255.0
        g = ((h >> 8) & 255) / 255.0
        b = (h & 255) / 255.0
        # lighten a bit
        r = 0.3 + 0.7 * r
        g = 0.3 + 0.7 * g
        b = 0.3 + 0.7 * b
        return r, g, b

    def on_fused(self, msg: FusedDetection):
        """Receive fused detections (from /fused_detections) and store recent points."""
        if not self.show_fused:
            return
        t = self._now()
        frame = msg.header.frame_id or "base_link"
        self.fused_points.append(
            {
                "x": msg.position.x,
                "y": msg.position.y,
                "z": msg.position.z,
                "frame": frame,
                "t": t,
            }
        )

    def on_track(self, detections: FusedDetectionArray):
        for msg in detections.detections:
            tid = msg.tracking_id or "unknown"
            t = self._now()
            rec = self.tracks.get(tid)
            p = Point(x=msg.position.x, y=msg.position.y, z=msg.position.z)
            if rec is None:
                trail = deque(maxlen=self.trail_len)
                trail.append(p)
                self.tracks[tid] = {
                    "x": p.x,
                    "y": p.y,
                    "z": p.z,
                    "frame": msg.header.frame_id or "base_link",
                    "last": t,
                    "trail": trail,
                    "last_pub": 0.0,
                    "speed": msg.speed,
                    "distance": msg.distance,
                }
            else:
                rec["x"], rec["y"], rec["z"] = p.x, p.y, p.z
                rec["frame"] = msg.header.frame_id or rec["frame"]
                rec["last"] = t
                rec["speed"] = msg.speed
                rec["distance"] = msg.distance
                rec["trail"].append(p)

    def on_timer(self):
        now = self._now()
        ma = MarkerArray()
        to_delete = []

        for tid, rec in list(self.tracks.items()):
            # TTL cleanup
            if (now - rec["last"]) > self.ttl:
                to_delete.append(tid)
                # publish deletes for this tid
                for ns in ("track_sphere", "track_text", "track_trail"):
                    m = Marker()
                    m.header.frame_id = rec["frame"]
                    m.header.stamp = self.get_clock().now().to_msg()
                    m.ns = ns
                    m.id = self._get_id(tid)
                    m.action = Marker.DELETE
                    ma.markers.append(m)
                continue

            # rate-limit unchanged re-pubs
            if (now - rec["last_pub"]) < self.keepalive_dt:
                continue

            # build markers
            base_id = self._get_id(tid)
            r, g, b = self._color_for(tid)

            # dot
            m = Marker()
            m.header.frame_id = rec["frame"]
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "track_sphere"
            m.id = base_id
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = (
                rec["x"],
                rec["y"],
                rec["z"],
            )
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = self.dot_size
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
            ma.markers.append(m)

            # label
            label = Marker()
            label.header.frame_id = rec["frame"]
            label.header.stamp = m.header.stamp
            label.ns = "track_text"
            label.id = base_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x, label.pose.position.y = rec["x"], rec["y"]
            label.pose.position.z = rec["z"] + self.dot_size * 1.2
            label.pose.orientation.w = 1.0
            label.scale.z = self.dot_size * 0.9  # text height
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 1.0
            spd = rec.get("speed", 0.0) or 0.0
            dist = rec.get("distance", 0.0) or 0.0
            label.text = f"{tid}\n{dist:.1f} m, {spd:.1f} m/s"
            ma.markers.append(label)

            # trail
            trail = Marker()
            trail.header.frame_id = rec["frame"]
            trail.header.stamp = m.header.stamp
            trail.ns = "track_trail"
            trail.id = base_id
            trail.type = Marker.LINE_STRIP
            trail.action = Marker.ADD
            trail.scale.x = self.dot_size * 0.15
            trail.color.r, trail.color.g, trail.color.b, trail.color.a = r, g, b, 0.7
            trail.points = list(rec["trail"])
            ma.markers.append(trail)

            rec["last_pub"] = now

        # ---- Fused detections (pre-tracks) as a point cloud ----
        if self.show_fused:
            # Drop old fused points
            self.fused_points = [
                p for p in self.fused_points if (now - p["t"]) <= self.fused_ttl
            ]

            if self.fused_points:
                # Use first point's frame; in practice all should be in fusion frame
                frame = self.fused_points[0]["frame"]
                fused_marker = Marker()
                fused_marker.header.frame_id = frame
                fused_marker.header.stamp = self.get_clock().now().to_msg()
                fused_marker.ns = "fused_points"
                fused_marker.id = 0
                fused_marker.type = Marker.POINTS
                fused_marker.action = Marker.ADD
                fused_marker.scale.x = self.dot_size * 0.3
                fused_marker.scale.y = self.dot_size * 0.3
                fused_marker.color.r = 1.0
                fused_marker.color.g = 1.0
                fused_marker.color.b = 0.0  # yellow
                fused_marker.color.a = 0.9
                fused_marker.points = [
                    Point(x=p["x"], y=p["y"], z=p["z"]) for p in self.fused_points
                ]
                ma.markers.append(fused_marker)

        # prune deleted from dict + id map
        for tid in to_delete:
            self.tracks.pop(tid, None)
            # keep id_map to preserve colors if the id reappears soon; drop if you prefer

        if ma.markers:
            self.pub.publish(ma)


def main():
    rclpy.init()
    node = TrackViz()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
