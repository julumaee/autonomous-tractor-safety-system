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

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from tractor_safety_system_interfaces.msg import FusedDetection
from visualization_msgs.msg import Marker, MarkerArray


class TrackViz(Node):

    def __init__(self):
        super().__init__('track_viz')
        self.sub = self.create_subscription(
            FusedDetection, '/tracked_detections', self.on_track, 50
        )
        self.pub = self.create_publisher(MarkerArray, '/tracked_markers', 10)

        # params
        self.declare_parameter('dot_size', 0.6)         # meters
        self.declare_parameter('trail_len', 30)         # points kept per track
        self.declare_parameter('ttl_sec', 1.5)          # delete if no update in this time
        self.declare_parameter('keepalive_hz', 2.0)     # republish unchanged at this rate

        self.dot_size = float(self.get_parameter('dot_size').value)
        self.trail_len = int(self.get_parameter('trail_len').value)
        self.ttl = float(self.get_parameter('ttl_sec').value)
        self.keepalive_dt = 1.0 / max(0.1, float(self.get_parameter('keepalive_hz').value))

        # state
        self.tracks = {}     # tid -> dict(x,y,z,frame,last,trail: deque[Point], last_pub)
        self.id_map = {}     # tid -> stable int id for markers
        self.next_id = 1

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
        h = abs(hash(tid)) % (256*256*256)
        r = ((h >> 16) & 255) / 255.0
        g = ((h >> 8) & 255) / 255.0
        b = (h & 255) / 255.0
        # lighten a bit
        r = 0.3 + 0.7*r
        g = 0.3 + 0.7*g
        b = 0.3 + 0.7*b
        return r, g, b

    def on_track(self, msg: FusedDetection):
        tid = msg.tracking_id or 'unknown'
        t = self._now()
        rec = self.tracks.get(tid)
        p = Point(x=msg.position.x, y=msg.position.y, z=msg.position.z)
        if rec is None:
            trail = deque(maxlen=self.trail_len)
            trail.append(p)
            self.tracks[tid] = {
                'x': p.x, 'y': p.y, 'z': p.z,
                'frame': msg.header.frame_id or 'base_link',
                'last': t, 'trail': trail, 'last_pub': 0.0,
                'speed': msg.speed, 'distance': msg.distance,
            }
        else:
            rec['x'], rec['y'], rec['z'] = p.x, p.y, p.z
            rec['frame'] = msg.header.frame_id or rec['frame']
            rec['last'] = t
            rec['speed'] = msg.speed
            rec['distance'] = msg.distance
            rec['trail'].append(p)

    def on_timer(self):
        now = self._now()
        ma = MarkerArray()
        to_delete = []

        for tid, rec in list(self.tracks.items()):
            # TTL cleanup
            if (now - rec['last']) > self.ttl:
                to_delete.append(tid)
                # publish deletes for this tid
                for ns in ('track_sphere', 'track_text', 'track_trail'):
                    m = Marker()
                    m.header.frame_id = rec['frame']
                    m.header.stamp = self.get_clock().now().to_msg()
                    m.ns = ns
                    m.id = self._get_id(tid)
                    m.action = Marker.DELETE
                    ma.markers.append(m)
                continue

            # rate-limit unchanged re-pubs
            if (now - rec['last_pub']) < self.keepalive_dt:
                continue

            # build markers
            base_id = self._get_id(tid)
            r, g, b = self._color_for(tid)

            # dot
            m = Marker()
            m.header.frame_id = rec['frame']
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'track_sphere'
            m.id = base_id
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = rec['x'], rec['y'], rec['z']
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = self.dot_size
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
            ma.markers.append(m)

            # label
            label = Marker()
            label.header.frame_id = rec['frame']
            label.header.stamp = m.header.stamp
            label.ns = 'track_text'
            label.id = base_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x, label.pose.position.y = rec['x'], rec['y']
            label.pose.position.z = rec['z'] + self.dot_size * 1.2
            label.pose.orientation.w = 1.0
            label.scale.z = self.dot_size * 0.9  # text height
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 1.0
            spd = rec.get('speed', 0.0) or 0.0
            dist = rec.get('distance', 0.0) or 0.0
            label.text = f'{tid}\n{dist:.1f} m, {spd:.1f} m/s'
            ma.markers.append(label)

            # trail
            trail = Marker()
            trail.header.frame_id = rec['frame']
            trail.header.stamp = m.header.stamp
            trail.ns = 'track_trail'
            trail.id = base_id
            trail.type = Marker.LINE_STRIP
            trail.action = Marker.ADD
            trail.scale.x = self.dot_size * 0.15
            trail.color.r, trail.color.g, trail.color.b, trail.color.a = r, g, b, 0.7
            trail.points = list(rec['trail'])
            ma.markers.append(trail)

            rec['last_pub'] = now

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


if __name__ == '__main__':
    main()
