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

import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from tractor_safety_system_interfaces.msg import RadarDetection


class LaserScanToRadar(Node):

    def __init__(self):
        super().__init__('laser_to_radar')
        self.declare_parameter('scan_topic', '/sim/lidar')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('sensor_frame', 'tractor_simple/lidar_link/front_lidar')
        self.declare_parameter('y_max', 0.6)  # lateral half-width in sensor frame
        self.scan_topic = self.get_parameter('scan_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.sensor_frame = self.get_parameter('sensor_frame').value
        self.y_max = float(self.get_parameter('y_max').value)

        self.pub = self.create_publisher(RadarDetection, '/radar_detections', 10)
        self.sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)

        self.prev_dist = None
        self.prev_time = None
        self.get_logger().info(f'LaserScan→Radar on {self.scan_topic}')

    def on_scan(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        # Keep finite ranges
        finite = np.isfinite(ranges)
        if not np.any(finite):
            return
        angles = msg.angle_min + np.arange(ranges.size, dtype=np.float32)*msg.angle_increment
        # Sensor-frame coordinates
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        # In front & within lateral gate
        m = (x > 0.0) & (np.abs(y) <= self.y_max) & finite
        if not np.any(m):
            return

        # Choose nearest forward beam
        idx = np.argmin(x[m])
        sel_x = float(x[m][idx])
        sel_y = float(y[m][idx])
        sel_z = 0.0  # planar

        # Transform to base_link if available
        px, py, pz = sel_x, sel_y, sel_z
        try:
            T = self.tf.lookup_transform(self.base_frame, self.sensor_frame, msg.header.stamp,
                                         timeout=Duration(seconds=0.05))
            # Build rotation matrix
            q = T.transform.rotation
            n = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w) or 1.0
            xq, yq, zq, wq = q.x/n, q.y/n, q.z/n, q.w/n
            R = np.array([
                [1-2*(yq*yq+zq*zq), 2*(xq*yq - wq*zq), 2*(xq*zq + wq*yq)],
                [2*(xq*yq + wq*zq), 1-2*(xq*xq+zq*zq), 2*(yq*zq - wq*xq)],
                [2*(xq*zq - wq*yq), 2*(yq*zq + wq*xq), 1-2*(xq*xq+yq*yq)]
            ])
            t = np.array([T.transform.translation.x,
                          T.transform.translation.y,
                          T.transform.translation.z])
            p = R @ np.array([sel_x, sel_y, sel_z]) + t
            px, py, pz = float(p[0]), float(p[1]), float(p[2])
        except Exception as e:
            # OK to publish in sensor frame if TF missing
            self.get_logger().debug(f'TF {self.sensor_frame}->{self.base_frame} missing: {e}')
            self.base_frame = self.sensor_frame

        dist = math.sqrt(px*px + py*py + pz*pz)
        t_now = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        speed = 0.0
        if self.prev_dist is not None and self.prev_time is not None:
            dt = max(1e-3, t_now - self.prev_time)
            speed = (self.prev_dist - dist) / dt
        self.prev_dist, self.prev_time = dist, t_now

        out = RadarDetection()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.base_frame
        out.position.x, out.position.y, out.position.z = px, py, pz
        out.distance = dist
        out.speed = speed
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(LaserScanToRadar())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
