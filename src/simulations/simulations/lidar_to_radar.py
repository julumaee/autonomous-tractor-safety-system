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
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener
from tractor_safety_system_interfaces.msg import RadarDetection


class SimpleLidarToRadar(Node):
    def __init__(self):
        super().__init__('simple_lidar_to_radar')
        self.declare_parameter('cloud_topic', '/sim/lidar/points')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('y_max', 1.5)       # lateral half-width (m) for “in front”
        self.declare_parameter('z_min', 0.05)      # ground reject (m)
        self.declare_parameter('z_max', 2.5)       # max height (m)
        self.declare_parameter('distance_mode', 'longitudinal')  # 'euclidean' or 'longitudinal'

        self.cloud_topic = self.get_parameter('cloud_topic').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.y_max = float(self.get_parameter('y_max').value)
        self.z_min = float(self.get_parameter('z_min').value)
        self.z_max = float(self.get_parameter('z_max').value)
        self.distance_mode = self.get_parameter('distance_mode').get_parameter_value().string_value

        self.tfbuf = Buffer()
        self.tfl = TransformListener(self.tfbuf, self)

        self.sub = self.create_subscription(
            PointCloud2, self.cloud_topic, self.on_cloud, rclpy.qos.qos_profile_sensor_data)
        self.pub = self.create_publisher(RadarDetection, '/radar_detections', 10)

        self.prev_dist = None
        self.prev_time = None
        self.get_logger().info('Simple LiDAR→Radar running.')

    def on_cloud(self, msg: PointCloud2):
        self.get_logger().debug(
            f'Cloud stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec} '
            f'frame={msg.header.frame_id} w={msg.width} h={msg.height} '
            f'point_step={msg.point_step} row_step={msg.row_step} data={len(msg.data)} bytes'
        )
        # TF to base_link
        try:
            tf = self.tfbuf.lookup_transform(
                self.base_frame, msg.header.frame_id, msg.header.stamp,
                timeout=Duration(seconds=0.05))
        except Exception as e:
            self.get_logger().warn(f'TF {msg.header.frame_id}->{self.base_frame} missing: {e}')
            return

        # Convert to Nx3 array and transform
        pts = np.array([[p[0], p[1], p[2]] for p in pc2.read_points(msg,
                                                                    field_names=('x', 'y', 'z'),
                                                                    skip_nans=True)],
                       dtype=np.float64)

        # 1) Quick sanity on fields
        field_names = [f.name for f in msg.fields]
        if not {'x', 'y', 'z'}.issubset(field_names):
            self.get_logger().warn(
                f'Cloud missing x/y/z fields: fields={field_names}; '
                f'w={msg.width}'
                f'h={msg.height}'
                f'point_step={msg.point_step}'
                f'len(data)={len(msg.data)}'
            )
            return

        # 2) Use numpy reader; don't drop NaNs here—filter after
        try:
            arr = pc2.read_points_numpy(
                msg,
                field_names=('x', 'y', 'z'),
                skip_nans=False,
            )
        except Exception as e:
            self.get_logger().warn(f'read_points_numpy failed: {e}')
            return

        if arr.size == 0:
            self.get_logger().warn(
                f'PointCloud2 parse produced 0 values '
                f'(w={msg.width}'
                f'h={msg.height}'
                f'point_step={msg.point_step}'
                f'row_step={msg.row_step}'
                f'len(data)={len(msg.data)}'
                f'is_bigendian={msg.is_bigendian})'
            )
            return

        # Normalize to (N,3) float64
        pts = np.asarray(arr)
        if pts.ndim == 1:
            pts = pts.reshape((-1, 3))

        # If it's a structured array (dtype.names present), rebuild (N,3)
        if getattr(pts, 'dtype', None) is not None and pts.dtype.names is not None:
            try:
                pts = np.vstack([pts['x'], pts['y'], pts['z']]).T
            except Exception as e:
                self.get_logger().warn(f'Failed to unpack structured array: {e}')
                return

        pts = pts.astype(np.float64, copy=False)

        # 3) Filter non-finite / zeros if your GPU lidar uses inf/NaN for no-returns
        mask = np.isfinite(pts).all(axis=1)
        pts = pts[mask]
        if pts.shape[0] == 0:
            self.get_logger().warn(
                'All points are non-finite after filtering '
                f'(original {arr.shape[0]}). Check lidar config / bridge.'
            )
            return

        # Apply transform (rotation+translation)
        # Build 3x3 R and t from geometry_msgs/Transform
        q = tf.transform.rotation
        n = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w) or 1.0
        x, y, z, w = q.x/n, q.y/n, q.z/n, q.w/n
        R = np.array([
            [1-2*(y*y+z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1-2*(x*x+z*z), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1-2*(x*x+y*y)]
        ], dtype=np.float64)
        t = np.array([tf.transform.translation.x,
                      tf.transform.translation.y,
                      tf.transform.translation.z], dtype=np.float64)
        pts_bl = (R @ pts.T).T + t

        # Keep points in front of the tractor (x>0), within lateral & height gates
        m = (pts_bl[:, 0] > 0.0) & (np.abs(pts_bl[:, 1]) <= self.y_max) \
            & (pts_bl[:, 2] >= self.z_min) & (pts_bl[:, 2] <= self.z_max)
        cand = pts_bl[m]
        if cand.shape[0] == 0:
            return

        # Choose the nearest forward point
        if self.distance_mode == 'longitudinal':
            idx = np.argmin(cand[:, 0])  # smallest x
            distance_m = float(max(0.0, cand[idx, 0]))
        else:
            dists = np.linalg.norm(cand, axis=1)  # euclidean
            idx = int(np.argmin(dists))
            distance_m = float(dists[idx])

        pos = cand[idx]  # x,y,z in base_link

        # Speed estimate = radial closing speed (positive when approaching)
        t_now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        speed_mps = 0.0
        if self.prev_dist is not None and self.prev_time is not None:
            dt = max(1e-3, t_now - self.prev_time)
            speed_mps = (self.prev_dist - distance_m) / dt  # + if getting closer
        self.prev_dist, self.prev_time = distance_m, t_now

        out = RadarDetection()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.base_frame
        out.position.x, out.position.y, out.position.z = map(float, pos)
        out.distance = int(round(distance_m))  # meters (integer)
        out.speed = int(round(speed_mps))  # m/s (integer)
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(SimpleLidarToRadar())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
