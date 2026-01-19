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
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

from tractor_safety_system_interfaces.msg import RadarDetection


class SimpleLidarToRadar(Node):

    def __init__(self):
        super().__init__("simple_lidar_to_radar")
        self.declare_parameter("cloud_topic", "/sim/lidar/points")
        self.declare_parameter("radar_frame", "radar_link")
        self.declare_parameter("y_max", 10.0)  # lateral half-width (m) for “in front”
        self.declare_parameter("z_min", -0.55)  # ground reject (m)
        self.declare_parameter("z_max", 2.5)  # max height (m)
        self.declare_parameter(
            "distance_mode", "longitudinal"
        )  # 'euclidean' or 'longitudinal'
        self.declare_parameter("az_bins", 36)  # how many angular sectors in front
        self.declare_parameter("fov_left_deg", 65.0)  # left limit (positive Y side)
        self.declare_parameter("fov_right_deg", 65.0)  # right limit (negative Y side)
        self.declare_parameter("min_points_bin", 1)  # require N points in a bin
        self.declare_parameter("max_targets", 16)  # cap how many we publish per cloud

        self.cloud_topic = (
            self.get_parameter("cloud_topic").get_parameter_value().string_value
        )
        self.radar_frame = (
            self.get_parameter("radar_frame").get_parameter_value().string_value
        )
        self.y_max = float(self.get_parameter("y_max").value)
        self.z_min = float(self.get_parameter("z_min").value)
        self.z_max = float(self.get_parameter("z_max").value)
        self.distance_mode = (
            self.get_parameter("distance_mode").get_parameter_value().string_value
        )
        self.az_bins = int(self.get_parameter("az_bins").value)
        self.fov_left_rad = math.radians(
            float(self.get_parameter("fov_left_deg").value)
        )
        self.fov_right_rad = math.radians(
            float(self.get_parameter("fov_right_deg").value)
        )
        self.min_points_bin = int(self.get_parameter("min_points_bin").value)
        self.max_targets = int(self.get_parameter("max_targets").value)

        self.sub = self.create_subscription(
            PointCloud2,
            self.cloud_topic,
            self.on_cloud,
            rclpy.qos.qos_profile_sensor_data,
        )
        self.pub = self.create_publisher(RadarDetection, "/radar_detections", 10)

        self.prev_dist = None
        self.prev_time = None
        self.prev_by_bin = {}  # bin_idx -> (dist_m, t_sec)

        self.get_logger().info("Simple LiDAR→Radar running.")

    def on_cloud(self, msg: PointCloud2):
        self.get_logger().debug(
            f"Cloud stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec} "
            f"frame={msg.header.frame_id} w={msg.width} h={msg.height} "
            f"point_step={msg.point_step} row_step={msg.row_step} data={len(msg.data)} bytes"
        )

        try:
            arr = pc2.read_points_numpy(
                msg,
                field_names=("x", "y", "z"),
                skip_nans=False,
            )
        except Exception as e:
            self.get_logger().warn(f"read_points_numpy failed: {e}")
            return

        if arr.size == 0:
            self.get_logger().warn(
                f"PointCloud2 parse produced 0 values "
                f"(w={msg.width}"
                f"h={msg.height}"
                f"point_step={msg.point_step}"
                f"row_step={msg.row_step}"
                f"len(data)={len(msg.data)}"
                f"is_bigendian={msg.is_bigendian})"
            )
            return

        # Normalize to (N,3) float64
        pts = np.asarray(arr)
        # If it's a structured array (dtype.names present), rebuild (N,3)
        if getattr(pts, "dtype", None) is not None and pts.dtype.names is not None:
            try:
                pts = np.vstack([pts["x"], pts["y"], pts["z"]]).T
            except Exception as e:
                self.get_logger().warn(f"Failed to unpack structured array: {e}")
                return
        if pts.ndim == 1:
            pts = pts.reshape((-1, 3))
        pts = pts.astype(np.float64, copy=False)

        # 3) Filter non-finite / zeros if your GPU lidar uses inf/NaN for no-returns
        mask = np.isfinite(pts).all(axis=1)
        pts = pts[mask]
        if pts.shape[0] == 0:
            self.get_logger().warn(
                "All points are non-finite after filtering "
                f"(original {arr.shape[0]}). Check lidar config / bridge."
            )
            return

        # Forward frustum & height/lateral gates
        x = pts[:, 0]
        y = pts[:, 1]
        z = pts[:, 2]
        m = (
            (x > 0.0)
            & (np.abs(y) <= self.y_max)
            & (z >= self.z_min)
            & (z <= self.z_max)
        )
        if not np.any(m):
            return
        x = x[m]
        y = y[m]
        z = z[m]

        # Azimuth and FOV gate (front sector only)
        az = np.arctan2(y, x)  # [-pi, pi]
        m_fov = (az <= self.fov_left_rad) & (az >= -self.fov_right_rad)
        if not np.any(m_fov):
            return
        x = x[m_fov]
        y = y[m_fov]
        z = z[m_fov]
        az = az[m_fov]
        # Choose distance per-point
        if self.distance_mode == "longitudinal":
            d = x  # longitudinal distance
        else:
            d = np.sqrt(x * x + y * y + z * z)  # euclidean

        # Bin edges across the kept FOV
        left = self.fov_left_rad
        right = -self.fov_right_rad
        # ensure left >= right numerically
        span = float(left - right)
        # map az in [right, left] to [0, az_bins)
        bin_w = span / float(self.az_bins)
        bin_idx = np.clip(((az - right) / bin_w).astype(np.int32), 0, self.az_bins - 1)

        # For each bin, select nearest point
        detections = []
        for bi in range(self.az_bins):
            idxs = np.where(bin_idx == bi)[0]
            if idxs.size < self.min_points_bin:
                continue
            # pick nearest by chosen metric d
            j = idxs[np.argmin(d[idxs])]
            detections.append((bi, x[j], y[j], z[j], d[j]))

        if not detections:
            return

        # Keep nearest N overall (optional)
        detections.sort(key=lambda it: it[4])
        if len(detections) > self.max_targets:
            detections = detections[: self.max_targets]

        # Cluster adjacent bins into "objects" and compute centroids
        # Sort by bin index so that adjacent detections in azimuth are neighbors
        detections.sort(key=lambda it: it[0])  # sort by bi

        clusters = []
        current_cluster = []
        last_bi = None

        for bi, xb, yb, zb, dist in detections:
            if last_bi is None:
                current_cluster = [(bi, xb, yb, zb, dist)]
            else:
                # if this bin is adjacent to previous, same cluster
                if bi == last_bi or bi == last_bi + 1:
                    current_cluster.append((bi, xb, yb, zb, dist))
                else:
                    clusters.append(current_cluster)
                    current_cluster = [(bi, xb, yb, zb, dist)]
            last_bi = bi

        if current_cluster:
            clusters.append(current_cluster)

        # Compute one centroid per cluster
        centroids = []
        for cl in clusters:
            bis = np.array([c[0] for c in cl], dtype=np.int32)
            xs = np.array([c[1] for c in cl], dtype=np.float64)
            ys = np.array([c[2] for c in cl], dtype=np.float64)
            zs = np.array([c[3] for c in cl], dtype=np.float64)
            dists = np.array([c[4] for c in cl], dtype=np.float64)

            # representative bin = mean of bins (rounded)
            rep_bi = int(round(bis.mean()))
            # centroid in x,y,z (simple mean; you could also weight by 1/dist if you want)
            cx = xs.mean()
            cy = ys.mean()
            cz = zs.mean()
            cd = dists.mean()

            centroids.append((rep_bi, cx, cy, cz, cd))

        # Optional: still limit how many targets we publish
        centroids.sort(key=lambda it: it[4])  # by distance
        if len(centroids) > self.max_targets:
            centroids = centroids[: self.max_targets]

        # ------------------------------------------------------------
        # Publish one RadarDetection per centroid
        # ------------------------------------------------------------
        t_now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        for rep_bi, xb, yb, zb, dist in centroids:
            # simple speed estimate using representative bin index
            prev = self.prev_by_bin.get(rep_bi)
            speed_mps = 0.0
            if prev is not None:
                pd, pt = prev
                dt = max(1e-3, t_now - pt)
                speed_mps = (pd - float(dist)) / dt  # + if getting closer
            self.prev_by_bin[rep_bi] = (float(dist), t_now)

            out = RadarDetection()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = self.radar_frame
            out.position.x = float(xb)
            out.position.y = float(yb)
            out.position.z = float(zb)
            out.distance = float(dist)  # meters
            out.speed = float(speed_mps)  # m/s
            self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(SimpleLidarToRadar())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
