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
from image_geometry import PinholeCameraModel
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import BoundingBox2D, Detection2DArray, ObjectHypothesis

from tractor_safety_system_interfaces.msg import CameraDetection


class DepthDetectionsToCamera(Node):

    def __init__(self):
        super().__init__("depth_dets_to_camera")
        # Params
        self.declare_parameter("dets_topic", "/sim/cam/detections2d")
        self.declare_parameter("depth_topic", "/sim/cam/depth")
        self.declare_parameter("info_topic", "/sim/cam/camera_info")
        self.declare_parameter("camera_frame", "camera_optical_frame")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter(
            "depth_encoding", "32FC1"
        )  # '32FC1' meters or '16UC1' millimeters
        self.declare_parameter(
            "sample_fraction", 0.2
        )  # center region of bbox for depth median

        self.dets_topic = self.get_parameter("dets_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.info_topic = self.get_parameter("info_topic").value
        self.cam_frame = self.get_parameter("camera_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.depth_encoding = self.get_parameter("depth_encoding").value
        self.sample_fraction = float(self.get_parameter("sample_fraction").value)

        # IO
        self.sub_info = self.create_subscription(
            CameraInfo, self.info_topic, self.on_info, 10
        )
        self.sub_depth = self.create_subscription(
            Image, self.depth_topic, self.on_depth, rclpy.qos.qos_profile_sensor_data
        )
        self.sub_dets = self.create_subscription(
            Detection2DArray, self.dets_topic, self.on_dets, 10
        )
        self.pub = self.create_publisher(CameraDetection, "/camera_detections", 10)

        # State
        self.cam = PinholeCameraModel()
        self.have_cam = False
        self.depth = None
        self.depth_stamp = None

        self.tfbuf = Buffer()
        self.tfl = TransformListener(self.tfbuf, self)

        self.get_logger().info("Detections+Depth → CameraDetection running.")

    def on_info(self, msg: CameraInfo):
        self.cam.fromCameraInfo(msg)
        self.have_cam = True

    def on_depth(self, msg: Image):
        self.depth_stamp = msg.header.stamp
        # Parse depth to float32 meters
        if self.depth_encoding == "32FC1":
            depth = np.frombuffer(msg.data, dtype=np.float32).reshape(
                (msg.height, msg.width)
            )
            self.depth = depth
        elif self.depth_encoding == "16UC1":
            depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                (msg.height, msg.width)
            )
            self.depth = depth.astype(np.float32) * 0.001  # mm → m
        else:
            self.get_logger().throttle(
                2000, f"Unsupported depth encoding {self.depth_encoding}"
            )
            self.depth = None

    def on_dets(self, msg: Detection2DArray):
        if not self.have_cam or self.depth is None:
            return

        # Try TF base_link <- camera frame at this timestamp
        try:
            T_cb = self.tfbuf.lookup_transform(
                self.base_frame,
                self.cam.tfFrame(),
                msg.header.stamp,
                timeout=Duration(seconds=0.05),
            )
        except Exception as e:
            self.get_logger().warn(
                f"TF {self.cam.tfFrame()}->{self.base_frame} missing: {e}"
            )
            return

        fx = self.cam.fx()
        fy = self.cam.fy()
        cx = self.cam.cx()
        cy = self.cam.cy()

        H, W = self.depth.shape

        for det in msg.detections:
            bb: BoundingBox2D = det.bbox
            u = bb.center.x
            v = bb.center.y
            w = max(2.0, bb.size_x)
            h = max(2.0, bb.size_y)

            # Sample a small central window for robust depth
            frac = self.sample_fraction
            wu = max(1, int(w * frac / 2))
            hv = max(1, int(h * frac / 2))
            u0 = int(np.clip(u - wu, 0, W - 1))
            u1 = int(np.clip(u + wu, 0, W - 1))
            v0 = int(np.clip(v - hv, 0, H - 1))
            v1 = int(np.clip(v + hv, 0, H - 1))

            window = self.depth[v0: v1 + 1, u0: u1 + 1]
            if window.size == 0:
                continue
            z_vals = window[np.isfinite(window) & (window > 0.05)]
            if z_vals.size == 0:
                continue
            Z = float(np.median(z_vals))  # meters

            # Back-project to camera frame
            X = (u - cx) * Z / fx
            Y = (v - cy) * Z / fy
            p_cam = np.array([X, Y, Z], dtype=np.float64)

            # Transform to base_link
            # Build transform matrix from geometry_msgs Transform
            t = T_cb.transform.translation
            q = T_cb.transform.rotation
            n = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) or 1.0
            x, y, z, wq = q.x / n, q.y / n, q.z / n, q.w / n
            R = np.array(
                [
                    [
                        1 - 2 * (y * y + z * z),
                        2 * (x * y - wq * z),
                        2 * (x * z + wq * y),
                    ],
                    [
                        2 * (x * y + wq * z),
                        1 - 2 * (x * x + z * z),
                        2 * (y * z - wq * x),
                    ],
                    [
                        2 * (x * z - wq * y),
                        2 * (y * z + wq * x),
                        1 - 2 * (x * x + y * y),
                    ],
                ],
                dtype=np.float64,
            )
            tvec = np.array([t.x, t.y, t.z], dtype=np.float64)
            p_bl = R @ p_cam + tvec

            # Compose your CameraDetection
            out = CameraDetection()
            out.header.stamp = msg.header.stamp
            out.header.frame_id = self.base_frame

            # Copy classes/scores if present
            if hasattr(det, "results"):
                for r in det.results:
                    hyp = ObjectHypothesis()
                    # Detection2D results may be ObjectHypothesis or ObjectHypothesisWithPose
                    if hasattr(r, "hypothesis"):
                        hyp.class_id = r.hypothesis.class_id
                        hyp.score = r.hypothesis.score
                    else:
                        hyp.class_id = r.class_id
                        hyp.score = r.score
                    out.results.append(hyp)

            # Copy bbox
            out.bbox.center.position.x = float(u)
            out.bbox.center.position.y = float(v)
            out.bbox.size_x = float(w)
            out.bbox.size_y = float(h)

            # 3D position in meters (base_link)
            out.position.x = float(p_bl[0])
            out.position.y = float(p_bl[1])
            out.position.z = float(p_bl[2])

            # Tracking (simple): leave off unless your input carries an ID
            out.is_tracking = False
            out.tracking_id = ""

            self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(DepthDetectionsToCamera())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
