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

import numpy as np
import rclpy
from depthai_ros_msgs.msg import SpatialDetection, SpatialDetectionArray
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from ultralytics_ros.msg import YoloResult
from vision_msgs.msg import BoundingBox2D, Detection2DArray, ObjectHypothesis


class SpatialFromYolo(Node):

    def __init__(self):
        super().__init__("spatial_from_yolo")
        self.declare_parameter(
            "dets2d_topic", "/yolo/result"
        )  # YoloResult or Detection2DArray
        self.declare_parameter("depth_topic", "/sim/cam/depth")  # 32FC1
        self.declare_parameter("info_topic", "/sim/cam/camera_info")
        self.declare_parameter("out_topic", "/color/yolov4_Spatial_detections")
        self.declare_parameter("sample_fraction", 0.2)  # central window % of bbox

        self.depth = None
        self.K = None
        self.cam_frame = "camera_frame"
        self.frac = float(self.get_parameter("sample_fraction").value)

        qos = rclpy.qos.qos_profile_sensor_data
        self.create_subscription(
            CameraInfo, self.get_parameter("info_topic").value, self.on_info, 10
        )
        self.create_subscription(
            Image, self.get_parameter("depth_topic").value, self.on_depth, 10
        )

        dets_topic = self.get_parameter("dets2d_topic").value
        self.create_subscription(YoloResult, dets_topic, self.on_yolo_result, qos)

        self.pub = self.create_publisher(
            SpatialDetectionArray, self.get_parameter("out_topic").value, 10
        )
        self.get_logger().info(
            "Publishing SpatialDetectionArray on" "/color/yolov4_Spatial_detections"
        )

    def on_info(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float32).reshape(3, 3)
        self.cam_frame = msg.header.frame_id or self.cam_frame

    def on_depth(self, msg: Image):
        if msg.encoding != "32FC1":
            self.get_logger().warn(f"Depth encoding is {msg.encoding}, expected 32FC1")
            return
        self.depth = np.frombuffer(msg.data, dtype=np.float32).reshape(
            (msg.height, msg.width)
        )
        if not np.isfinite(self.depth).any():
            self.get_logger().warn("Depth frame has no finite values (all inf/NaN)")

    def on_yolo_result(self, msg):
        for key in ("detections", "result", "detection_array"):
            d2d = getattr(msg, key, None)
            if d2d is not None:
                break

        if d2d is None:
            self.get_logger().warn(
                "YoloResult has no detections/result/detection_array field"
            )
            return

        if not hasattr(d2d, "header") or not hasattr(d2d, "detections"):
            self.get_logger().warn(f"Detections field is unexpected type: {type(d2d)}")
            return
        if not isinstance(d2d.detections, (list, tuple)):
            self.get_logger().warn("detections member is not a list/tuple")
            return

        # Wrap into a real Detection2DArray message (ensures fields exist)
        msg2 = Detection2DArray()
        msg2.header = d2d.header
        msg2.detections = d2d.detections
        self._emit_spatial(msg2)

    def _center_xy(self, bb_center):
        # Works with both Pose2D{x,y,theta} and Pose2D{position{x,y}, theta}
        if hasattr(bb_center, "x") and hasattr(bb_center, "y"):
            return float(bb_center.x), float(bb_center.y)
        if hasattr(bb_center, "position"):
            return float(bb_center.position.x), float(bb_center.position.y)
        # Last resort: try dict-like access or default zeros
        return float(getattr(bb_center, "x", 0.0)), float(getattr(bb_center, "y", 0.0))

    def _emit_spatial(self, msg: Detection2DArray):
        if self.depth is None or self.K is None:
            return
        H, W = self.depth.shape
        fx, fy = float(self.K[0, 0]), float(self.K[1, 1])
        cx, cy = float(self.K[0, 2]), float(self.K[1, 2])

        out = SpatialDetectionArray()
        out.header = msg.header
        out.header.frame_id = self.cam_frame

        for det in msg.detections:
            bb: BoundingBox2D = det.bbox
            u, v = self._center_xy(bb.center)
            w = max(2.0, float(bb.size_x))
            h = max(2.0, float(bb.size_y))

            du = max(1, int(w * self.frac * 0.5))
            dv = max(1, int(h * self.frac * 0.5))
            u0 = int(np.clip(u - du, 0, W - 1))
            u1 = int(np.clip(u + du, 0, W - 1))
            v0 = int(np.clip(v - dv, 0, H - 1))
            v1 = int(np.clip(v + dv, 0, H - 1))
            win = self.depth[v0: v1 + 1, u0: u1 + 1]
            if win.size == 0:
                self.get_logger().warn("Depth window empty after clipping")
                continue
            mask = np.isfinite(win) & (win > 0.05)
            if not mask.any():
                self.get_logger().warn("No finite depths inside bbox window")
                continue
            z = win[np.isfinite(win) & (win > 0.05)]
            if z.size == 0:
                continue
            Z = float(np.median(z))
            X = (u - cx) * Z / fx
            Y = (v - cy) * Z / fy

            sd = SpatialDetection()
            sd.bbox.center.position.x = u
            sd.bbox.center.position.y = v
            sd.bbox.size_x = w
            sd.bbox.size_y = h
            # copy first hypothesis if present
            if getattr(det, "results", None):
                r0 = det.results[0]
                hyp = ObjectHypothesis()
                if hasattr(r0, "hypothesis"):
                    hyp.class_id = r0.hypothesis.class_id
                    hyp.score = r0.hypothesis.score
                else:
                    hyp.class_id = r0.class_id
                    hyp.score = r0.score
                sd.results.append(hyp)
            sd.position.x = X
            sd.position.y = Y
            sd.position.z = Z
            out.detections.append(sd)

        if out.detections:
            self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(SpatialFromYolo())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
