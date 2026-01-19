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
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Header
from vision_msgs.msg import BoundingBox2D, ObjectHypothesis

from tractor_safety_system_interfaces.msg import CameraDetection, RadarDetection


def now_msg(node: Node) -> Time:
    return node.get_clock().now().to_msg()


class SyntheticTargetsPublisher(Node):

    def __init__(self):
        super().__init__("synthetic_targets")

        # -------- Parameters (runtime-tunable) --------
        # Target A position (meters, radar frame convention)
        self.declare_parameter("targetA_x", 10.0)
        self.declare_parameter("targetA_y", 0.0)

        # Target B position (enable + meters)
        self.declare_parameter("enable_B", False)
        self.declare_parameter("targetB_x", 12.0)
        self.declare_parameter("targetB_y", 0.0)

        # Noise (std dev in meters) applied independently per sensor
        self.declare_parameter("cam_xy_noise_std", 0.00)
        self.declare_parameter("rad_xy_noise_std", 0.00)

        # Deviation between sensor detections for target A
        self.declare_parameter("deviation_Ay", 0.0)  # (y-axis)
        self.declare_parameter("deviation_Ax", 0.0)  # (x-axis)

        # Camera bbox (for your fusion’s bearing heuristics); size in pixels-ish
        self.declare_parameter("bbox_w", 60.0)
        self.declare_parameter("bbox_h", 40.0)

        # Camera “confidence” to fill ObjectHypothesis.score (0..1)
        self.declare_parameter("camera_confidence", 0.9)

        # Publication rate & optional camera–radar time skew
        self.declare_parameter("pub_rate_hz", 0.5)
        self.declare_parameter("time_skew_s", 0.0)  # camera timestamp = now - skew

        # Optional constant radial speeds (for RadarDetection.speed)
        self.declare_parameter("radial_speed_A", 0.0)  # m/s
        self.declare_parameter("radial_speed_B", 0.0)  # m/s

        # Frames
        self.declare_parameter("camera_frame", "camera_frame")
        self.declare_parameter("radar_frame", "radar_frame")

        # Read params
        self._load_params()

        # Dynamic param updates
        self.add_on_set_parameters_callback(self._on_set_params)

        # Publishers
        self.pub_cam = self.create_publisher(CameraDetection, "/camera_detections", 10)
        self.pub_rad = self.create_publisher(RadarDetection, "/radar_detections", 10)

        # Timer
        self.timer = self.create_timer(1.0 / max(1e-3, self.pub_rate_hz), self._on_tick)

        self.get_logger().info("SyntheticTargetsPublisher ready.")

    # ------- Param helpers -------

    def _load_params(self):
        gp = self.get_parameter
        self.targetA = np.array(
            [gp("targetA_x").value, gp("targetA_y").value], dtype=float
        )
        self.enable_B = bool(gp("enable_B").value)
        self.targetB = np.array(
            [gp("targetB_x").value, gp("targetB_y").value], dtype=float
        )

        self.cam_noise = float(gp("cam_xy_noise_std").value)
        self.rad_noise = float(gp("rad_xy_noise_std").value)
        self.deviation_y = float(gp("deviation_Ay").value)
        self.deviation_x = float(gp("deviation_Ax").value)

        self.bbox_w = float(gp("bbox_w").value)
        self.bbox_h = float(gp("bbox_h").value)

        self.cam_conf = float(gp("camera_confidence").value)

        self.pub_rate_hz = float(gp("pub_rate_hz").value)
        self.time_skew_s = float(gp("time_skew_s").value)

        self.vr_A = float(gp("radial_speed_A").value)
        self.vr_B = float(gp("radial_speed_B").value)

        self.camera_frame = str(gp("camera_frame").value)
        self.radar_frame = str(gp("radar_frame").value)

    def _on_set_params(self, params):
        # Accept everything; re-read into locals
        self._load_params()
        # Update timer period if rate changed
        self.timer.timer_period_ns = int(1e9 / max(1e-3, self.pub_rate_hz))
        return SetParametersResult(successful=True)

    # ------- Publish loop -------

    def _on_tick(self):
        self._load_params()  # ensure we have latest params
        # Draw fresh noise each tick
        cam_noise = np.random.normal(0.0, self.cam_noise, size=2)
        rad_noise = np.random.normal(0.0, self.rad_noise, size=2)

        # Target A: base positions (radar frame)
        Ax, Ay = self.targetA
        Bx, By = self.targetB

        # Publish RADAR A
        self._publish_radar_point(
            Ax + rad_noise[0], Ay + rad_noise[1], self.vr_A, tag="A"
        )

        # Publish CAMERA A (with independent noise)
        self._publish_camera_point(
            Ax + cam_noise[0] + self.deviation_x,
            Ay + cam_noise[1] + self.deviation_y,
            conf=self.cam_conf,
            tag="A",
        )

        if self.enable_B:
            # Draw fresh noise for B (independent)
            cam_noise_B = np.random.normal(0.0, self.cam_noise, size=2)
            rad_noise_B = np.random.normal(0.0, self.rad_noise, size=2)

            # RADAR B
            self._publish_radar_point(
                Bx + rad_noise_B[0], By + rad_noise_B[1], self.vr_B, tag="B"
            )

            # CAMERA B
            self._publish_camera_point(
                Bx + cam_noise_B[0], By + cam_noise_B[1], conf=self.cam_conf, tag="B"
            )

    # ------- Message builders -------

    def _publish_radar_point(self, x: float, y: float, radial_speed: float, tag: str):
        msg = RadarDetection()
        msg.header = Header()
        msg.header.frame_id = self.radar_frame
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.position = Point(x=float(x), y=float(y), z=0.0)
        msg.distance = float(math.hypot(x, y))
        msg.speed = float(radial_speed)
        self.pub_rad.publish(msg)

        self.get_logger().info(
            f"RAD {tag}: pos=({x:.2f},{y:.2f}) r={msg.distance:.2f} vr={radial_speed:.2f}"
        )

    def _publish_camera_point(self, x: float, y: float, conf: float, tag: str):
        msg = CameraDetection()
        msg.header = Header()
        msg.header.frame_id = self.camera_frame
        msg.header.stamp = self.get_clock().now().to_msg()

        # Minimal bbox with center (pixels not used downstream, but included)
        bbox = BoundingBox2D()
        bbox.center.position.x = 0.0
        bbox.center.position.y = 0.0
        bbox.size_x = float(self.bbox_w)
        bbox.size_y = float(self.bbox_h)
        msg.bbox = bbox

        # One hypothesis with configurable confidence
        hyp = ObjectHypothesis()
        hyp.class_id = "target" + tag
        hyp.score = float(conf)
        msg.results = [hyp]

        msg.position = Point(x=-float(y), y=0.0, z=float(x))  # camera frame convention
        msg.is_tracking = False
        msg.tracking_id = ""

        self.pub_cam.publish(msg)

        self.get_logger().info(
            f"CAM {tag}:"
            f"pos=({x:.2f},{y:.2f}) "
            f"conf={conf:.2f} "
            f"bbox=({self.bbox_w:.1f},{self.bbox_h:.1f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticTargetsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
