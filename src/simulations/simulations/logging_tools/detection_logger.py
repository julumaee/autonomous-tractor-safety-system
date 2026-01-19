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

# Logs raw sensor detections, fused detections and tracks
# to CSV files for offline analysis.
import csv
import math
import os
from datetime import datetime

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from tractor_safety_system_interfaces.msg import (
    CameraDetection,
    FusedDetection,
    FusedDetectionArray,
    RadarDetection,
)


class DetectionLogger(Node):

    def __init__(self):
        super().__init__("detection_logger_node")
        # Optional scenario name (e.g. "S1", "S2")
        self.declare_parameter("scenario_name", "unknown")
        self.scenario_name = (
            self.get_parameter("scenario_name").get_parameter_value().string_value
        )

        # Directory + timestamp for filenames
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_dir = "test_logs"
        os.makedirs(log_dir, exist_ok=True)

        # -----------------------
        # Open CSV files + header
        # -----------------------

        # Raw camera / radar detections
        self.raw_file = open(
            os.path.join(log_dir, f"raw_detections_{self.scenario_name}.csv"),
            "w",
            newline="",
        )
        self.raw_writer = csv.writer(self.raw_file)
        self.raw_writer.writerow(
            [
                "sensor_type",  # "radar" or "camera"
                "logger_stamp",  # arrival to logger
                "header_stamp",  # detection timestamp
                "x",
                "y",
                "z",  # position
                f"Simulation was run at: {timestamp}",
            ]
        )

        # Fused detections
        self.fused_file = open(
            os.path.join(log_dir, f"fused_detections_{self.scenario_name}.csv"),
            "w",
            newline="",
        )
        self.fused_writer = csv.writer(self.fused_file)
        self.fused_writer.writerow(
            [
                "logger_stamp",  # arrival to logger
                "header_stamp",  # detection timestamp
                "detection_type",  # "fused" / "camera" / "radar"
                "x",
                "y",
                "z",  # position
                "distance",  # distance to ego
                "speed",  # speed of the detected object
                f"Simulation was run at: {timestamp}",
            ]
        )

        # Tracks
        self.tracks_file = open(
            os.path.join(log_dir, f"tracks_{self.scenario_name}.csv"), "w", newline=""
        )
        self.tracks_writer = csv.writer(self.tracks_file)
        self.tracks_writer.writerow(
            [
                "logger_stamp",  # arrival to logger
                "header_stamp",  # track message timestamp
                "tracking_id",  # unique track ID
                "x",
                "y",
                "z",  # position
                "distance",  # distance to ego
                "speed",  # speed of the tracked object (ego-compensated)
                "age",  # number of frames since track was created
                "consecutive_misses",  # number of consecutive misses for the track
                f"Simulation was run at: {timestamp}",
            ]
        )

        # Ego ground truth (odometry)
        self.ego_file = open(
            os.path.join(log_dir, f"ego_odom_{self.scenario_name}.csv"), "w", newline=""
        )
        self.ego_writer = csv.writer(self.ego_file)
        self.ego_writer.writerow(
            [
                "logger_stamp",  # arrival to logger
                "header_stamp",  # odom message timestamp
                "x",
                "y",  # position
                "yaw",  # orientation
                "v",  # forward speed
                "yaw_rate",  # yaw rate
                f"Simulation was run at: {timestamp}",
            ]
        )

        # -----------------------
        # Subscriptions
        # -----------------------

        # Raw detections
        self.create_subscription(
            RadarDetection, "/radar_detections", self.handle_radar, 10
        )
        self.create_subscription(
            CameraDetection, "/camera_detections", self.handle_camera, 10
        )

        # Fused detections (from fusion node)
        self.create_subscription(
            FusedDetection, "/fused_detections", self.handle_fused, 10
        )

        # Tracks (from tracker node)
        self.create_subscription(
            FusedDetectionArray, "/tracked_detections", self.handle_track, 50
        )

        # Ego odom (from Gazebo bridge)
        self.create_subscription(Odometry, "/odom", self.handle_odom, 20)

        self.get_logger().info(
            f'DetectionLogger started, logging to "{log_dir}" '
            f'for scenario "{self.scenario_name}"'
        )

    # -----------------------
    # Time helpers
    # -----------------------

    def _stamp_to_seconds(self, time_msg) -> float:
        """Convert ROS2 Time message to datetime."""
        return time_msg.sec + time_msg.nanosec * 1e-9

    def _get_current_time(self) -> float:
        """Get current time."""
        now = self.get_clock().now().to_msg()
        return self._stamp_to_seconds(now)

    # -----------------------
    # Callbacks
    # -----------------------

    def handle_radar(self, msg: RadarDetection):
        header_time = self._stamp_to_seconds(msg.header.stamp)
        logger_time = self._get_current_time()
        self.raw_writer.writerow(
            [
                "radar",
                logger_time,
                header_time,
                msg.position.x,
                msg.position.y,
                msg.position.z,
            ]
        )

    def handle_camera(self, msg: CameraDetection):
        header_time = self._stamp_to_seconds(msg.header.stamp)
        logger_time = self._get_current_time()
        self.raw_writer.writerow(
            [
                "camera",
                logger_time,
                header_time,
                msg.position.x,
                msg.position.y,
                msg.position.z,
            ]
        )

    def handle_fused(self, msg: FusedDetection):
        header_time = self._stamp_to_seconds(msg.header.stamp)
        logger_time = self._get_current_time()
        self.fused_writer.writerow(
            [
                logger_time,
                header_time,
                msg.detection_type,
                msg.position.x,
                msg.position.y,
                msg.position.z,
                msg.distance,
                msg.speed,
            ]
        )

    def handle_track(self, msg: FusedDetectionArray):
        """Tracks published by KF tracker as FusedDetectionArray messages."""
        for msg in msg.detections:
            header_time = self._stamp_to_seconds(msg.header.stamp)
            logger_time = self._get_current_time()
            self.tracks_writer.writerow(
                [
                    logger_time,
                    header_time,
                    msg.tracking_id,
                    msg.position.x,
                    msg.position.y,
                    msg.position.z,
                    msg.distance,
                    msg.speed,
                    msg.age,
                    msg.consecutive_misses,
                ]
            )

    def handle_odom(self, msg: Odometry):
        header_time = self._stamp_to_seconds(msg.header.stamp)
        logger_time = self._get_current_time()

        # Position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Yaw from quaternion
        q = msg.pose.pose.orientation
        # yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        v = msg.twist.twist.linear.x
        yaw_rate = msg.twist.twist.angular.z

        self.ego_writer.writerow(
            [
                logger_time,
                header_time,
                x,
                y,
                yaw,
                v,
                yaw_rate,
            ]
        )

    # -----------------------
    # Cleanup
    # -----------------------

    def destroy_node(self):
        # Close all files cleanly
        for f in [
            getattr(self, "raw_file", None),
            getattr(self, "fused_file", None),
            getattr(self, "tracks_file", None),
            getattr(self, "ego_file", None),
        ]:
            if f is not None:
                try:
                    f.close()
                except Exception:
                    pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DetectionLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
