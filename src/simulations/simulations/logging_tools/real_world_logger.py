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

# Logger for real-world tractor testing (GPS-based position tracking)
# Logs raw sensor detections, fused detections, tracks, and GPS-based ego motion.
import csv
import math
import os
from datetime import datetime

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped as TWCS
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

from tractor_safety_system_interfaces.msg import (
    CameraDetection,
    FusedDetection,
    FusedDetectionArray,
    RadarDetection,
)


class RealWorldLogger(Node):

    def __init__(self):
        super().__init__("real_world_logger_node")

        # Test scenario identifier
        self.declare_parameter("test_name", "test_1")
        self.declare_parameter("gps_fix_topic", "/gps/fix")

        self.test_name = (
            self.get_parameter("test_name").get_parameter_value().string_value
        )
        gps_fix_topic = (
            self.get_parameter("gps_fix_topic").get_parameter_value().string_value
        )

        # GPS origin for local coordinate conversion (set on first GPS message)
        self.origin_lat = None
        self.origin_lon = None
        self.origin_alt = None

        # Current GPS-based pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Current velocity from ego_motion
        self.vx = 0.0
        self.yaw_rate = 0.0
        self.last_ego_time = None

        # Directory + timestamp for filenames
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_dir = "real_world_logs"
        os.makedirs(log_dir, exist_ok=True)

        self.get_logger().info(f"Logging to directory: {log_dir}")
        self.get_logger().info(f"Test name: {self.test_name}")
        self.get_logger().info(f"GPS fix topic: {gps_fix_topic}")

        # -----------------------
        # Open CSV files + headers
        # -----------------------

        # Raw camera / radar detections
        self.raw_file = open(
            os.path.join(log_dir, f"raw_detections_{self.test_name}.csv"),
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
                f"Test was run at: {timestamp}",
            ]
        )

        # Fused detections
        self.fused_file = open(
            os.path.join(log_dir, f"fused_detections_{self.test_name}.csv"),
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
                f"Test was run at: {timestamp}",
            ]
        )

        # Tracks
        self.tracks_file = open(
            os.path.join(log_dir, f"tracks_{self.test_name}.csv"),
            "w",
            newline="",
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
                f"Test was run at: {timestamp}",
            ]
        )

        # Ego motion (GPS position + velocity)
        self.ego_file = open(
            os.path.join(log_dir, f"ego_motion_{self.test_name}.csv"),
            "w",
            newline="",
        )
        self.ego_writer = csv.writer(self.ego_file)
        self.ego_writer.writerow(
            [
                "logger_stamp",  # arrival to logger
                "header_stamp",  # message timestamp
                "x",  # GPS-based x position in local frame (m)
                "y",  # GPS-based y position in local frame (m)
                "yaw",  # Heading from integrated yaw rate (rad)
                "v",  # forward speed from ego_motion (m/s)
                "yaw_rate",  # yaw rate from ego_motion (rad/s)
                f"Test was run at: {timestamp}",
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

        # Ego motion velocity/yaw rate (from gps_ego_motion node)
        self.create_subscription(TWCS, "/ego_motion", self.handle_ego_motion, 20)

        # GPS position (for ground truth position logging)
        self.create_subscription(NavSatFix, gps_fix_topic, self.handle_gps_fix, 20)

        self.get_logger().info(
            f'RealWorldLogger started, logging to "{log_dir}" '
            f'for test "{self.test_name}"'
        )

    # -----------------------
    # Time helpers
    # -----------------------

    def _stamp_to_seconds(self, time_msg) -> float:
        """Convert ROS2 Time message to seconds."""
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
        for detection in msg.detections:
            header_time = self._stamp_to_seconds(detection.header.stamp)
            logger_time = self._get_current_time()
            self.tracks_writer.writerow(
                [
                    logger_time,
                    header_time,
                    detection.tracking_id,
                    detection.position.x,
                    detection.position.y,
                    detection.position.z,
                    detection.distance,
                    detection.speed,
                    detection.age,
                    detection.consecutive_misses,
                ]
            )

    def handle_gps_fix(self, msg: NavSatFix):
        """Convert GPS lat/lon to local x/y coordinates using tangent plane projection."""
        # Set origin on first GPS message
        if self.origin_lat is None:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.origin_alt = msg.altitude
            self.get_logger().info(
                f"GPS origin set: lat={self.origin_lat:.6f}, lon={self.origin_lon:.6f}"
            )
            return

        # Convert lat/lon to local x/y using equirectangular approximation
        # (Accurate enough for distances within a few km)
        origin_lat_rad = math.radians(self.origin_lat)

        # Earth radius in meters
        R = 6371000.0

        # Δx = R * cos(lat) * Δlon (East)
        # Δy = R * Δlat (North)
        dx = R * math.cos(origin_lat_rad) * math.radians(msg.longitude - self.origin_lon)
        dy = R * math.radians(msg.latitude - self.origin_lat)

        self.x = dx
        self.y = dy

    def handle_ego_motion(self, msg: TWCS):
        """Log ego motion (velocity + estimated yaw from integration)."""
        header_time = self._stamp_to_seconds(msg.header.stamp)
        logger_time = self._get_current_time()

        self.vx = msg.twist.twist.linear.x
        self.yaw_rate = msg.twist.twist.angular.z

        # Integrate yaw rate to estimate heading
        if self.last_ego_time is not None:
            dt = header_time - self.last_ego_time

            # Clip dt to reasonable bounds (avoid huge jumps from message gaps)
            if 0.001 < dt < 1.0:
                self.yaw += self.yaw_rate * dt

                # Normalize yaw to [-pi, pi]
                self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        self.last_ego_time = header_time

        # Log ego motion with or without GPS position
        # If GPS available: use actual GPS position (x, y from lat/lon)
        # If GPS unavailable: log zeros for x, y (ego-motion compensation won't work,
        # but still trackable)
        self.ego_writer.writerow(
            [
                logger_time,
                header_time,
                self.x,
                self.y,
                self.yaw,
                self.vx,
                self.yaw_rate,
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
    node = RealWorldLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Logger stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
