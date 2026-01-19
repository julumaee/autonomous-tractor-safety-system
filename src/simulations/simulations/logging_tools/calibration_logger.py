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

import csv
import os
import time
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node

from tractor_safety_system_interfaces.msg import CameraDetection, RadarDetection


class CalibrationLogger(Node):

    def __init__(self):
        super().__init__("calibration_logger")

        # Create a CSV file to store results
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"calibration_log/calibration_data_{timestamp}.csv"
        os.makedirs("calibration_log", exist_ok=True)

        self.file = open(self.filename, mode="w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(["timestamp", "x", "y", "z", "sensor_type"])

        self.object_subscription = self.create_subscription(
            RadarDetection, "/radar_detections", self.handle_radar_data, 10
        )

        self.camera_subscriber = self.create_subscription(
            CameraDetection, "/camera_detections", self.handle_camera_data, 10
        )

        self.latest_radar = None
        self.latest_camera = None

        self.last_logged_time = 0.0
        self.log_interval_sec = 1.0  # log at most once per second
        self.matching_distance_threshold = 2.0  # meters

    def destroy_node(self):
        self.file.close()
        super().destroy_node()

    def handle_radar_data(self, msg):
        self.latest_radar = msg
        self.try_log_pair()

    def handle_camera_data(self, msg):
        self.latest_camera = msg
        self.try_log_pair()

    def try_log_pair(self):
        """Log if radar and camera detections are spatially close and within logging rate."""
        if not self.latest_radar or not self.latest_camera:
            return

        now = time.time()
        if now - self.last_logged_time < self.log_interval_sec:
            return

        # Calculate Euclidean distance
        radar_distance = np.linalg.norm(
            [
                self.latest_radar.position.x,
                self.latest_radar.position.y,
                self.latest_radar.position.z,
            ]
        )
        camera_distance = np.linalg.norm(
            [
                self.latest_camera.position.x,
                self.latest_camera.position.y,
                self.latest_camera.position.z,
            ]
        )
        distance_diff = abs(radar_distance - camera_distance)

        if distance_diff <= self.matching_distance_threshold:
            self.write_detection("radar", self.latest_radar)
            self.write_detection("camera", self.latest_camera)
            self.last_logged_time = now

    def write_detection(self, sensor_type, msg):
        """Store detection timestamp and position."""
        timestamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt_object = datetime.fromtimestamp(timestamp_sec)
        human_readable_time = dt_object.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        self.writer.writerow(
            [
                human_readable_time,
                msg.position.x,
                msg.position.y,
                msg.position.z,
                sensor_type,
            ]
        )


def main(args=None):
    rclpy.init(args=args)
    calibration_logger = CalibrationLogger()
    rclpy.spin(calibration_logger)
    calibration_logger.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
