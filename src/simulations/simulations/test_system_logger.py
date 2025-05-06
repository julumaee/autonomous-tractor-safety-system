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
from datetime import datetime
import os

import rclpy
from rclpy.node import Node
from tractor_safety_system_interfaces.msg import (
    CameraDetection,
    ControlCommand,
    FusedDetection,
    RadarDetection,
)


class TestLogger(Node):

    def __init__(self):
        super().__init__('test_logger_node')

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        os.makedirs('test_logs', exist_ok=True)

        # Combined fused+control log
        self.fused_file = open(f'test_logs/test_log_{timestamp}.csv', 'w', newline='')
        self.fused_writer = csv.writer(self.fused_file)

        # Raw radar/camera detections
        self.raw_file = open(f'test_logs/raw_detections_{timestamp}.csv', 'w', newline='')
        self.raw_writer = csv.writer(self.raw_file)
        self.raw_writer.writerow([
            'time', 'x', 'y', 'z', 'sensor_type'
        ])

        # Subscriptions
        self.create_subscription(FusedDetection, '/fused_detections', self.handle_fused, 10)
        self.create_subscription(ControlCommand, '/control', self.handle_control, 10)
        self.create_subscription(RadarDetection, '/radar_detections', self.handle_radar, 10)
        self.create_subscription(CameraDetection, '/camera_detections', self.handle_camera, 10)

    def get_time_str(self):
        now = self.get_clock().now().to_msg()
        timestamp = now.sec + now.nanosec * 1e-9
        return datetime.fromtimestamp(timestamp).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]

    def handle_fused(self, msg):
        if msg.detection_type == 'fused':
            self.log_fused(msg)

    def handle_control(self, msg):
        self.log_control(msg)

    def handle_radar(self, msg):
        self.log_raw(msg, 'radar')

    def handle_camera(self, msg):
        self.log_raw(msg, 'camera')

    def log_fused(self, msg):
        time_str = self.get_time_str()

        self.fused_writer.writerow([
            time_str,
            'Fused detection at:',
            msg.position.x, msg.position.y, msg.position.z,
            'Distance: ',
            msg.distance
        ])

    def log_control(self, msg):
        time_str = self.get_time_str()
        self.fused_writer.writerow([
            time_str,
            'Current speed:',
            msg.speed
        ])

    def log_raw(self, msg, sensor_type):
        time_str = self.get_time_str()
        self.raw_writer.writerow([
            time_str,
            msg.position.x,
            msg.position.y,
            msg.position.z,
            sensor_type
        ])

    def destroy_node(self):
        self.fused_file.close()
        self.raw_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TestLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
