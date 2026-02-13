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

import random

import can
import rclpy
from rclpy.node import Node

from tractor_safety_system_interfaces.msg import SimulatedObject


class RadarSimulator(Node):

    def __init__(self):
        super().__init__("radar_simulator")
        self.object_subscription = self.create_subscription(
            SimulatedObject, "/simulated_objects", self.send_radar_data, 10
        )
        try:
            self.bus = can.interface.Bus(channel="vcan0", bustype="socketcan")
            self.get_logger().info("Connected to CAN bus (vcan0)")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to CAN bus: {e}")
            self.bus = None

    def send_radar_data(self, simulated_object):
        """Send radar data frames over the virtual CAN bus."""
        if self.bus is None:
            self.get_logger().warn("CAN bus not connected, cannot send radar data")
            return

        vrel_long = random.uniform(-30.0, 30.0)
        cluster_id = simulated_object.object_id & 0x7F
        dist_long = simulated_object.position.x
        dist_lat = simulated_object.position.y
        # The radar node filters by min_height_m (default 0.6). Ensure simulated
        # objects generate detections that pass the filter.
        height = max(float(simulated_object.position.z), 1.0)

        # Frame 0: dist_long, dist_lat, vrel_long
        frame0 = self.format_frame0(cluster_id, dist_long, dist_lat, vrel_long)
        msg0 = can.Message(arbitration_id=0x701, data=frame0, is_extended_id=False)

        # Frame 1: height + confidence (radar_node decodes confidence from byte7 bits0..5)
        frame1 = self.format_frame1(cluster_id, height, confidence_percent=100.0)
        msg1 = can.Message(arbitration_id=0x701, data=frame1, is_extended_id=False)

        try:
            self.bus.send(msg0)
            self.bus.send(msg1)
            self.get_logger().info(f"Sent Radar Frames for ID={cluster_id}")
        except can.CanError as e:
            self.get_logger().error(f"Failed to send radar frames: {e}")

    @staticmethod
    def format_frame0(cluster_id, dist_long, dist_lat, vrel_long):
        """Format frame 0 of radar detection message."""
        dist_long_scaled = int((dist_long + 100) / 0.05) & 0x1FFF
        dist_lat_scaled = int((dist_lat + 50) / 0.05) & 0x07FF
        vrel_long_scaled = int((vrel_long + 128) / 0.25) & 0x03FF

        frame = bytearray(8)

        # Byte 0: MSB = 0 (frame 0), lower 7 bits = cluster_id
        frame[0] = cluster_id & 0x7F

        # Bytes 1-2: dist_long
        frame[1] = (dist_long_scaled >> 5) & 0xFF
        frame[2] = ((dist_long_scaled & 0x1F) << 3) & 0xF8
        frame[2] |= (dist_lat_scaled >> 8) & 0x07

        # Byte 3: dist_lat LSB
        frame[3] = dist_lat_scaled & 0xFF

        # Bytes 4-5: vrel_long
        frame[4] = (vrel_long_scaled >> 2) & 0xFF
        frame[5] = ((vrel_long_scaled & 0x03) << 6) & 0xC0

        # Remaining bytes unused in frame 0
        frame[6] = 0x00
        frame[7] = 0x00

        return bytes(frame)

    @staticmethod
    def format_frame1(cluster_id, height, confidence_percent=100.0):
        """Format frame 1 of radar detection message."""
        height_scaled = int((height + 30) / 0.1) & 0x03FF  # height range [-30, 70]

        # Confidence raw value encodes percent as raw * 5.0.
        raw_conf = int(round(float(confidence_percent) / 5.0))
        raw_conf = max(0, min(0x3F, raw_conf))

        frame = bytearray(8)

        # Byte 0: MSB = 1 (frame 1), lower 7 bits = cluster_id
        frame[0] = (1 << 7) | (cluster_id & 0x7F)

        # Bytes 1-2: height
        frame[1] = (height_scaled >> 2) & 0xFF
        frame[2] = ((height_scaled & 0x03) << 6) & 0xC0

        # Remaining bytes unused in frame 1
        frame[3] = 0x00
        frame[4] = 0x00
        frame[5] = 0x00
        frame[6] = 0x00
        frame[7] = raw_conf & 0x3F

        return bytes(frame)


def main(args=None):
    rclpy.init(args=args)
    radar_simulator = RadarSimulator()
    rclpy.spin(radar_simulator)
    radar_simulator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
