# Copyright 2024 Eemil Kulmala, University of Oulu
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

import unittest
from unittest.mock import patch

import can
from radar_interface.radar_node import RadarNode
import rclpy


class TestRadarNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 before any test is run."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS2 after all tests are completed."""
        rclpy.shutdown()

    def setUp(self):
        """Initialize RadarNode with a mocked CAN bus."""
        # Mock `listen_to_can()` so it does nothing
        with patch.object(RadarNode, 'listen_to_can'), \
                patch('can.interface.Bus') as MockBus:

            self.mock_can_bus = MockBus.return_value
            self.radar_node = RadarNode()
            self.radar_node.bus = self.mock_can_bus

        # Mock publisher output
        self.published_detections = []

        def mock_publisher(radar_msg):
            """Mock method to capture published messages."""
            self.published_detections.append(radar_msg)

        self.radar_node.publisher_.publish = mock_publisher

    def tearDown(self):
        """Destroy the RadarNode after each test."""
        self.radar_node.destroy_node()

    def create_radar_data(self, x, y, z, frame_id, speed):
        """Create a CAN-message format radar frame."""
        cluster_id = frame_id     # Cluster ID: 0-255 (8 bits)
        dist_long = x       # Longitudinal distance in meters
        dist_lat = y        # Lateral distance in meters
        vrel_long = speed   # Relative velocity in m/s
        height = z          # Height in meters
        # Frame 0: dist_long, dist_lat, vrel_long
        frame0 = self.format_frame0(cluster_id, dist_long, dist_lat, vrel_long)
        msg0 = can.Message(arbitration_id=0x701, data=frame0, is_extended_id=False)

        # Frame 1: height
        frame1 = self.format_frame1(cluster_id, height)
        msg1 = can.Message(arbitration_id=0x701, data=frame1, is_extended_id=False)

        return msg0, msg1

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
    def format_frame1(cluster_id, height):
        """Format frame 1 of radar detection message."""
        height_scaled = int((height + 30) / 0.1) & 0x03FF  # height range [-30, 70]

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
        frame[7] = 0x00

        return bytes(frame)

    def test_radar_node(self):
        """Ensure that radar data is correctly extracted and published."""
        x, y, z = 3.0, 12.0, 0.0
        frame_id = 1
        speed = 2
        frame0, frame1 = self.create_radar_data(x, y, z, frame_id, speed)
        self.radar_node.process_radar_data(frame0)
        self.radar_node.process_radar_data(frame1)

        # Check if detection was published
        self.assertEqual(len(self.published_detections), 1,
                         msg='Detection should have been published once.')

        # Verify that the detection has expected values
        radar_detection = self.published_detections[0]

        self.assertEqual(radar_detection.header.frame_id, f'target_{frame_id}',
                         msg='Detection should have correct ID')
        self.assertAlmostEqual(radar_detection.position.x, x, places=2,
                               msg='Fused position X should match radar')
        self.assertAlmostEqual(radar_detection.position.y, y, places=2,
                               msg='Fused position Y should match radar')
        self.assertEqual(radar_detection.speed, speed,
                         msg='Fused speed should match radar')


if __name__ == '__main__':
    unittest.main()
