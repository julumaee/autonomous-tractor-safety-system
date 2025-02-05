import unittest
from unittest.mock import patch

import can
from radar_interface.radar_node_can import RadarNode
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
        frame_data = self.format_radar_frame(cluster_id,
                                             dist_long,
                                             dist_lat,
                                             vrel_long,
                                             height)

        # Return a CAN frame
        return can.Message(arbitration_id=0x701, data=frame_data, is_extended_id=False)

    @staticmethod
    def format_radar_frame(cluster_id, dist_long, dist_lat, vrel_long, height):
        """Format a radar CAN frame based on the specified bit layout."""
        # Scale and encode fields
        dist_long_scaled = int((dist_long + 100) / 0.05) & 0x1FFF  # 13 bits
        dist_lat_scaled = int((dist_lat + 50) / 0.05) & 0x07FF     # 11 bits
        vrel_long_scaled = int((vrel_long + 128) / 0.25) & 0x03FF  # 10 bits
        height_scaled = int((height + 64) / 0.25) & 0x03FF         # 10 bits
        dyn_prop = 0x00

        # Pack the data into bytes according to the memory layout
        frame = bytearray(8)

        # Byte 0: Cluster_ID
        frame[0] = cluster_id & 0xFF

        # Bytes 1-2: DistLong (13 bits)
        frame[1] = (dist_long_scaled >> 5) & 0xFF
        frame[2] = ((dist_long_scaled & 0x1F) << 3) & 0xF8

        # Add DistLat (11 bits)
        frame[2] |= (dist_lat_scaled >> 8) & 0x07
        frame[3] = dist_lat_scaled & 0xFF

        # Bytes 4-5: VrelLong (10 bits) and Height (10 bits)
        frame[4] = (vrel_long_scaled >> 2) & 0xFF
        frame[5] = ((vrel_long_scaled & 0x03) << 6) & 0xC0
        frame[5] |= (height_scaled >> 4) & 0x3F

        # Bytes 6-7: Height (remaining 6 bits) dynProp (3 bits) and RCS (8 bits)
        frame[6] = ((height_scaled & 0x0F) << 4) & 0xF0  # Height (remaining bits)
        frame[6] |= (dyn_prop << 1) & 0x0E  # DynProp (3 bits, shifted left 1)

        frame[7] = 0x00  # RCS set to 0

        return bytes(frame)

    def test_radar_node(self):
        """Ensure that radar data is correctly extracted and published."""
        x, y, z = 3.0, 12.0, 0.0
        frame_id = 1
        speed = 2
        radar_frame = self.create_radar_data(x, y, z, frame_id, speed)
        self.radar_node.process_radar_data(radar_frame)

        # Check if fusion was published
        self.assertEqual(len(self.published_detections), 1,
                         msg='Detection should have been published once.')

        # Verify that the fused detection has expected values
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
