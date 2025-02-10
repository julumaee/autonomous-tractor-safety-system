##############################################################################################
#                                                                                            #
# This module contains a set of unit tests for the FusionNode class, which is part of the    #
# tractor safety system. The tests are designed to ensure that the FusionNode correctly      #
# processes and fuses camera and radar detection data.                                       #
#                                                                                            #
# Classes:                                                                                   #
#    TestFusionNode: A unittest.TestCase subclass that contains tests for the FusionNode     #
#    class.                                                                                  #
#                                                                                            #
# Methods:                                                                                   #
#    setUpClass(cls): Initializes ROS2 before any test is run.                               #
#    tearDownClass(cls): Shuts down ROS2 after all tests are completed.                      #
#    setUp(self): Initializes a FusionNode instance with a mocked publisher.                 #
#    create_camera_detection(self, x, y, z, tracking_id): Creates a CameraDetection message. #
#    create_radar_detection(self, x, y, z, distance, speed, frame_id): Creates a             #
#    RadarDetection message.                                                                 #
#    test_untrusted_detections_handling(self): Ensures that untrusted radar and camera       #
#    detections are handled correctly.                                                       #
#                                                                                            #
##############################################################################################

import unittest

from geometry_msgs.msg import Point
import numpy as np
import rclpy
from sensor_fusion.fusion_node import FusionNode
from std_msgs.msg import Header
from tractor_safety_system_interfaces.msg import CameraDetection, RadarDetection


class TestDetectionLimits(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 before any test is run."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS2 after all tests are completed."""
        rclpy.shutdown()

    def setUp(self):
        """Initialize the FusionNode for testing."""
        self.fusion_node = FusionNode()

        # Mock publisher output
        self.published_detections = []

        def mock_publisher(fused_msg):
            """Mock method to capture published messages."""
            self.published_detections.append(fused_msg)

        self.fusion_node.publisher_.publish = mock_publisher

    def create_camera_detection(self, x, y, z, tracking_id='test_camera'):
        """Create a CameraDetection message."""
        msg = CameraDetection()
        msg.header = Header()
        msg.header.stamp.sec = 1  # Simulated timestamp
        msg.header.stamp.nanosec = 0
        msg.position = Point(x=x, y=y, z=z)
        msg.tracking_id = tracking_id
        return msg

    def create_radar_detection(self, x, y, z, distance, speed, frame_id='test_radar'):
        """Create a RadarDetection message."""
        msg = RadarDetection()
        msg.header = Header()
        msg.header.stamp.sec = 1
        msg.header.stamp.nanosec = 0
        msg.position = Point(x=x, y=y, z=z)
        msg.distance = distance
        msg.speed = speed
        msg.header.frame_id = frame_id
        return msg

    def test_untrusted_detections_handling(self):
        """Ensure that untrusted radar and camera detections are handled correctly."""
        camera_too_far = self.create_camera_detection(20.0, 30.0, 0.0)
        radar_too_close = self.create_radar_detection(1.0, 0.0, 0.0, distance=1, speed=2)

        # Override parameters for testing
        self.fusion_node.time_threshold = 0.5
        self.fusion_node.distance_threshold = 1.0
        self.fusion_node.radar_trust_min = 4.0
        self.fusion_node.camera_trust_max = 12.0
        self.fusion_node.R = np.eye(3)  # Identity matrix (no rotation)
        self.fusion_node.T = np.zeros(3)  # No translation

        # Add camera detection to the fusion node
        self.fusion_node.camera_detections.append(camera_too_far)

        # Attempt fusion
        self.fusion_node.attempt_fusion()

        # Check if the detection was published
        self.assertEqual(len(self.published_detections), 0,
                         msg='Camera detection too far, nothing should have been published.')

        # Check if the detection was removed from the deque
        self.assertEqual(len(self.fusion_node.camera_detections), 0,
                         msg='Camera detection too far, \
                         should have been removed from the deque')

        # Add radar detection to the fusion node
        self.fusion_node.radar_detections.append(radar_too_close)

        # Attempt fusion
        self.fusion_node.attempt_fusion()

        # Check if the detection was published
        self.assertEqual(len(self.published_detections), 0,
                         msg='Radar detection too close, nothing should have been published.')

        # Check if the detection was removed from the deque
        self.assertEqual(len(self.fusion_node.radar_detections), 0,
                         msg='Radar detection too close, \
                         should have been removed from the deque')


if __name__ == '__main__':
    unittest.main()
