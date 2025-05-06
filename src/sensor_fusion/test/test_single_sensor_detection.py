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

##############################################################################################
#                                                                                            #
# This module contains a set of unit tests for the FusionNode class, which is part of the    #
# tractor safety system. The tests are designed to ensure that the FusionNode correctly      #
# processes detections from single sensors, when trusted data isn't available from both.     #
#                                                                                            #
# Classes:                                                                                   #
#    TestSingleSensorDetection: A unittest.TestCase subclass that contains tests for the     #
#    FusionNode class.                                                                       #
#                                                                                            #
# Methods:                                                                                   #
#    setUpClass(cls): Initializes ROS2 before any test is run.                               #
#    tearDownClass(cls): Shuts down ROS2 after all tests are completed.                      #
#    setUp(self): Initializes a FusionNode instance with a mocked publisher.                 #
#    create_camera_detection(self, x, y, z, hypothesis, tracking_id): Creates a              #
#    CameraDetection message.                                                                #
#    create_radar_detection(self, x, y, z, speed, frame_id): Creates a RadarDetection        #
#    message.                                                                                #
#    test_single_detection_handling(self): Ensures correct handling of individual radar      #
#    and camera detections, including trust thresholds and fusion logic.                     #
#                                                                                            #
##############################################################################################

import unittest

from geometry_msgs.msg import Point
import numpy as np
import rclpy
from sensor_fusion.fusion_node import FusionNode
from std_msgs.msg import Header
from tractor_safety_system_interfaces.msg import CameraDetection, RadarDetection
from vision_msgs.msg import ObjectHypothesis


class TestSingleSensorDetection(unittest.TestCase):

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

    def create_camera_detection(self, x, y, z, hypothesis, tracking_id='test_camera'):
        """Create a CameraDetection message."""
        msg = CameraDetection()
        msg.header = Header()
        msg.header.stamp.sec = 1  # Simulated timestamp
        msg.header.stamp.nanosec = 0
        msg.position = Point(x=x, y=y, z=z)
        msg.tracking_id = tracking_id
        msg.results = []
        msg.results.append(hypothesis)
        return msg

    def create_radar_detection(self, x, y, z, speed, frame_id='test_radar'):
        """Create a RadarDetection message."""
        msg = RadarDetection()
        msg.header = Header()
        msg.header.stamp.sec = 1
        msg.header.stamp.nanosec = 0
        msg.position = Point(x=x, y=y, z=z)
        msg.distance = (x**2 + y**2)/0.5
        msg.speed = speed
        msg.header.frame_id = frame_id
        return msg

    def test_single_detection_handling(self):
        """Ensure correct handling of individual radar and camera detections."""
        # Create a camera detection passing the trust threshold
        hypothesis1 = ObjectHypothesis(class_id='person', score=0.95)
        camera_msg_trusted = self.create_camera_detection(3.0, 3.0, 0.0, hypothesis1)
        # Create a radar detection passing the trust threshold
        radar_msg_trusted = self.create_radar_detection(10.0, 20.0, 0.0, speed=2)

        # Create a camera detection not passing the trust threshold
        hypothesis2 = ObjectHypothesis(class_id='person', score=0.4)
        camera_msg_untrusted = self.create_camera_detection(3.0, 3.0, 0.0, hypothesis2)

        # Create a radar detection not passing the trust threshold
        radar_msg_untrusted = self.create_radar_detection(1.0, 0.0, 0.0, speed=2)

        # Override parameters for testing
        self.fusion_node.time_threshold = 0.5
        self.fusion_node.distance_threshold = 1.0
        self.fusion_node.radar_trust_min = 4.0
        self.fusion_node.camera_trust_max = 12.0
        self.fusion_node.detection_score_trust = 0.6
        self.fusion_node.R = np.eye(3)  # Identity matrix (no rotation)
        self.fusion_node.T = np.zeros(3)  # No translation

        # Test with untrusted detections:

        # Add radar detection to the fusion node
        self.fusion_node.radar_detections.append(radar_msg_untrusted)

        # Attempt fusion
        self.fusion_node.attempt_fusion()

        # Add camera detection to the fusion node
        self.fusion_node.camera_detections.append(camera_msg_untrusted)

        # Attempt fusion
        self.fusion_node.attempt_fusion()

        # Check if no detections have been published
        self.assertEqual(len(self.published_detections), 0,
                         msg='No detections should have been published.')

        # Test with trusted detections:

        # Add radar detection to the fusion node
        self.fusion_node.radar_detections.append(radar_msg_trusted)

        # Attempt fusion
        self.fusion_node.attempt_fusion()

        # Add camera detection to the fusion node
        self.fusion_node.camera_detections.append(camera_msg_trusted)

        # Attempt fusion
        self.fusion_node.attempt_fusion()

        # Check if both detections have been published
        self.assertEqual(len(self.published_detections), 2,
                         msg='Both detections should have been published.')

        # Verify that the sent detections have expected values
        radar_detection = self.published_detections[0]

        self.assertEqual(radar_detection.detection_type, 'radar',
                         msg='Sent radar detection should be of type radar')
        self.assertAlmostEqual(radar_detection.position.x,
                               radar_msg_trusted.position.x,
                               places=2,
                               msg='Fused position X should match radar')
        self.assertAlmostEqual(radar_detection.position.y,
                               radar_msg_trusted.position.y,
                               places=2,
                               msg='Fused position Y should match radar')
        self.assertEqual(radar_detection.distance,
                         radar_msg_trusted.distance,
                         msg='Fused distance should match radar')
        self.assertEqual(radar_detection.speed,
                         radar_msg_trusted.speed,
                         msg='Fused speed should match radar')

        camera_detection = self.published_detections[1]

        self.assertEqual(camera_detection.detection_type, 'camera',
                         msg='Sent camera detection should be of type camera')
        self.assertAlmostEqual(camera_detection.position.x,
                               camera_msg_trusted.position.z,
                               places=2,
                               msg='Position X should match camera transferred to radar')
        self.assertAlmostEqual(camera_detection.position.y,
                               -camera_msg_trusted.position.x,
                               places=2,
                               msg='Position Y should match camera')


if __name__ == '__main__':
    unittest.main()
