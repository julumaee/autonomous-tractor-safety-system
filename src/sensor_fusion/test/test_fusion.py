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
#    test_fusion_performs_correctly(self): Ensures that radar and camera detections are      #
#    correctly fused and published.                                                          #
#                                                                                            #
##############################################################################################

import unittest

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from std_msgs.msg import Header

from sensor_fusion.fusion_node import FusionNode
from tractor_safety_system_interfaces.msg import CameraDetection, RadarDetection


class TestFusionNode(unittest.TestCase):

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
        self.published_fusions = []

        def mock_publisher(fused_msg):
            """Mock method to capture published messages."""
            self.published_fusions.append(fused_msg)

        self.fusion_node.publisher_.publish = mock_publisher

    def create_camera_detection(self, x, y, z):
        """Create a CameraDetection message."""
        msg = CameraDetection()
        msg.header = Header()
        msg.header.stamp = self.fusion_node.get_clock().now().to_msg()
        msg.position = Point(x=x, y=y, z=z)
        return msg

    def create_radar_detection(self, x, y, z, distance, speed):
        """Create a RadarDetection message."""
        msg = RadarDetection()
        msg.header = Header()
        msg.header.stamp = self.fusion_node.get_clock().now().to_msg()
        msg.position = Point(x=x, y=y, z=z)
        msg.distance = distance
        msg.speed = speed
        return msg

    def test_fusion_performs_correctly(self):
        """Ensure that radar and camera detections are correctly fused and published."""
        camera_msg = self.create_camera_detection(12.0, 3.0, 0.0)
        radar_msg = self.create_radar_detection(
            12.0, 3.0, 0.0, distance=np.sqrt(12.0**2 + 3.0**2), speed=2
        )

        # Add detections to the fusion node
        self.fusion_node.camera_detections.append(camera_msg)
        self.fusion_node.radar_detections.append(radar_msg)

        # Perform fusion
        self.fusion_node.attempt_fusion()

        # Check if fusion was published
        self.assertEqual(
            len(self.published_fusions),
            1,
            msg="Fusion should have been published once.",
        )

        # Verify that the fused detection has expected values
        fused_detection = self.published_fusions[0]

        self.assertEqual(
            fused_detection.detection_type,
            "fused",
            msg="Detection type should be fused",
        )
        self.assertAlmostEqual(
            fused_detection.position.x,
            radar_msg.position.x,
            places=1,
            msg="Fused position X should match true position",
        )
        self.assertAlmostEqual(
            fused_detection.position.y,
            radar_msg.position.y,
            places=1,
            msg="Fused position Y should match true position",
        )
        self.assertAlmostEqual(
            fused_detection.distance,
            radar_msg.distance,
            places=1,
            msg="Fused distance should match radar",
        )
        self.assertEqual(
            fused_detection.speed, radar_msg.speed, msg="Fused speed should match radar"
        )


if __name__ == "__main__":
    unittest.main()
