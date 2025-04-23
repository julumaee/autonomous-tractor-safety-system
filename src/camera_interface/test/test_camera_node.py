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
# This module contains a set of unit tests for the CameraNode class, which is part of the    #
# tractor safety system. The tests are designed to ensure that the CameraNode correctly      #
# processes and publishes camera detection data.                                             #
#                                                                                            #
# Classes:                                                                                   #
#    TestCameraNode: A unittest.TestCase subclass that contains tests for the CameraNode     #
#    class.                                                                                  #
#                                                                                            #
# Methods:                                                                                   #
#    setUpClass(cls): Initializes ROS2 before any test is run.                               #
#    tearDownClass(cls): Shuts down ROS2 after all tests are completed.                      #
#    setUp(self): Initializes a CameraNode instance with a mocked CAN bus and publisher.     #
#    tearDown(self): Destroys the CameraNode instance after each test.                       #
#    create_camera_detection(self, x, y, z, tracking_id, object_hypothesis): Creates a       #
#    CameraDetection message.                                                                #
#    create_detection_array(self): Creates a simulated SpatialDetectionArray message with    #
#    two detections.                                                                         #
#    test_camera_node(self): Ensures that camera data is correctly extracted and published.  #
#                                                                                            #
##############################################################################################

import unittest

from camera_interface.camera_node import CameraNode
from depthai_ros_msgs.msg import SpatialDetection, SpatialDetectionArray
from geometry_msgs.msg import Point
import rclpy
from std_msgs.msg import Header
from vision_msgs.msg import ObjectHypothesis


class TestCameraNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 before any test is run."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS2 after all tests are completed."""
        rclpy.shutdown()

    def setUp(self):
        """Initialize CameraNode with a mocked CAN bus."""
        self.camera_node = CameraNode()

        # Mock publisher output
        self.published_detections = []

        def mock_publisher(camera_msg):
            """Mock method to capture published messages."""
            self.published_detections.append(camera_msg)

        self.camera_node.publisher_.publish = mock_publisher

    def tearDown(self):
        """Destroy the CameraNode after each test."""
        self.camera_node.destroy_node()

    def create_camera_detection(self, x, y, z, tracking_id, object_hypothesis):
        """Create a CameraDetection message."""
        msg = SpatialDetection()
        msg.position = Point(x=x, y=y, z=z)
        msg.is_tracking = True
        msg.tracking_id = f'object_{tracking_id}'
        msg.results = []
        msg.results.append(object_hypothesis)
        return msg

    def create_detection_array(self):
        """Create a simulated SpatialDetectionArray message with 2 detections."""
        detection_array = SpatialDetectionArray()
        detection_array.header = Header()
        detection_array.header.stamp.sec = 1  # Simulated timestamp
        detection_array.header.stamp.nanosec = 0
        detection_array.header.frame_id = 'oakd_camera_frame'

        # Detection 1:
        object_1 = ObjectHypothesis(class_id='person', score=0.95)
        detection1 = self.create_camera_detection(3.0, 12.0, 0.0, 1, object_1)

        # Detection 2:
        object_2 = ObjectHypothesis(class_id='car', score=0.8)
        detection2 = self.create_camera_detection(10.0, 5.0, 0.0, 2, object_2)

        # Add detections to the array
        detection_array.detections.append(detection1)
        detection_array.detections.append(detection2)

        return detection_array

    def test_camera_node(self):
        """Ensure that camera data is correctly extracted and published."""
        # Create a simulated SpatialDetectionArray message with 2 detections
        camera_msg = self.create_detection_array()
        self.camera_node.publish_detections(camera_msg)

        # Check if detections were published
        self.assertEqual(len(self.published_detections), 2,
                         msg='Two detections should have been published.')

        # Verify that the detections have expected values
        camera_detection_1 = self.published_detections[0]
        camera_detection_2 = self.published_detections[1]

        # Verify the first detection
        self.assertEqual(camera_detection_1.header.frame_id,
                         camera_msg.detections[0].results[0].class_id,
                         msg='Detection 1 should have correct ID')
        self.assertEqual(camera_detection_1.results[0].class_id,
                         camera_msg.detections[0].results[0].class_id,
                         msg='Detection 1 results should retain class ID')
        self.assertEqual(camera_detection_1.tracking_id,
                         camera_msg.detections[0].tracking_id,
                         msg='Detection 1 should retain tracking ID')
        self.assertAlmostEqual(camera_detection_1.position.x,
                               camera_msg.detections[0].position.x, places=2,
                               msg='Detection 1 position X should match camera')
        self.assertAlmostEqual(camera_detection_1.position.y,
                               camera_msg.detections[0].position.y, places=2,
                               msg='Detection 1 position Y should match camera')

        # Verify the second detection
        self.assertEqual(camera_detection_2.header.frame_id,
                         camera_msg.detections[1].results[0].class_id,
                         msg='Detection 2 should have correct ID')
        self.assertEqual(camera_detection_2.results[0].class_id,
                         camera_msg.detections[1].results[0].class_id,
                         msg='Detection 2 results should retain class ID')
        self.assertEqual(camera_detection_2.tracking_id,
                         camera_msg.detections[1].tracking_id,
                         msg='Detection 2 should retain tracking ID')
        self.assertAlmostEqual(camera_detection_2.position.x,
                               camera_msg.detections[1].position.x, places=2,
                               msg='Detection 2 position X should match camera')
        self.assertAlmostEqual(camera_detection_2.position.y,
                               camera_msg.detections[1].position.y, places=2,
                               msg='Detection 2 position Y should match camera')


if __name__ == '__main__':
    unittest.main()
