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
# This module contains a set of unit tests for the SafetyMonitor class, which is part of the #
# tractor safety system. The tests are designed to ensure that the SafetyMonitor correctly   #
# processes fused detection data and transitions vehicle states accordingly.                 #
#                                                                                            #
# Classes:                                                                                   #
#    TestFusionNode: A unittest.TestCase subclass that contains tests for the SafetyMonitor  #
#    class.                                                                                  #
#                                                                                            #
# Methods:                                                                                   #
#    setUpClass(cls): Initializes ROS2 before any test is run.                               #
#    tearDownClass(cls): Shuts down ROS2 after all tests are completed.                      #
#    setUp(self): Initializes a SafetyMonitor instance for testing.                          #
#    create_fused_detection(self, distance): Creates a FusedDetection message with the       #
#    given distance.                                                                         #
#    test_safety_monitor_state_transitions(self): Ensures that the safety monitor states     #
#    transition correctly based on the detection distances.                                  #
#                                                                                            #
##############################################################################################

import unittest

import rclpy

from safety_monitor.safety_monitor import SafetyMonitor
from tractor_safety_system_interfaces.msg import FusedDetection


class TestSafetyMonitorStates(unittest.TestCase):

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
        self.safety_monitor = SafetyMonitor()

    def create_fused_detection(self, distance):
        """Create a FusedDetection message with the given distance and header."""
        fused_msg = FusedDetection()
        fused_msg.distance = distance
        return fused_msg

    def test_safety_monitor_state_transitions(self):
        """Ensure that the safety monitor states transition correctly."""
        # Override the default parameters for testing
        self.safety_monitor.safety_distance_1 = 40
        self.safety_monitor.safety_distance_2 = 10
        self.safety_monitor.stop_distance = 3
        self.safety_monitor.speed_override_1 = 5
        self.safety_monitor.speed_override_2 = 2
        self.safety_monitor.detection_active_reset_time = 5.0
        self.safety_monitor.vehicle_stopped_reset_time = 5.0

        # Test that the vehicle state transitions from 'agopen'...

        # ... to 'moderate', when a detection is inside the safety_distance_1 parameter
        self.safety_monitor.vehicle_state = "agopen"
        detection = self.create_fused_detection(30)  # Inside safety_distance_1
        self.safety_monitor.control_speed_state(detection)
        self.assertEqual(
            self.safety_monitor.vehicle_state,
            "moderate",
            msg="Vehicle state should be 'moderate'\
                         when detection is inside safety_distance_1",
        )

        # ... to 'slow', when a detection is inside the safety_distance_2 parameter
        self.safety_monitor.vehicle_state = "agopen"
        detection = self.create_fused_detection(8)  # Inside safety_distance_2
        self.safety_monitor.control_speed_state(detection)
        self.assertEqual(
            self.safety_monitor.vehicle_state,
            "slow",
            msg="Vehicle state should be 'slow'\
                         when detection is inside safety_distance_2",
        )

        # ... to 'stopped', when a detection is inside the stop_distance parameter
        self.safety_monitor.vehicle_state = "agopen"
        detection = self.create_fused_detection(2)  # Inside stop_distance
        self.safety_monitor.control_speed_state(detection)
        self.assertEqual(
            self.safety_monitor.vehicle_state,
            "stopped",
            msg="Vehicle state should be 'stopped'\
                         when detection is inside stop_distance",
        )

        # Test that the vehicle state transitions from 'moderate'...

        # ... to 'slow', when a detection is inside the safety_distance_2 parameter
        self.safety_monitor.vehicle_state = "moderate"
        detection = self.create_fused_detection(8)  # Inside safety_distance_2
        self.safety_monitor.control_speed_state(detection)
        self.assertEqual(
            self.safety_monitor.vehicle_state,
            "slow",
            msg="Vehicle state should be 'slow'\
                         when detection is inside safety_distance_2",
        )

        # ... to 'stopped', when a detection is inside the stop_distance parameter
        self.safety_monitor.vehicle_state = "moderate"
        detection = self.create_fused_detection(2)  # Inside stop_distance
        self.safety_monitor.control_speed_state(detection)
        self.assertEqual(
            self.safety_monitor.vehicle_state,
            "stopped",
            msg="Vehicle state should be 'stopped'\
                         when detection is inside stop_distance",
        )

        # ... to 'agopen', ONLY when active_detection_reset_time has passed
        self.safety_monitor.last_detection_time_1 = (
            self.safety_monitor.last_detection_time_1
            - rclpy.duration.Duration(
                seconds=self.safety_monitor.detection_active_reset_time + 1
            )
        )
        self.safety_monitor.last_detection_time_2 = (
            self.safety_monitor.last_detection_time_2
            - rclpy.duration.Duration(
                seconds=self.safety_monitor.detection_active_reset_time + 1
            )
        )
        self.safety_monitor.vehicle_state = "moderate"
        detection = self.create_fused_detection(30)  # Inside safety_distance_1
        self.safety_monitor.control_speed_state(detection)
        self.assertEqual(
            self.safety_monitor.vehicle_state,
            "moderate",
            msg="Vehicle state should be 'moderate'\
                         when detection is inside safety_distance_1",
        )
        self.safety_monitor.state_control()
        self.assertEqual(
            self.safety_monitor.vehicle_state,
            "moderate",
            msg="Vehicle state should not change\
                         when active_detection_reset_time has not passed",
        )
        # Simulate waiting for active_detection_reset_time
        self.safety_monitor.last_detection_time_1 = (
            self.safety_monitor.get_clock().now()
            - rclpy.duration.Duration(
                seconds=self.safety_monitor.detection_active_reset_time + 1
            )
        )
        self.safety_monitor.state_control()
        self.assertEqual(
            self.safety_monitor.vehicle_state,
            "agopen",
            msg="Vehicle state should be 'agopen'\
                         when active_detection_reset_time has passed \
                         and the state was 'moderate'",
        )

        # Test that the vehicle state transitions from 'slow'...

        # ... to 'stopped', when a detection is inside the stop_distance parameter
        self.safety_monitor.vehicle_state = "slow"
        detection = self.create_fused_detection(2)  # Inside stop_distance
        self.safety_monitor.control_speed_state(detection)
        self.assertEqual(
            self.safety_monitor.vehicle_state,
            "stopped",
            msg="Vehicle state should be 'stopped'\
                         when detection is inside stop_distance",
        )

        # ... to 'moderate', ONLY when active_detection_reset_time has passed
        self.safety_monitor.last_detection_time_1 = (
            self.safety_monitor.last_detection_time_1
            - rclpy.duration.Duration(
                seconds=self.safety_monitor.detection_active_reset_time + 1
            )
        )
        self.safety_monitor.last_detection_time_2 = (
            self.safety_monitor.last_detection_time_2
            - rclpy.duration.Duration(
                seconds=self.safety_monitor.detection_active_reset_time + 1
            )
        )
        self.safety_monitor.vehicle_state = "slow"
        detection = self.create_fused_detection(5)  # Inside safety_distance_2
        self.safety_monitor.control_speed_state(detection)
        self.assertEqual(
            self.safety_monitor.vehicle_state,
            "slow",
            msg="Vehicle state should be 'slow'\
                         when detection is inside safety_distance_2",
        )
        self.safety_monitor.state_control()
        self.assertEqual(
            self.safety_monitor.vehicle_state,
            "slow",
            msg="Vehicle state should not change\
                         when active_detection_reset_time has not passed",
        )
        # Simulate waiting for active_detection_reset_time
        self.safety_monitor.last_detection_time_2 = (
            self.safety_monitor.get_clock().now()
            - rclpy.duration.Duration(
                seconds=self.safety_monitor.detection_active_reset_time + 1
            )
        )
        self.safety_monitor.state_control()
        self.assertEqual(
            self.safety_monitor.vehicle_state,
            "moderate",
            msg="Vehicle state should be 'moderate'\
                         when active_detection_reset_time has passed \
                         and the state was 'slow'",
        )

        # Test that the vehicle state transitions from 'stopped'...

        # ... to 'slow', ONLY when stopped_reset_time has passed
        self.safety_monitor.stop_time = (
            self.safety_monitor.stop_time
            - rclpy.duration.Duration(
                seconds=self.safety_monitor.vehicle_stopped_reset_time + 1
            )
        )
        detection = self.create_fused_detection(1)
        self.safety_monitor.control_speed_state(detection)
        self.assertEqual(
            self.safety_monitor.vehicle_state,
            "stopped",
            msg="Vehicle state should be 'stopped'\
                         when detection is inside stop_distance",
        )
        self.safety_monitor.state_control()
        self.assertEqual(
            self.safety_monitor.vehicle_state,
            "stopped",
            msg="Vehicle state should not change\
                         when active_detection_reset_time has not passed",
        )
        # Simulate waiting for vehicle_stopped_reset_time
        self.safety_monitor.stop_time = (
            self.safety_monitor.get_clock().now()
            - rclpy.duration.Duration(
                seconds=self.safety_monitor.vehicle_stopped_reset_time + 1
            )
        )
        self.safety_monitor.state_control()
        self.assertEqual(
            self.safety_monitor.vehicle_state,
            "slow",
            msg="Vehicle state should be 'slow'\
                         when vehicle_stopped_reset_time has passed \
                         and the state was 'stopped",
        )

        # Test that the vehicle state doesn't change...

        # ... when the detection is outside the safety_distance_1 parameter
        self.safety_monitor.vehicle_state = "agopen"
        detection = self.create_fused_detection(50)
        self.safety_monitor.control_speed_state(detection)
        self.assertEqual(
            self.safety_monitor.vehicle_state,
            "agopen",
            msg="Vehicle state should not change\
                         when detection is outside safety_distance_1",
        )

        # ... when state is slow and detection is outside safety_distance_2
        self.safety_monitor.vehicle_state = "slow"
        detection = self.create_fused_detection(12)
        self.safety_monitor.control_speed_state(detection)
        self.assertEqual(
            self.safety_monitor.vehicle_state,
            "slow",
            msg="Vehicle state should not change\
                         when detection is outside safety_distance_2",
        )

        # ... on detections when state is stopped
        self.safety_monitor.vehicle_state = "stopped"
        detection = self.create_fused_detection(5)
        self.safety_monitor.control_speed_state(detection)
        self.assertEqual(
            self.safety_monitor.vehicle_state,
            "stopped",
            msg="Vehicle state should not change\
                         from detections when the state is 'stopped'",
        )

        # Test that the vehicle state transitions to 'stopped',
        # if the state is unknown
        self.safety_monitor.vehicle_state = "anything_else"
        self.safety_monitor.state_control()
        self.assertEqual(
            self.safety_monitor.vehicle_state,
            "stopped",
            msg="Vehicle state should be 'stopped'\
                         when the state is unknown",
        )


if __name__ == "__main__":
    unittest.main()
