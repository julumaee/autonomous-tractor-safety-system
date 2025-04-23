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
# updates its parameters and maintains expected behavior.                                    #
#                                                                                            #
# Classes:                                                                                   #
#    TestSafetyMonitorParameters: A unittest.TestCase subclass that contains tests for the   #
#    parameter update functionality of the SafetyMonitor class.                              #
#                                                                                            #
# Methods:                                                                                   #
#    setUpClass(cls): Initializes ROS2 before any test is run.                               #
#    tearDownClass(cls): Shuts down ROS2 after all tests are completed.                      #
#    setUp(self): Initializes a SafetyMonitor instance for testing.                          #
#    test_parameter_update(self): Ensures that parameters can be updated correctly and       #
#    verifies the updated values.                                                            #
#                                                                                            #
##############################################################################################

import unittest

import rclpy
from rclpy.parameter import Parameter
from safety_monitor.safety_monitor import SafetyMonitor


class TestSafetyMonitorParameters(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 before any test is run."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS2 after all tests are completed."""
        rclpy.shutdown()

    def setUp(self):
        """Initialize the SafetyMonitor for testing."""
        self.safety_monitor = SafetyMonitor()

    def test_parameter_update(self):
        """Test that parameters can be updated correctly."""
        new_params = [
            Parameter(name='safety_distance_1', value=50),
            Parameter(name='safety_distance_2', value=20),
            Parameter(name='stop_distance', value=4),
            Parameter(name='speed_override_1', value=6),
            Parameter(name='speed_override_2', value=3),
            Parameter(name='detection_active_reset_time', value=6.0),
            Parameter(name='vehicle_stopped_reset_time', value=6.0)
        ]
        self.safety_monitor.on_set_parameters(new_params)

        # Check that the parameters are updated correctly
        self.assertEqual(self.safety_monitor.safety_distance_1, 50,
                         msg='safety_distance_1 parameter did not update correctly')
        self.assertEqual(self.safety_monitor.safety_distance_2, 20,
                         msg='safety_distance_2 parameter did not update correctly')
        self.assertEqual(self.safety_monitor.stop_distance, 4,
                         msg='stop_distance parameter did not update correctly')
        self.assertEqual(self.safety_monitor.speed_override_1, 6,
                         msg='speed_override_1 parameter did not update correctly')
        self.assertEqual(self.safety_monitor.speed_override_2, 3,
                         msg='speed_override_2 parameter did not update correctly')
        self.assertEqual(self.safety_monitor.detection_active_reset_time, 6.0,
                         msg='detection_active_reset_time parameter did not update correctly')
        self.assertEqual(self.safety_monitor.vehicle_stopped_reset_time, 6.0,
                         msg='vehicle_stopped_reset_time parameter did not update correctly')


if __name__ == '__main__':
    unittest.main()
