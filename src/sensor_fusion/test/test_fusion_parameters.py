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
# handles parameter updates and maintains consistency in its internal state.                 #
#                                                                                            #
# Classes:                                                                                   #
#    TestFusionParameters: A unittest.TestCase subclass that contains tests for the          #
#    parameter handling of the FusionNode class.                                             #
#                                                                                            #
# Methods:                                                                                   #
#    setUpClass(cls): Initializes ROS2 before any test is run.                               #
#    tearDownClass(cls): Shuts down ROS2 after all tests are completed.                      #
#    setUp(self): Initializes a FusionNode instance for testing.                             #
#    test_parameter_update(self): Ensures that parameters can be updated correctly and       #
#    verifies the internal state of the FusionNode.                                          #
#                                                                                            #
##############################################################################################

import unittest

import numpy as np
import rclpy
from rclpy.parameter import Parameter
from sensor_fusion.fusion_node import FusionNode


class TestFusionParameters(unittest.TestCase):

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

    def test_parameter_update(self):
        """Test that parameters can be updated correctly."""
        new_params = [
            Parameter(name='time_threshold', value=1.0),
            Parameter(name='camera_trust_max', value=15.0),
            Parameter(name='rotation_matrix',
                      value=[0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]),
            Parameter(name='translation_vector', value=[1.0, 2.0, 3.0])
        ]
        self.fusion_node.on_set_parameters(new_params)

        # Check that the parameters are updated correctly
        self.assertEqual(self.fusion_node.time_threshold, 1.0)
        self.assertEqual(self.fusion_node.camera_trust_max, 15.0)
        np.testing.assert_array_equal(self.fusion_node.R, np.array([[0.0, -1.0, 0.0],
                                                                    [1.0, 0.0, 0.0],
                                                                    [0.0, 0.0, 1.0]]))
        np.testing.assert_array_equal(self.fusion_node.T, np.array([1.0, 2.0, 3.0]))


if __name__ == '__main__':
    unittest.main()
