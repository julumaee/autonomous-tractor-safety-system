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

import time
import unittest

import launch_testing
import pytest
import rclpy
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

from tractor_safety_system_interfaces.msg import ControlCommand, FusedDetection


@pytest.mark.rostest
def generate_test_description():
    """Launch all necessary nodes for the integration test."""
    object_simulator = Node(
        package="simulations",
        executable="object_simulator",
        name="object_simulator",
        parameters=[
            {
                "min_x": 5.0,
                "max_x": 10.0,
                "min_y": -5.0,
                "max_y": 5.0,
            }
        ],
        output="screen",
    )
    radar_simulator_can = Node(
        package="simulations",
        executable="radar_simulator_can",
        name="radar_simulator_can",
        output="screen",
    )
    camera_simulator = Node(
        package="simulations",
        executable="camera_simulator",
        name="camera_simulator",
        output="screen",
    )
    agopen_simulator = Node(
        package="simulations",
        executable="agopen_simulator",
        name="agopen_simulator",
        output="screen",
    )
    camera_node = Node(
        package="camera_interface",
        executable="camera_node",
        name="camera_node",
        output="screen",
    )
    radar_node = Node(
        package="radar_interface",
        executable="radar_node",
        name="radar_node",
        parameters=[{"can_channel": "vcan0"}],
        output="screen",
    )
    fusion_node = Node(
        package="sensor_fusion",
        executable="fusion_node",
        name="fusion_node",
        output="screen",
    )
    safety_monitor = Node(
        package="safety_monitor",
        executable="safety_monitor",
        name="safety_monitor",
        output="screen",
    )

    # --------------------
    # Static TFs (match your SDF poses)
    # --------------------
    # base_link -> camera optical (0, 0, 0) + optical rotation RPY(-90, 0, -90)
    tf_cam = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_camera_optical",
        arguments=[
            "0",
            "0",
            "0",
            "-1.5708",
            "0",
            "-1.5708",
            "base_link",
            "camera_link",
        ],
        output="screen",
    )

    # base_link -> radar (0, 0, 0)
    tf_radar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_radar",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "radar_link"],
        output="screen",
    )

    return LaunchDescription(
        [
            object_simulator,
            radar_simulator_can,
            camera_simulator,
            agopen_simulator,
            camera_node,
            radar_node,
            fusion_node,
            safety_monitor,
            tf_cam,
            tf_radar,
            TimerAction(period=5.0, actions=[launch_testing.actions.ReadyToTest()]),
        ]
    ), {
        "test": TestIntegration,
    }


class TestIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 before running tests."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS2 after tests complete."""
        rclpy.shutdown()

    def setUp(self):
        """Set up subscribers."""
        self.node = rclpy.create_node("test_integration_node")

        # Store received messages
        self.received_fused_detections = []
        self.received_control_cmds = []

        self.subscription_fused = self.node.create_subscription(
            FusedDetection, "/fused_detections", self.fusion_callback, 10
        )

        self.subscription_control = self.node.create_subscription(
            ControlCommand, "/control", self.control_callback, 10
        )

    def fusion_callback(self, msg):
        """Capture fused detection messages."""
        self.received_fused_detections.append(msg)
        print("Received FusedDetection:", msg)

    def control_callback(self, msg):
        """Capture final control command message from safety_monitor."""
        self.received_control_cmds.append(msg)

    def test_system_flow(self):
        """Test the complete system flow."""
        # Wait for messages to be received
        timeout = 5
        start_time = time.time()
        while time.time() - start_time < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            # Break before timeout if detections are received
            if (
                len(self.received_fused_detections) > 0
                and len(self.received_control_cmds) > 0
            ):
                break

        # Ensure at least one fused detection was received
        self.assertGreater(
            len(self.received_fused_detections), 0, msg="No fused detections received."
        )

        # Ensure at least one control command was received
        self.assertGreater(
            len(self.received_control_cmds), 0, msg="No control commands received."
        )
