# Copyright 2025 Eemil Kulmala, University of Oulu
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

from launch import LaunchDescription
from launch.actions import LogInfo, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    nodes = [
        Node(
            package='radar_interface',
            executable='radar_node',
            name='radar_node',
        ),
        Node(
            package='camera_interface',
            executable='camera_node',
            name='camera_node',
        ),
        Node(
            package='simulations',
            executable='calibration_logger',
            name='calibration_logger',
        ),
    ]

    # Create shutdown handlers for all nodes
    shutdown_handlers = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=node,
                on_exit=[Shutdown()]  # Shut down all nodes when any one exits
            )
        )
        for node in nodes
    ]

    return LaunchDescription([
        # Log message to indicate test start
        LogInfo(msg='Launching all nodes...'),
        *nodes,
        *shutdown_handlers,
        # Log message to indicate test setup completion
        LogInfo(msg='All nodes launched successfully.'),
    ])
