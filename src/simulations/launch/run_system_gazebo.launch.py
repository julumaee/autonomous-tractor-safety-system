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

import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Resolve default parameter file inside the installed package
    pkg_share = get_package_share_path("simulations")
    default_params = os.path.join(pkg_share, "config", "parameters_simulated.yaml")

    # Allow overriding from CLI
    params_arg = DeclareLaunchArgument(
        "params",
        default_value=default_params,
        description="Path to parameters YAML for safety_monitor",
    )

    # Option to start safety monitor
    start_safety_monitor_arg = DeclareLaunchArgument(
        "start_safety_monitor",
        default_value="true",
        description="Whether to start the safety monitor node",
    )

    params = LaunchConfiguration("params")
    start_safety_monitor = LaunchConfiguration("start_safety_monitor")

    # List of all nodes to launch
    nodes = [
        # Camera Node (Processes real or simulated camera detections)
        Node(
            package="camera_interface",
            executable="camera_node",
            name="camera_node",
            parameters=[params],
            output="screen",
        ),
        # Sensor Fusion Node (Combines camera & radar data)
        Node(
            package="sensor_fusion",
            executable="fusion_node",
            name="fusion_node",
            parameters=[params],
            output="screen",
        ),
        # Tracker node (Tracks fused detections over time)
        Node(
            package="sensor_fusion",
            executable="kf_tracker",
            name="kf_tracker",
            parameters=[params],
            output="screen",
        ),
        # Safety Monitor (Monitors fused detections and controls vehicle state)
        Node(
            package="safety_monitor",
            executable="safety_monitor",
            name="safety_monitor",
            parameters=[params],
            condition=IfCondition(start_safety_monitor),
            output="screen",
        ),
    ]

    # Create shutdown handlers for all nodes
    shutdown_handlers = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=node,
                on_exit=[Shutdown()],  # Shut down all nodes when any one exits
            )
        )
        for node in nodes
    ]

    return LaunchDescription(
        [
            params_arg,
            start_safety_monitor_arg,
            LogInfo(msg="Launching all nodes..."),
            *nodes,
            *shutdown_handlers,
            LogInfo(msg="All nodes launched successfully."),
        ]
    )
