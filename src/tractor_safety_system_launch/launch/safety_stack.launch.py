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

from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for the safety and control stack.

    This launch file brings up:
    - Sensor fusion node (needed for safety monitor input)
    - Kalman filter tracker (optional)
    - Safety monitor node
    - Control interface node (optional)

    Assumes that sensor interfaces (camera/radar) are running separately
    or have already been launched.

    Usage:
        # Launch safety stack with default parameters
        ros2 launch tractor_safety_system_launch safety_stack.launch.py

        # Disable tracker or control
        ros2 launch tractor_safety_system_launch safety_stack.launch.py \
            start_tracker:=false start_control:=false
    """
    # Toggle nodes
    start_tracker = LaunchConfiguration("start_tracker")
    start_safety_monitor = LaunchConfiguration("start_safety_monitor")
    start_control = LaunchConfiguration("start_control")

    # Parameters file
    sim_pkg_share = get_package_share_path("simulations")
    default_params = os.path.join(sim_pkg_share, "config", "parameters.yaml")

    params_arg = DeclareLaunchArgument(
        "params",
        default_value=default_params,
        description="Path to parameters YAML file",
    )
    params = LaunchConfiguration("params")

    # --- Fusion and Tracking Nodes (needed for safety monitor) ---
    fusion_node = Node(
        package="sensor_fusion",
        executable="fusion_node",
        name="fusion_node",
        output="screen",
        parameters=[params],
    )

    tracker_node = Node(
        package="sensor_fusion",
        executable="kf_tracker",
        name="kf_tracker",
        output="screen",
        parameters=[params],
        condition=IfCondition(start_tracker),
    )

    # --- Safety and Control Nodes ---
    safety_monitor_node = Node(
        package="safety_monitor",
        executable="safety_monitor",
        name="safety_monitor",
        output="screen",
        parameters=[params],
        condition=IfCondition(start_safety_monitor),
    )

    control_node = Node(
        package="tractor_control",
        executable="control_node",
        name="control_node",
        output="screen",
        parameters=[params],
        condition=IfCondition(start_control),
    )

    return LaunchDescription(
        [
            params_arg,
            # Launch args
            DeclareLaunchArgument(
                "start_tracker",
                default_value="true",
                description="Start the Kalman filter tracker node",
            ),
            DeclareLaunchArgument(
                "start_safety_monitor",
                default_value="true",
                description="Start the safety monitor node",
            ),
            DeclareLaunchArgument(
                "start_control",
                default_value="true",
                description="Start the control interface node",
            ),
            # Nodes
            fusion_node,
            tracker_node,
            safety_monitor_node,
            control_node,
        ]
    )
