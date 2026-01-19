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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for the complete tractor safety system core components.

    This launch file brings up the entire safety system including:
    - Perception stack (sensors, fusion, tracking) - included via perception_stack.launch.py
    - Safety monitoring
    - Control interface
    - TF transforms

    The perception stack includes:
    - Camera interface node (optionally with camera driver)
    - Radar interface node
    - Sensor fusion node
    - Kalman filter tracker

    Usage:
        # Launch with default parameters
        ros2 launch tractor_safety_system_launch safety_system_core.launch.py

        # Launch with camera driver included
        ros2 launch tractor_safety_system_launch safety_system_core.launch.py \
            start_camera_driver:=true

        # Disable specific components
        ros2 launch tractor_safety_system_launch safety_system_core.launch.py \
            start_camera:=false start_radar:=false start_tracker:=false

        # Custom parameter file
        ros2 launch tractor_safety_system_launch safety_system_core.launch.py \
            params:=/path/to/custom/parameters.yaml

        # Custom TF transforms
        ros2 launch tractor_safety_system_launch safety_system_core.launch.py \
            camera_tf_x:=0.5 camera_tf_z:=0.9
    """
    # Toggle nodes
    start_tracker = LaunchConfiguration("start_tracker")
    start_safety_monitor = LaunchConfiguration("start_safety_monitor")
    start_radar = LaunchConfiguration("start_radar")
    start_camera = LaunchConfiguration("start_camera")
    start_control = LaunchConfiguration("start_control")
    publish_tf = LaunchConfiguration("publish_tf")

    # TF values (base_link -> camera_link)
    cam_x = LaunchConfiguration("camera_tf_x")
    cam_y = LaunchConfiguration("camera_tf_y")
    cam_z = LaunchConfiguration("camera_tf_z")
    cam_yaw = LaunchConfiguration("camera_tf_yaw")
    cam_pitch = LaunchConfiguration("camera_tf_pitch")
    cam_roll = LaunchConfiguration("camera_tf_roll")

    # TF values (base_link -> radar_link)
    rad_x = LaunchConfiguration("radar_tf_x")
    rad_y = LaunchConfiguration("radar_tf_y")
    rad_z = LaunchConfiguration("radar_tf_z")
    rad_yaw = LaunchConfiguration("radar_tf_yaw")
    rad_pitch = LaunchConfiguration("radar_tf_pitch")
    rad_roll = LaunchConfiguration("radar_tf_roll")

    # A shared parameters file for nodes that use it
    # Default to simulations package config, but allow override
    sim_pkg_share = get_package_share_path("simulations")
    default_params = os.path.join(sim_pkg_share, "config", "parameters.yaml")

    # Allow overriding from CLI: params:=/abs/path/to/custom.yaml
    params_arg = DeclareLaunchArgument(
        "params",
        default_value=default_params,
        description="Path to parameters YAML file",
    )
    params = LaunchConfiguration("params")

    # Include perception stack (camera, radar, fusion, tracker)
    # This avoids duplication and keeps launch files modular
    launch_pkg_share = get_package_share_path("tractor_safety_system_launch")
    perception_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_pkg_share, "launch", "perception_stack.launch.py")
        ),
        launch_arguments={
            "params": params,
            "start_tracker": start_tracker,
            "start_radar": start_radar,
            "start_camera": start_camera,
            "start_camera_driver": LaunchConfiguration("start_camera_driver"),
            "publish_tf": publish_tf,
            "camera_tf_x": cam_x,
            "camera_tf_y": cam_y,
            "camera_tf_z": cam_z,
            "camera_tf_roll": cam_roll,
            "camera_tf_pitch": cam_pitch,
            "camera_tf_yaw": cam_yaw,
            "radar_tf_x": rad_x,
            "radar_tf_y": rad_y,
            "radar_tf_z": rad_z,
            "radar_tf_roll": rad_roll,
            "radar_tf_pitch": rad_pitch,
            "radar_tf_yaw": rad_yaw,
        }.items(),
    )

    # --- Safety and Control nodes ---
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
                "start_radar",
                default_value="true",
                description="Start the radar interface node",
            ),
            DeclareLaunchArgument(
                "start_camera",
                default_value="true",
                description="Start the camera interface node",
            ),
            DeclareLaunchArgument(
                "start_control",
                default_value="true",
                description="Start the control interface node",
            ),
            DeclareLaunchArgument(
                "publish_tf",
                default_value="true",
                description="Publish static TF transforms for sensors",
            ),
            DeclareLaunchArgument(
                "start_camera_driver",
                default_value="false",
                description="Start the OAK-D S2 camera driver (depthai-ros-driver). "
                "If false, assumes camera driver is already running.",
            ),
            # Camera TF (base_link -> camera_link)
            DeclareLaunchArgument(
                "camera_tf_x",
                default_value="0.0",
                description="Camera X offset from base_link (meters)",
            ),
            DeclareLaunchArgument(
                "camera_tf_y",
                default_value="0.0",
                description="Camera Y offset from base_link (meters)",
            ),
            DeclareLaunchArgument(
                "camera_tf_z",
                default_value="0.0",
                description="Camera Z offset from base_link (meters)",
            ),
            DeclareLaunchArgument(
                "camera_tf_roll",
                default_value="0.0",
                description="Camera roll angle (radians)",
            ),
            DeclareLaunchArgument(
                "camera_tf_pitch",
                default_value="0.0",
                description="Camera pitch angle (radians)",
            ),
            DeclareLaunchArgument(
                "camera_tf_yaw",
                default_value="0.0",
                description="Camera yaw angle (radians)",
            ),
            # Radar TF (base_link -> radar_link)
            DeclareLaunchArgument(
                "radar_tf_x",
                default_value="0.0",
                description="Radar X offset from base_link (meters)",
            ),
            DeclareLaunchArgument(
                "radar_tf_y",
                default_value="0.0",
                description="Radar Y offset from base_link (meters)",
            ),
            DeclareLaunchArgument(
                "radar_tf_z",
                default_value="0.0",
                description="Radar Z offset from base_link (meters)",
            ),
            DeclareLaunchArgument(
                "radar_tf_roll",
                default_value="0.0",
                description="Radar roll angle (radians)",
            ),
            DeclareLaunchArgument(
                "radar_tf_pitch",
                default_value="0.0",
                description="Radar pitch angle (radians)",
            ),
            DeclareLaunchArgument(
                "radar_tf_yaw",
                default_value="0.0",
                description="Radar yaw angle (radians)",
            ),
            # Include perception stack (includes camera, radar, fusion, tracker, TF)
            perception_stack,
            # Safety and control nodes
            safety_monitor_node,
            control_node,
        ]
    )
