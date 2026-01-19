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

"""
DEPRECATED: This launch file has been moved to tractor_safety_system_launch package.

This file is kept for backwards compatibility and forwards to the new location.
Please update your launch files to use:
    ros2 launch tractor_safety_system_launch safety_system_core.launch.py

This file will be removed in a future release.
"""

import os

from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Forward to tractor_safety_system_launch package.

    This file is kept for backwards compatibility.
    Use instead: ros2 launch tractor_safety_system_launch safety_system_core.launch.py
    """
    # Forward to the new location
    launch_pkg_share = get_package_share_path("tractor_safety_system_launch")
    sim_pkg_share = get_package_share_path("simulations")
    default_params = os.path.join(sim_pkg_share, "config", "parameters.yaml")

    # Declare launch arguments for backwards compatibility
    # These will be forwarded to the new launch file
    return LaunchDescription(
        [
            LogInfo(
                msg="[DEPRECATED] safety_system_core.launch.py in "
                "simulations package is deprecated. "
                "Please use: ros2 launch tractor_safety_system_launch"
                "safety_system_core.launch.py"
            ),
            # Forward all potential launch arguments
            DeclareLaunchArgument("start_tracker", default_value="true"),
            DeclareLaunchArgument("start_safety_monitor", default_value="true"),
            DeclareLaunchArgument("start_radar", default_value="true"),
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("start_control", default_value="true"),
            DeclareLaunchArgument("publish_tf", default_value="true"),
            DeclareLaunchArgument("params", default_value=default_params),
            DeclareLaunchArgument("camera_tf_x", default_value="0.0"),
            DeclareLaunchArgument("camera_tf_y", default_value="0.0"),
            DeclareLaunchArgument("camera_tf_z", default_value="0.0"),
            DeclareLaunchArgument("camera_tf_roll", default_value="0.0"),
            DeclareLaunchArgument("camera_tf_pitch", default_value="0.0"),
            DeclareLaunchArgument("camera_tf_yaw", default_value="0.0"),
            DeclareLaunchArgument("radar_tf_x", default_value="0.0"),
            DeclareLaunchArgument("radar_tf_y", default_value="0.0"),
            DeclareLaunchArgument("radar_tf_z", default_value="0.0"),
            DeclareLaunchArgument("radar_tf_roll", default_value="0.0"),
            DeclareLaunchArgument("radar_tf_pitch", default_value="0.0"),
            DeclareLaunchArgument("radar_tf_yaw", default_value="0.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        launch_pkg_share, "launch", "safety_system_core.launch.py"
                    )
                ),
                # Forward all launch arguments
                launch_arguments={
                    "start_tracker": LaunchConfiguration("start_tracker"),
                    "start_safety_monitor": LaunchConfiguration("start_safety_monitor"),
                    "start_radar": LaunchConfiguration("start_radar"),
                    "start_camera": LaunchConfiguration("start_camera"),
                    "start_control": LaunchConfiguration("start_control"),
                    "publish_tf": LaunchConfiguration("publish_tf"),
                    "params": LaunchConfiguration("params"),
                    "camera_tf_x": LaunchConfiguration("camera_tf_x"),
                    "camera_tf_y": LaunchConfiguration("camera_tf_y"),
                    "camera_tf_z": LaunchConfiguration("camera_tf_z"),
                    "camera_tf_roll": LaunchConfiguration("camera_tf_roll"),
                    "camera_tf_pitch": LaunchConfiguration("camera_tf_pitch"),
                    "camera_tf_yaw": LaunchConfiguration("camera_tf_yaw"),
                    "radar_tf_x": LaunchConfiguration("radar_tf_x"),
                    "radar_tf_y": LaunchConfiguration("radar_tf_y"),
                    "radar_tf_z": LaunchConfiguration("radar_tf_z"),
                    "radar_tf_roll": LaunchConfiguration("radar_tf_roll"),
                    "radar_tf_pitch": LaunchConfiguration("radar_tf_pitch"),
                    "radar_tf_yaw": LaunchConfiguration("radar_tf_yaw"),
                }.items(),
            ),
        ]
    )
