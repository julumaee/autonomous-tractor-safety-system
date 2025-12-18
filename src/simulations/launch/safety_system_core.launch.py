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
    # Toggle nodes
    start_tracker = LaunchConfiguration('start_tracker')
    start_safety_monitor = LaunchConfiguration('start_safety_monitor')
    start_radar = LaunchConfiguration('start_radar')
    start_camera = LaunchConfiguration('start_camera')
    start_control = LaunchConfiguration('start_control')
    publish_tf = LaunchConfiguration('publish_tf')

    # TF values (base_link -> camera_link
    cam_x = LaunchConfiguration('camera_tf_x')
    cam_y = LaunchConfiguration('camera_tf_y')
    cam_z = LaunchConfiguration('camera_tf_z')
    cam_yaw = LaunchConfiguration('camera_tf_yaw')
    cam_pitch = LaunchConfiguration('camera_tf_pitch')
    cam_roll = LaunchConfiguration('camera_tf_roll')

    # TF values (base_link -> radar_link)
    rad_x = LaunchConfiguration('radar_tf_x')
    rad_y = LaunchConfiguration('radar_tf_y')
    rad_z = LaunchConfiguration('radar_tf_z')
    rad_yaw = LaunchConfiguration('radar_tf_yaw')
    rad_pitch = LaunchConfiguration('radar_tf_pitch')
    rad_roll = LaunchConfiguration('radar_tf_roll')

    # A shared parameters file for nodes that use it
    # Resolve default parameter file inside the installed package
    pkg_share = get_package_share_path('simulations')
    default_params = os.path.join(pkg_share, 'config', 'parameters.yaml')

    # Allow overriding from CLI: params:=/abs/path/to/custom.yaml
    params_arg = DeclareLaunchArgument(
        'params',
        default_value=default_params,
        description='Path to parameters YAML for safety_monitor'
    )
    params = LaunchConfiguration('params')

    # --- Core nodes ---
    fusion_node = Node(
        package='sensor_fusion',
        executable='fusion_node',
        name='fusion_node',
        output='screen',
        parameters=[params],
    )

    tracker_node = Node(
        package='sensor_fusion',
        executable='kf_tracker',
        name='kf_tracker',
        output='screen',
        parameters=[params],
        condition=IfCondition(start_tracker),
    )

    safety_monitor_node = Node(
        package='safety_monitor',
        executable='safety_monitor',
        name='safety_monitor',
        output='screen',
        parameters=[params],
        condition=IfCondition(start_safety_monitor),
    )

    # --- Optional I/O nodes ---
    radar_node = Node(
        package='radar_interface',
        executable='radar_node',
        name='radar_node',
        output='screen',
        parameters=[params],
        condition=IfCondition(start_radar),
    )

    camera_node = Node(
        package='camera_interface',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[params],
        condition=IfCondition(start_camera),
    )

    control_node = Node(
        package='tractor_control',
        executable='control_node',
        name='control_node',
        output='screen',
        parameters=[params],
        condition=IfCondition(start_control),
    )

    # -----------------------------------------------------------
    # TF: base_link -> camera_link, base_link -> radar_link
    # Match the TFs to the actual sensor positions using launch args!
    # -----------------------------------------------------------
    # base_link -> camera_link
    tf_cam = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera',
        arguments=[
            cam_x, cam_y, cam_z,
            cam_roll, cam_pitch, cam_yaw,
            'base_link',
            'camera_link',
        ],
        condition=IfCondition(publish_tf),
        output='screen'
    )

    # base_link -> radar_link
    tf_radar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_radar',
        arguments=[
            rad_x, rad_y, rad_z,
            rad_roll, rad_pitch, rad_yaw,
            'base_link',
            'radar_link',
        ],
        condition=IfCondition(publish_tf),
        output='screen'
    )
    return LaunchDescription([
        params_arg,
        # Launch args
        DeclareLaunchArgument('start_tracker', default_value='true'),
        DeclareLaunchArgument('start_safety_monitor', default_value='true'),
        DeclareLaunchArgument('start_radar', default_value='true'),
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('start_control', default_value='true'),
        DeclareLaunchArgument('publish_tf', default_value='true'),

        # Shared params (set a sensible default path in your repo)
        DeclareLaunchArgument(
            'params_file',
            default_value='',
            description='YAML file passed as parameters to nodes (optional).',
        ),

        # Camera TF (base_link -> camera_link)
        DeclareLaunchArgument('camera_tf_x', default_value='0.0'),
        DeclareLaunchArgument('camera_tf_y', default_value='0.0'),
        DeclareLaunchArgument('camera_tf_z', default_value='0.0'),
        DeclareLaunchArgument('camera_tf_roll', default_value='0.0'),
        DeclareLaunchArgument('camera_tf_pitch', default_value='0.0'),
        DeclareLaunchArgument('camera_tf_yaw', default_value='0.0'),

        # Radar TF (base_link -> radar_link)
        DeclareLaunchArgument('radar_tf_x', default_value='0.0'),
        DeclareLaunchArgument('radar_tf_y', default_value='0.0'),
        DeclareLaunchArgument('radar_tf_z', default_value='0.0'),
        DeclareLaunchArgument('radar_tf_roll', default_value='0.0'),
        DeclareLaunchArgument('radar_tf_pitch', default_value='0.0'),
        DeclareLaunchArgument('radar_tf_yaw', default_value='0.0'),

        # Nodes
        fusion_node,
        tracker_node,
        safety_monitor_node,
        radar_node,
        camera_node,
        control_node,
        tf_cam,
        tf_radar,
    ])
