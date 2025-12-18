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
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    Shutdown,
    TimerAction
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_path('simulations')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time from /clock'
    )

    scenario_arg = DeclareLaunchArgument(
        'scenario',
        default_value='S1',
        description='Scenario name (e.g. S1, S2, S3, S4)'
    )

    params_arg = DeclareLaunchArgument(
        'params',
        default_value=os.path.join(pkg_share, 'config', 'parameters_simulated.yaml'),
        description='Path to parameters YAML'
    )

    bring_up_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'bring_up_sim.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'scenario': LaunchConfiguration('scenario'),
            'start_teleop_bridges': 'false',
            'start_rviz_bridge': 'false',
        }.items()
    )

    run_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'safety_system_core.launch.py')
        ),
        launch_arguments={
            'params': LaunchConfiguration('params'),
            'start_safety_monitor': 'false',
            'start_radar': 'false',
            'start_control': 'false',
            'camera_tf_x': '0.5',
            'camera_tf_y': '0.0',
            'camera_tf_z': '0.9',
            'camera_tf_roll': '-1.5708',  # -90 degrees in radians
            'camera_tf_pitch': '0.0',
            'camera_tf_yaw': '-1.5708',  # -90 degrees in radians
            'radar_tf_x': '0.6',
            'radar_tf_y': '0.0',
            'radar_tf_z': '0.6',
        }.items()
    )

    scripted_driver_node = Node(
        package='simulations',
        executable='scripted_driver',
        name='scripted_driver',
        output='screen',
        parameters=[{
            'scenario_name': LaunchConfiguration('scenario'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    detection_logger_node = Node(
        package='simulations',
        executable='detection_logger',
        name='detection_logger_node',
        output='screen',
        parameters=[{
            'scenario_name': LaunchConfiguration('scenario'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    detection_logger = TimerAction(
        period=5.0,
        actions=[detection_logger_node]
    )

    scripted_driver = TimerAction(
        period=5.0,
        actions=[scripted_driver_node]
    )

    # When the driver exits, shut down the whole launch system
    driver_shutdown = RegisterEventHandler(
        OnProcessExit(
            target_action=scripted_driver_node,
            on_exit=[
                LogInfo(msg='Scripted driver finished. Shutting down experiment.'),
                Shutdown()
            ]
        )
    )

    return LaunchDescription([
        use_sim_time_arg,
        scenario_arg,
        params_arg,

        LogInfo(msg=['Launching full experiment for scenario ', LaunchConfiguration('scenario')]),

        bring_up_sim,
        run_system,
        scripted_driver,
        detection_logger,
        driver_shutdown,
    ])
