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
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launch file for the perception stack (sensors + fusion).

    This launch file brings up:
    - Camera interface node
    - Radar interface node
    - Sensor fusion node
    - Kalman filter tracker
    - TF transforms for sensors

    Usage:
        # Launch perception stack with default parameters
        # (Assumes camera driver is already running separately)
        ros2 launch tractor_safety_system_launch perception_stack.launch.py

        # Launch with OAK-D S2 camera driver included
        ros2 launch tractor_safety_system_launch perception_stack.launch.py \
            start_camera_driver:=true

        # Disable specific sensors
        ros2 launch tractor_safety_system_launch perception_stack.launch.py \
            start_camera:=false

        # Disable tracker
        ros2 launch tractor_safety_system_launch perception_stack.launch.py \
            start_tracker:=false
    """
    # Toggle nodes
    start_tracker = LaunchConfiguration("start_tracker")
    start_radar = LaunchConfiguration("start_radar")
    start_camera = LaunchConfiguration("start_camera")
    start_camera_driver = LaunchConfiguration("start_camera_driver")
    publish_tf = LaunchConfiguration("publish_tf")

    can_channel = LaunchConfiguration("can_channel")

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

    # Parameters file
    sim_pkg_share = get_package_share_path("simulations")
    default_params = os.path.join(sim_pkg_share, "config", "parameters.yaml")

    params_arg = DeclareLaunchArgument(
        "params",
        default_value=default_params,
        description="Path to parameters YAML file",
    )
    params = LaunchConfiguration("params")

    # --- Camera Driver (optional) ---
    # Launch OAK-D S2 camera driver if requested
    # This only launches the hardware driver - camera_node is launched separately below
    camera_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("camera_interface"), "/launch", "/oak_d_s2.launch.py"]
        ),
        condition=IfCondition(start_camera_driver),
        launch_arguments={
            # Add any camera driver arguments here if needed
            # e.g., 'params_file': '/path/to/nn/config.yaml'
        }.items(),
    )

    # --- Sensor Interface Nodes ---
    radar_node = Node(
        package="radar_interface",
        executable="radar_node",
        name="radar_node",
        output="screen",
        parameters=[params, {"can_channel": can_channel}],
        condition=IfCondition(start_radar),
    )

    # Camera node - always launched if start_camera=true
    # This is the application layer that converts SpatialDetectionArray to CameraDetection
    # It can work with the camera driver launched above, or with an external camera driver
    camera_node = Node(
        package="camera_interface",
        executable="camera_node",
        name="camera_node",
        output="screen",
        parameters=[params],
        condition=IfCondition(start_camera),
    )

    # --- Fusion and Tracking Nodes ---
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

    # --- TF Transforms ---
    tf_cam = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_camera",
        arguments=[
            cam_x,
            cam_y,
            cam_z,
            cam_roll,
            cam_pitch,
            cam_yaw,
            "base_link",
            "camera_link",
        ],
        condition=IfCondition(publish_tf),
        output="screen",
    )

    tf_radar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_radar",
        arguments=[
            rad_x,
            rad_y,
            rad_z,
            rad_roll,
            rad_pitch,
            rad_yaw,
            "base_link",
            "radar_link",
        ],
        condition=IfCondition(publish_tf),
        output="screen",
    )

    return LaunchDescription(
        [
            params_arg,
            # Launch args
            DeclareLaunchArgument(
                "can_channel",
                default_value="can0",
                description="SocketCAN channel for radar (e.g., can0, vcan0)",
            ),
            DeclareLaunchArgument(
                "start_tracker",
                default_value="true",
                description="Start the Kalman filter tracker node",
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
                "start_camera_driver",
                default_value="false",
                description="Start the OAK-D S2 camera driver (depthai-ros-driver). "
                "If false, assumes camera driver is already running.",
            ),
            DeclareLaunchArgument(
                "publish_tf",
                default_value="true",
                description="Publish static TF transforms for sensors",
            ),
            # Camera TF (base_link -> camera_link)
            # Default values convert camera axes to match real axis orientation:
            # roll = -1.5708 rad  (-90 deg), yaw = -1.5708 rad (-90 deg), pitch = 0
            # Position defaults remain at origin and can be overridden at launch.
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
                default_value="-1.5708",
                description="Camera roll angle (radians)",
            ),
            DeclareLaunchArgument(
                "camera_tf_pitch",
                default_value="0.0",
                description="Camera pitch angle (radians)",
            ),
            DeclareLaunchArgument(
                "camera_tf_yaw",
                default_value="-1.5708",
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
            # Camera driver (optional - launches depthai-ros-driver only)
            # camera_node is launched separately below as part of the perception stack
            camera_driver_launch,
            # Sensor interface nodes
            radar_node,
            camera_node,
            fusion_node,
            tracker_node,
            tf_cam,
            tf_radar,
        ]
    )
