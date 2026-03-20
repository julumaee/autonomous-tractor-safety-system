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
Launch file for real-world tractor testing.

This launch file brings up:
- Perception stack (radar, camera, fusion, tracker)
- Real-world logger
- Ego motion publisher (needs integration with tractor sensors)
- (Optional) RTK GPS stack (u-blox + NTRIP)

Usage:
    # Basic test with default parameters
    ros2 launch tractor_safety_system_launch real_world_test.launch.py \
        test_name:=test_1

    # With custom sensor transforms (measure these on your tractor!)
    ros2 launch tractor_safety_system_launch real_world_test.launch.py \
        test_name:=pedestrian_approach \
        camera_tf_x:=1.2 \
        camera_tf_z:=0.8 \
        radar_tf_x:=1.5 \
        radar_tf_z:=0.5

    # Disable specific sensors if not available
    ros2 launch tractor_safety_system_launch real_world_test.launch.py \
        test_name:=radar_only \
        start_camera:=false
"""

import os

from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # A shared parameters file for nodes that use it.
    # Default to this package's config, but allow override.
    launch_pkg_share = get_package_share_path("tractor_safety_system_launch")
    default_params = os.path.join(launch_pkg_share, "config", "parameters.yaml")

    params_arg = DeclareLaunchArgument(
        "params",
        default_value=default_params,
        description="Path to parameters YAML file used by perception stack nodes",
    )
    params = LaunchConfiguration("params")

    start_ego_motion = LaunchConfiguration("start_ego_motion")
    test_name = LaunchConfiguration("test_name")
    can_channel = LaunchConfiguration("can_channel")

    # Perception stack toggles (passed through)
    start_tracker = LaunchConfiguration("start_tracker")
    start_radar = LaunchConfiguration("start_radar")
    start_camera = LaunchConfiguration("start_camera")
    start_camera_driver = LaunchConfiguration("start_camera_driver")
    publish_tf = LaunchConfiguration("publish_tf")

    use_measurement_covariance_models = LaunchConfiguration(
        "use_measurement_covariance_models"
    )
    test_name_arg = DeclareLaunchArgument(
        "test_name",
        default_value="test_1",
        description="Name/identifier for this test run",
    )

    # Include the perception stack
    # This brings up: radar_node, camera_node, fusion_node, tracker_node, TF transforms
    perception_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("tractor_safety_system_launch"),
                "/launch",
                "/perception_stack.launch.py",
            ]
        ),
        launch_arguments={
            "params": params,
            "start_camera_driver": start_camera_driver,
            "start_tracker": start_tracker,
            "start_radar": start_radar,
            "start_camera": start_camera,
            "publish_tf": publish_tf,
            "can_channel": can_channel,
            "use_measurement_covariance_models": use_measurement_covariance_models,
            # Pass through any TF overrides from command line
            # Users can specify these when launching this file
            "camera_tf_x": LaunchConfiguration("camera_tf_x", default="0.0"),
            "camera_tf_y": LaunchConfiguration("camera_tf_y", default="0.0"),
            "camera_tf_z": LaunchConfiguration("camera_tf_z", default="0.0"),
            "camera_tf_roll": LaunchConfiguration("camera_tf_roll", default="0.0"),
            "camera_tf_pitch": LaunchConfiguration("camera_tf_pitch", default="0.0"),
            "camera_tf_yaw": LaunchConfiguration("camera_tf_yaw", default="0.0"),
            "radar_tf_x": LaunchConfiguration("radar_tf_x", default="0.0"),
            "radar_tf_y": LaunchConfiguration("radar_tf_y", default="0.0"),
            "radar_tf_z": LaunchConfiguration("radar_tf_z", default="0.0"),
            "radar_tf_roll": LaunchConfiguration("radar_tf_roll", default="0.0"),
            "radar_tf_pitch": LaunchConfiguration("radar_tf_pitch", default="0.0"),
            "radar_tf_yaw": LaunchConfiguration("radar_tf_yaw", default="0.0"),
        }.items(),
    )

    # Real-world logger
    logger_node = Node(
        package="simulations",
        executable="real_world_logger",
        name="real_world_logger",
        output="screen",
        parameters=[{"test_name": test_name}],
    )

    # GPS-based ego motion publisher (FIELD TEST: straight line)
    # Uses ublox NavPVT for speed; yaw-rate forced to 0
    ego_motion_node = Node(
        package="simulations",
        executable="gps_ego_motion",
        name="gps_ego_motion",
        output="screen",
        condition=IfCondition(start_ego_motion),
        parameters=[
            {
                # Input from ublox_gps
                "navpvt_topic": LaunchConfiguration("navpvt_topic", default="/navpvt"),

                # Output used by your perception stack
                "publish_topic": LaunchConfiguration("ego_motion_topic", default="/ego_motion"),

                # Publish rate (steady, not event-driven)
                "publish_rate_hz": LaunchConfiguration("ego_motion_rate_hz", default="10.0"),

                # Simple gating / safety behavior
                "min_fix_type": LaunchConfiguration("ego_motion_min_fix_type", default="2"),
                "min_sv": LaunchConfiguration("ego_motion_min_sv", default="4"),
                "timeout_s": LaunchConfiguration("ego_motion_timeout_s", default="1.0"),
            }
        ],
    )

    # Optional: bring up RTK GPS stack (u-blox + NTRIP) as part of this launch.
    gps_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("tractor_safety_system_launch"),
                "/launch",
                "/gps_rtk_ublox_ntrip.launch.py",
            ]
        ),
        condition=IfCondition(LaunchConfiguration("start_gps", default="false")),
        launch_arguments={
            "gps_device": LaunchConfiguration("gps_device", default="/dev/ttyACM0"),
            "gps_baudrate": LaunchConfiguration("gps_baudrate", default="115200"),
            "gps_fix_topic": LaunchConfiguration("gps_fix_topic", default="/gps/fix"),
            "ublox_config_file": LaunchConfiguration(
                "ublox_config_file", default=""
            ),
            "rtcm_topic": LaunchConfiguration("rtcm_topic", default="/rtcm"),
            "start_ntrip": LaunchConfiguration("start_ntrip", default="true"),
            "enable_ntrip_fix_rate_limit": LaunchConfiguration(
                "enable_ntrip_fix_rate_limit", default="true"
            ),
            "ntrip_fix_topic": LaunchConfiguration("ntrip_fix_topic", default="/gps/fix_ntrip"),
            "ntrip_fix_rate_hz": LaunchConfiguration("ntrip_fix_rate_hz", default="1.0"),
            "ntrip_host": LaunchConfiguration("ntrip_host", default=""),
            "ntrip_port": LaunchConfiguration("ntrip_port", default="2101"),
            "ntrip_version": LaunchConfiguration("ntrip_version", default="Ntrip/2.0"),
            "ntrip_mountpoint": LaunchConfiguration("ntrip_mountpoint", default=""),
            "ntrip_authenticate": LaunchConfiguration("ntrip_authenticate", default="false"),
            "ntrip_username": LaunchConfiguration("ntrip_username", default=""),
            "ntrip_password": LaunchConfiguration("ntrip_password", default=""),
            "rtcm_message_package": LaunchConfiguration(
                "rtcm_message_package", default="rtcm_msgs"
            ),
            "ublox_package": LaunchConfiguration("ublox_package", default="ublox_gps"),
            "ublox_executable": LaunchConfiguration("ublox_executable", default="ublox_gps_node"),
            "ntrip_package": LaunchConfiguration(
                "ntrip_package", default="tractor_safety_system_launch"
            ),
            "ntrip_executable": LaunchConfiguration(
                "ntrip_executable", default="ntrip_tcp_client"
            ),
        }.items(),
    )

    return LaunchDescription(
        [
            test_name_arg,
            params_arg,
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
                default_value="true",
                description=(
                    "Start the OAK-D S2 camera driver (depthai-ros-driver). "
                    "If false, assumes camera driver is already running."
                ),
            ),
            DeclareLaunchArgument(
                "publish_tf",
                default_value="true",
                description="Publish static TF transforms for sensors",
            ),
            DeclareLaunchArgument(
                "use_measurement_covariance_models",
                default_value="true",
                description=(
                    "If true, kf_tracker uses range/bearing-based measurement covariance models. "
                    "If false, uses constant per-source covariance (R_meas_*_xy)."
                ),
            ),
            DeclareLaunchArgument(
                "start_ego_motion",
                default_value="true",
                description=(
                    "Start the GPS-based ego motion publisher (publishes /ego_motion). "
                    "Set false to run the stack without GPS on a stationary platform."
                ),
            ),
            DeclareLaunchArgument(
                "start_gps",
                default_value="false",
                description="If true, launch u-blox + NTRIP GPS stack inside this launch",
            ),
            DeclareLaunchArgument(
                "gps_device",
                default_value="/dev/ttyACM0",
                description="Serial device for u-blox receiver (e.g., /dev/ttyACM0)",
            ),
            DeclareLaunchArgument(
                "gps_baudrate",
                default_value="115200",
                description="Serial baudrate for u-blox receiver",
            ),
            DeclareLaunchArgument(
                "ublox_config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("tractor_safety_system_launch"),
                        "config",
                        "zed_f9p_rover.yaml",
                    ]
                ),
                description=(
                    "YAML config file for ublox_gps_node. Defaults to the workspace-owned "
                    "tractor_safety_system_launch/config/zed_f9p_rover.yaml."
                ),
            ),
            DeclareLaunchArgument(
                "rtcm_topic",
                default_value="/rtcm",
                description="RTCM corrections topic published by ntrip_client",
            ),
            DeclareLaunchArgument(
                "start_ntrip",
                default_value="true",
                description="If false, do not launch the NTRIP client (GPS only)",
            ),
            DeclareLaunchArgument(
                "enable_ntrip_fix_rate_limit",
                default_value="true",
                description=(
                    "If true, rate-limit NavSatFix fed to ntrip_client (controls GGA rate)"
                ),
            ),
            DeclareLaunchArgument(
                "ntrip_fix_topic",
                default_value="/gps/fix_ntrip",
                description="NavSatFix topic used by ntrip_client for generating NMEA GGA",
            ),
            DeclareLaunchArgument(
                "ntrip_fix_rate_hz",
                default_value="1.0",
                description=(
                    "Rate (Hz) for NavSatFix messages fed into ntrip_client (controls GGA rate)"
                ),
            ),
            DeclareLaunchArgument(
                "ntrip_host",
                default_value="",
                description="NTRIP caster hostname (required when start_gps:=true)",
            ),
            DeclareLaunchArgument(
                "ntrip_port",
                default_value="2101",
                description="NTRIP caster port",
            ),
            DeclareLaunchArgument(
                "ntrip_version",
                default_value="Ntrip/2.0",
                description=(
                    "Value for the NTRIP version header (RTK2go typically needs Ntrip/2.0)"
                ),
            ),
            DeclareLaunchArgument(
                "ntrip_mountpoint",
                default_value="",
                description="NTRIP mountpoint (required when start_gps:=true)",
            ),
            DeclareLaunchArgument(
                "ntrip_authenticate",
                default_value="false",
                description="If true, authenticate to the caster with username/password",
            ),
            DeclareLaunchArgument(
                "ntrip_username",
                default_value="",
                description="NTRIP username",
            ),
            DeclareLaunchArgument(
                "ntrip_password",
                default_value="",
                description="NTRIP password",
            ),
            DeclareLaunchArgument(
                "rtcm_message_package",
                default_value="rtcm_msgs",
                description="RTCM message type package for ntrip_client (rtcm_msgs recommended)",
            ),
            DeclareLaunchArgument(
                "ublox_package",
                default_value="ublox_gps",
                description="Package providing the u-blox node",
            ),
            DeclareLaunchArgument(
                "ublox_executable",
                default_value="ublox_gps_node",
                description="Executable for the u-blox node",
            ),
            DeclareLaunchArgument(
                "ntrip_package",
                default_value="ntrip_client",
                description="Package providing the NTRIP client node",
            ),
            DeclareLaunchArgument(
                "ntrip_executable",
                default_value="ntrip_ros.py",
                description="Executable for the NTRIP client node",
            ),

            # GPS configuration
            DeclareLaunchArgument(
                "gps_fix_topic",
                default_value="/gps/fix",
                description="GPS fix topic (sensor_msgs/NavSatFix)",
            ),
            DeclareLaunchArgument(
                "gps_vel_topic",
                default_value="/gps/vel",
                description=(
                    "GPS velocity topic (geometry_msgs/TwistStamped) "
                    "(if available)"
                ),
            ),
            DeclareLaunchArgument(
                "gps_heading_topic",
                default_value="/gps/heading",
                description=(
                    "GPS heading/course topic (std_msgs/Float64, degrees) "
                    "(if available)"
                ),
            ),
            DeclareLaunchArgument(
                "use_gps_vel",
                default_value="false",
                description=(
                    "Set to true if GPS publishes velocity topic; "
                    "false to estimate from position"
                ),
            ),
            DeclareLaunchArgument(
                "use_gps_heading",
                default_value="false",
                description="Set to true if GPS publishes heading/course topic",
            ),
            # Sensor TF arguments (optional overrides)
            DeclareLaunchArgument(
                "camera_tf_x",
                default_value="0.0",
                description="Camera X offset from base_link (meters) - MEASURE ON TRACTOR!",
            ),
            DeclareLaunchArgument(
                "camera_tf_y",
                default_value="0.0",
                description="Camera Y offset from base_link (meters) - MEASURE ON TRACTOR!",
            ),
            DeclareLaunchArgument(
                "camera_tf_z",
                default_value="0.0",
                description="Camera Z offset from base_link (meters) - MEASURE ON TRACTOR!",
            ),
            DeclareLaunchArgument(
                "camera_tf_roll",
                default_value="-1.5708",
                description="Camera roll (radians)",
            ),
            DeclareLaunchArgument(
                "camera_tf_pitch",
                default_value="0.0",
                description="Camera pitch (radians)",
            ),
            DeclareLaunchArgument(
                "camera_tf_yaw",
                default_value="-1.5708",
                description="Camera yaw (radians)",
            ),
            DeclareLaunchArgument(
                "radar_tf_x",
                default_value="0.0",
                description="Radar X offset from base_link (meters) - MEASURE ON TRACTOR!",
            ),
            DeclareLaunchArgument(
                "radar_tf_y",
                default_value="0.0",
                description="Radar Y offset from base_link (meters) - MEASURE ON TRACTOR!",
            ),
            DeclareLaunchArgument(
                "radar_tf_z",
                default_value="0.0",
                description="Radar Z offset from base_link (meters) - MEASURE ON TRACTOR!",
            ),
            DeclareLaunchArgument(
                "radar_tf_roll", default_value="0.0", description="Radar roll (radians)"
            ),
            DeclareLaunchArgument(
                "radar_tf_pitch", default_value="0.0", description="Radar pitch (radians)"
            ),
            DeclareLaunchArgument(
                "radar_tf_yaw", default_value="0.0", description="Radar yaw (radians)"
            ),
            # Nodes
            gps_stack,
            perception_stack,
            logger_node,
            ego_motion_node,
        ]
    )
