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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _parse_calibration_tf_file(path: str) -> dict:
    """
    Parse a calibration_tf.txt file.

    Expected lines (comments with # ignored):
        radar_tf:  x y z roll pitch yaw
        camera_tf: x y z roll pitch yaw

    Returns a dict with keys 'radar_tf' and/or 'camera_tf', each a list of
    6 floats [x, y, z, roll, pitch, yaw].
    """
    result: dict = {}
    if not path:
        return result
    abs_path = os.path.abspath(path)
    if not os.path.exists(abs_path):
        print(f"[perception_stack] WARNING: tf_file not found: {abs_path}")
        return result
    with open(abs_path, encoding="utf-8") as f:
        for raw_line in f:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            for key in ("radar_tf", "camera_tf"):
                if line.startswith(key + ":"):
                    parts = line.split(":", 1)[1].strip().split()
                    if len(parts) >= 6:
                        result[key] = [float(p) for p in parts[:6]]
    return result


def _make_tf_nodes(context, *args, **kwargs):
    """
    Resolve TF values from file plus individual overrides.

    Priority (highest wins):
        1. Individual tf arg set to a non-empty value on the command line.
        2. Value from tf_file (if provided and parseable).
        3. Default of 0.0.
    """
    tf_file = LaunchConfiguration("tf_file").perform(context).strip()
    parsed = _parse_calibration_tf_file(tf_file)

    if tf_file and parsed:
        print(f"[perception_stack] Loaded TF from file: {tf_file}")
        for k, v in parsed.items():
            print(f"  {k}: {' '.join(str(x) for x in v)}")
    elif tf_file and not parsed:
        print(f"[perception_stack] WARNING: tf_file set but could not be parsed: {tf_file}")

    radar_file = parsed.get("radar_tf")  # [x, y, z, roll, pitch, yaw] or None
    camera_file = parsed.get("camera_tf")

    def _resolve(arg_name, file_values, idx):
        """Return string value: arg override > file value > '0.0'."""
        val = LaunchConfiguration(arg_name).perform(context).strip()
        if val:  # explicitly set on command line
            return val
        if file_values is not None:
            return str(file_values[idx])
        return "0.0"

    rad_x = _resolve("radar_tf_x", radar_file, 0)
    rad_y = _resolve("radar_tf_y", radar_file, 1)
    rad_z = _resolve("radar_tf_z", radar_file, 2)
    rad_roll = _resolve("radar_tf_roll", radar_file, 3)
    rad_pitch = _resolve("radar_tf_pitch", radar_file,  4)
    rad_yaw = _resolve("radar_tf_yaw", radar_file, 5)

    cam_x = _resolve("camera_tf_x", camera_file, 0)
    cam_y = _resolve("camera_tf_y", camera_file, 1)
    cam_z = _resolve("camera_tf_z", camera_file, 2)
    cam_roll = _resolve("camera_tf_roll", camera_file, 3)
    cam_pitch = _resolve("camera_tf_pitch", camera_file, 4)
    cam_yaw = _resolve("camera_tf_yaw", camera_file, 5)

    do_publish = LaunchConfiguration("publish_tf").perform(context).strip().lower() in (
        "true",
        "1",
        "yes",
    )

    if not do_publish:
        return []

    tf_cam = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_camera",
        arguments=[
            "--x", cam_x, "--y", cam_y, "--z", cam_z,
            "--yaw", cam_yaw, "--pitch", cam_pitch, "--roll", cam_roll,
            "--frame-id", "base_link",
            "--child-frame-id", "camera_link",
        ],
        output="screen",
    )

    tf_radar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_radar",
        arguments=[
            "--x", rad_x, "--y", rad_y, "--z", rad_z,
            "--yaw", rad_yaw, "--pitch", rad_pitch, "--roll", rad_roll,
            "--frame-id", "base_link",
            "--child-frame-id", "radar_link",
        ],
        output="screen",
    )

    return [tf_cam, tf_radar]


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

    use_measurement_covariance_models = LaunchConfiguration(
        "use_measurement_covariance_models"
    )

    can_channel = LaunchConfiguration("can_channel")

    # Parameters file
    # Default to this package's config (real-world/perception defaults).
    launch_pkg_share = get_package_share_path("tractor_safety_system_launch")
    default_params = os.path.join(launch_pkg_share, "config", "parameters.yaml")

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
        parameters=[
            params,
            {"use_measurement_covariance_models": use_measurement_covariance_models},
        ],
        condition=IfCondition(start_tracker),
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
                "use_measurement_covariance_models",
                default_value="true",
                description=(
                    "If true, kf_tracker uses range/bearing-based measurement covariance models. "
                    "If false, uses constant per-source covariance (R_meas_*_xy)."
                ),
            ),
            DeclareLaunchArgument(
                "publish_tf",
                default_value="true",
                description="Publish static TF transforms for sensors",
            ),
            # TF file (base_link -> sensor_link for all sensors).
            # When set, values are read from the file and used as defaults.
            # Individual tf_* args below will override the file value when provided.
            DeclareLaunchArgument(
                "tf_file",
                default_value="",
                description=(
                    "Path to calibration TF file (calibration_tf.txt format). "
                    "Values are used as defaults; individual tf_* args override them."
                ),
            ),
            # Camera TF (base_link -> camera_link).
            # NOTE: camera_node converts incoming detections from optical axes to camera_link
            # online, so this TF should represent only the physical mounting of camera_link
            # on the vehicle (no fixed optical correction).
            # Leave empty to use tf_file value (or 0.0 if no file given).
            DeclareLaunchArgument(
                "camera_tf_x",
                default_value="",
                description="Camera X offset from base_link (meters). Empty = use tf_file.",
            ),
            DeclareLaunchArgument(
                "camera_tf_y",
                default_value="",
                description="Camera Y offset from base_link (meters). Empty = use tf_file.",
            ),
            DeclareLaunchArgument(
                "camera_tf_z",
                default_value="",
                description="Camera Z offset from base_link (meters). Empty = use tf_file.",
            ),
            DeclareLaunchArgument(
                "camera_tf_roll",
                default_value="",
                description="Camera roll angle (radians). Empty = use tf_file.",
            ),
            DeclareLaunchArgument(
                "camera_tf_pitch",
                default_value="",
                description="Camera pitch angle (radians). Empty = use tf_file.",
            ),
            DeclareLaunchArgument(
                "camera_tf_yaw",
                default_value="",
                description="Camera yaw angle (radians). Empty = use tf_file.",
            ),
            # Radar TF (base_link -> radar_link).
            # Leave empty to use tf_file value (or 0.0 if no file given).
            DeclareLaunchArgument(
                "radar_tf_x",
                default_value="",
                description="Radar X offset from base_link (meters). Empty = use tf_file.",
            ),
            DeclareLaunchArgument(
                "radar_tf_y",
                default_value="",
                description="Radar Y offset from base_link (meters). Empty = use tf_file.",
            ),
            DeclareLaunchArgument(
                "radar_tf_z",
                default_value="",
                description="Radar Z offset from base_link (meters). Empty = use tf_file.",
            ),
            DeclareLaunchArgument(
                "radar_tf_roll",
                default_value="",
                description="Radar roll angle (radians). Empty = use tf_file.",
            ),
            DeclareLaunchArgument(
                "radar_tf_pitch",
                default_value="",
                description="Radar pitch angle (radians). Empty = use tf_file.",
            ),
            DeclareLaunchArgument(
                "radar_tf_yaw",
                default_value="",
                description="Radar yaw angle (radians). Empty = use tf_file.",
            ),
            # Camera driver (optional - launches depthai-ros-driver only)
            # camera_node is launched separately below as part of the perception stack
            camera_driver_launch,
            # Sensor interface nodes
            radar_node,
            camera_node,
            fusion_node,
            tracker_node,
            # TF transforms resolved from tf_file + optional per-arg overrides
            OpaqueFunction(function=_make_tf_nodes),
        ]
    )
