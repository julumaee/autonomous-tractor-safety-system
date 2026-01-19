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

#    Launch file for OAK-D S2 camera using depthai-ros-driver.
#
#    This launch file brings up the OAK-D S2 camera driver using depthai_ros_driver's
#    camera.launch.py. This is the hardware layer - it only launches the camera driver.
#
#    The camera_node (application layer) should be launched separately, typically
#    as part of the perception stack (see perception_stack.launch.py).
#
#    This launch file can run object detection in two ways:
#
#    Option 1: Neural network on camera (recommended, more efficient)
#        - Run detection directly on the camera's VPU using depthai-ros-driver
#        - More efficient, lower latency
#        - Requires a params_file with NN configuration
#
#    Option 2: External detection node
#        - Run detection on host (e.g., ultralytics_ros)
#        - More flexible, can use any detection model
#        - Requires separate detection node + conversion to Detection3DArray
#
#    Usage:
#        # Basic usage (camera driver only, no detection)
#        # Note: You'll need to launch camera_node separately for the safety system
#        ros2 launch camera_interface oak_d_s2.launch.py
#
#        # With custom configuration:
#        ros2 launch camera_interface oak_d_s2.launch.py \
#            params_file:=/path/to/your/custom_nn_config.yaml
#
#        # Custom camera name (affects topic namespaces)
#        ros2 launch camera_interface oak_d_s2.launch.py \
#            camera_name:=my_camera
#
#        # Disable depth if not needed
#        ros2 launch camera_interface oak_d_s2.launch.py \
#            enable_depth:=false
#
#        # Recommended: Launch via perception_stack.launch.py which handles both
#        # the camera driver and camera_node together
#        ros2 launch tractor_safety_system_launch perception_stack.launch.py \
#            start_camera_driver:=true

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    camera_model_arg = DeclareLaunchArgument(
        "camera_model",
        default_value="OAK-D-S2",
        description="Camera model: OAK-D-S2, OAK-D, etc.",
    )
    camera_name_arg = DeclareLaunchArgument(
        "camera_name",
        default_value="oak",
        description="Name for the camera (used in topic namespaces)",
    )
    enable_depth_arg = DeclareLaunchArgument(
        "enable_depth", default_value="true", description="Enable depth estimation"
    )
    enable_rgb_arg = DeclareLaunchArgument(
        "enable_rgb", default_value="true", description="Enable RGB camera"
    )
    enable_nn_arg = DeclareLaunchArgument(
        "enable_nn",
        default_value="true",
        description="Enable neural network (object detection) in depthai-ros-driver",
    )
    nn_type_arg = DeclareLaunchArgument(
        "nn_type",
        default_value="spatial",
        description='Neural network type: "spatial" (for object detection with depth), '
        '"rgb" (for RGB-only detection), or "none" (no NN)',
    )
    nn_config_path_arg = DeclareLaunchArgument(
        "nn_config_path",
        default_value="depthai_ros_driver/yolo",
        description='Path to neural network configuration'
        '(e.g., "depthai_ros_driver/yolo" for YOLO, '
        '"depthai_ros_driver/mobilenet" for MobileNet, or custom path)',
    )
    # Parameter file: default oak_stable.yaml
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("camera_interface"),
                "config",
                "oak_stable.yaml",
            ]
        ),
        description="Path to parameters YAML file for depthai-ros-driver. "
        "Defaults to camera_interface/config/oak_stable.yaml.",
    )
    enable_logging_arg = DeclareLaunchArgument(
        "enable_logging",
        default_value="false",
        description="Enable detailed logging of detections",
    )

    # Launch configurations
    LaunchConfiguration("camera_model")
    LaunchConfiguration("camera_name")
    LaunchConfiguration("enable_depth")
    LaunchConfiguration("enable_rgb")
    LaunchConfiguration("enable_nn")
    LaunchConfiguration("nn_type")
    LaunchConfiguration("nn_config_path")
    LaunchConfiguration("params_file")
    LaunchConfiguration("enable_logging")

    # DepthAI ROS Driver - professional way to launch OAK-D camera
    # This launch file ONLY launches the camera driver (hardware layer).
    # The camera_node (application layer) should be launched separately,
    # typically as part of the perception stack.
    #
    # This can optionally run neural networks (object detection) directly on the camera
    # using the camera's VPU, which is more efficient than running detection on the host.
    #
    # To enable object detection, provide a params_file with neural network configuration.
    # Example params file structure:
    #   /<camera_name>:
    #     ros__parameters:
    #       camera:
    #         i_nn_type: spatial  # or "rgb" for RGB-only detection
    #         i_pipeline_type: RGBD
    #       nn:
    #         i_nn_config_path: depthai_ros_driver/yolo  # or custom path
    #
    # When NN is enabled, depthai-ros-driver publishes SpatialDetectionArray to:
    #   /<camera_name>/nn/spatial_detections

    def generate_depthai_launch(context):
        """Build launch for depthai_ros_driver camera.launch.py."""
        # Resolve launch configuration values
        camera_model_val = context.launch_configurations.get("camera_model", "OAK-D-S2")
        camera_name_val = context.launch_configurations.get("camera_name", "oak")
        enable_depth_val = context.launch_configurations.get("enable_depth", "true")
        enable_rgb_val = context.launch_configurations.get("enable_rgb", "true")
        params_file_val = context.launch_configurations.get("params_file", "")

        # Build base launch arguments
        launch_args = {
            "camera_model": camera_model_val,
            "name": camera_name_val,
            "enable_depth": enable_depth_val,
            "enable_rgb": enable_rgb_val,
        }
        # Use params_file (defaults to oak_stable.yaml via launch argument,
        # but can be overridden on the command line)
        if params_file_val:
            launch_args["params_file"] = params_file_val

        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        FindPackageShare("depthai_ros_driver"),
                        "/launch",
                        "/camera.launch.py",
                    ]
                ),
                launch_arguments=launch_args.items(),
            )
        ]

    depthai_driver_launch = OpaqueFunction(function=generate_depthai_launch)

    return LaunchDescription(
        [
            # Launch arguments
            camera_model_arg,
            camera_name_arg,
            enable_depth_arg,
            enable_rgb_arg,
            enable_nn_arg,
            nn_type_arg,
            nn_config_path_arg,
            params_file_arg,
            enable_logging_arg,
            # Launch the camera driver
            depthai_driver_launch,
        ]
    )
