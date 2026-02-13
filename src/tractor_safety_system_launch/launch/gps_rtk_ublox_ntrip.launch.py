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
Bring up RTK GPS using u-blox driver + NTRIP.

This is intended for ArduSimple RTK2B-style setups where:
- `ublox_gps` publishes `sensor_msgs/NavSatFix` on `/fix`
- an NTRIP client provides RTCM corrections (either as a ROS topic and/or by writing
    the raw RTCM bytes to a receiver UART)

RTCM corrections are expected to be forwarded from the NTRIP client to the ublox node
via its `rtcm_input` parameter (topic name).

Exact parameters/executables can vary slightly between Jazzy package builds.
If a node fails to start, inspect its parameters with:
- `ros2 run <pkg> <exe> --ros-args --help`
- `ros2 param list /<node_name>`
"""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Ensure the colcon overlay that provides this launch file also takes
    # precedence when resolving other packages (notably ublox_gps).
    #
    # This helps avoid accidentally using the system-installed ublox_gps when the
    # shell environment was not sourced as expected.
    launch_pkg_share = get_package_share_directory("tractor_safety_system_launch")
    launch_pkg_prefix = os.path.abspath(os.path.join(launch_pkg_share, os.pardir, os.pardir))

    # This workspace uses a per-package install layout:
    #   <ws>/install/<pkg>/share/<pkg>
    # so we need to prepend the *package prefixes* we care about.
    workspace_install_dir = os.path.abspath(os.path.join(launch_pkg_prefix, os.pardir))

    prefixes_to_prepend: list[str] = []
    for pkg_prefix in (
        os.path.join(workspace_install_dir, "ublox_gps"),
        launch_pkg_prefix,
    ):
        if os.path.isdir(pkg_prefix):
            prefixes_to_prepend.append(pkg_prefix)

    existing = os.environ.get("AMENT_PREFIX_PATH", "")
    prefix_parts = [p for p in prefixes_to_prepend if p]
    if existing:
        prefix_parts.append(existing)
    am_prefix_path = os.pathsep.join(prefix_parts)

    # Important: update the launch process environment *now* so that any ament index
    # lookups performed while constructing the Node actions will see the overlay.
    os.environ["AMENT_PREFIX_PATH"] = am_prefix_path

    try:
        ublox_prefix = get_package_prefix("ublox_gps")
    except Exception as exc:  # pragma: no cover
        ublox_prefix = f"<unresolved: {exc}>"

    # u-blox
    gps_device = LaunchConfiguration("gps_device")
    gps_baudrate = LaunchConfiguration("gps_baudrate")
    gps_fix_topic = LaunchConfiguration("gps_fix_topic")
    ublox_config_file = LaunchConfiguration("ublox_config_file")
    rtcm_topic = LaunchConfiguration("rtcm_topic")

    # ntrip
    ntrip_host = LaunchConfiguration("ntrip_host")
    ntrip_port = LaunchConfiguration("ntrip_port")
    ntrip_mountpoint = LaunchConfiguration("ntrip_mountpoint")
    ntrip_username = LaunchConfiguration("ntrip_username")
    ntrip_password = LaunchConfiguration("ntrip_password")
    ntrip_authenticate = LaunchConfiguration("ntrip_authenticate")
    ntrip_version = LaunchConfiguration("ntrip_version")
    start_ntrip = LaunchConfiguration("start_ntrip")

    # ntrip GGA rate limiting (RTK2go often expects ~1 Hz)
    enable_ntrip_fix_rate_limit = LaunchConfiguration("enable_ntrip_fix_rate_limit")
    ntrip_fix_topic = LaunchConfiguration("ntrip_fix_topic")
    ntrip_fix_rate_hz = LaunchConfiguration("ntrip_fix_rate_hz")

    # Alternative: generate and send GGA via the ntrip_client 'nmea' topic instead of using its
    # internal fix->GGA conversion (which can be caster-sensitive).
    use_external_gga = LaunchConfiguration("use_external_gga")
    ntrip_nmea_topic = LaunchConfiguration("ntrip_nmea_topic")
    ntrip_gga_hz = LaunchConfiguration("ntrip_gga_hz")

    # ntrip RTCM message type (ntrip_client supports mavros_msgs or rtcm_msgs)
    rtcm_message_package = LaunchConfiguration("rtcm_message_package")

    # Allow overriding package/executable names.
    # Default to the repo-local client since the upstream `ntrip_client` uses a
    # User-Agent that RTK2go can respond to with a SOURCETABLE (HTTP 200) instead
    # of RTCM data.
    ublox_package = LaunchConfiguration("ublox_package")
    ublox_executable = LaunchConfiguration("ublox_executable")
    ntrip_package = LaunchConfiguration("ntrip_package")
    ntrip_executable = LaunchConfiguration("ntrip_executable")

    ublox_node = Node(
        package=ublox_package,
        executable=ublox_executable,
        name="ublox_gps_node",
        output="screen",
        parameters=[
            # Base config file (repo-owned by default, overrideable via launch arg)
            ublox_config_file,
            # Small set of overrides for convenience
            {
                "device": gps_device,
                # ArduSimple tutorial wiring: /rtcm -> ublox `rtcm_input`
                "rtcm_input": rtcm_topic,
                "uart1": {"baudrate": gps_baudrate},
            },
        ],
        remappings=[
            ("fix", gps_fix_topic),
        ],
    )

    ntrip_params = {
        "host": ntrip_host,
        "port": ntrip_port,
        "mountpoint": ntrip_mountpoint,
        "ntrip_version": ntrip_version,
        "authenticate": ntrip_authenticate,
        "username": ntrip_username,
        "password": ntrip_password,
        "rtcm_message_package": rtcm_message_package,
    }

    ntrip_fix_rate_limiter_node = Node(
        package="tractor_safety_system_launch",
        executable="navsatfix_rate_limiter",
        name="ntrip_fix_rate_limiter",
        output="screen",
        condition=IfCondition(enable_ntrip_fix_rate_limit),
        parameters=[
            {
                "input_topic": gps_fix_topic,
                "output_topic": ntrip_fix_topic,
                "publish_hz": ntrip_fix_rate_hz,
                "require_finite_latlon": True,
                "require_valid_fix_status": False,
                "reject_zero_latlon": True,
            }
        ],
    )

    ntrip_gga_publisher_node = Node(
        package="tractor_safety_system_launch",
        executable="ntrip_gga_publisher",
        name="ntrip_gga_publisher",
        output="screen",
        condition=IfCondition(use_external_gga),
        parameters=[
            {
                "input_fix_topic": gps_fix_topic,
                "output_nmea_topic": ntrip_nmea_topic,
                "publish_hz": ntrip_gga_hz,
                "require_valid_fix_status": False,
                "reject_zero_latlon": True,
                # Many casters (incl. RTK2go) behave better if GGA indicates a basic
                # fix (quality=1) even before the receiver reports an RTK status via
                # NavSatFix.
                "quality_override": 1,
            }
        ],
    )

    ntrip_node_external_gga = Node(
        package=ntrip_package,
        executable=ntrip_executable,
        name="ntrip_client",
        output="screen",
        condition=IfCondition(use_external_gga),
        parameters=[ntrip_params],
        remappings=[
            # External GGA path: prevent internal fix->GGA by subscribing to a non-existent topic.
            ("fix", "/ntrip/unused_fix"),
            ("nmea", ntrip_nmea_topic),
            ("rtcm", rtcm_topic),
        ],
    )

    ntrip_node_fix = Node(
        package=ntrip_package,
        executable=ntrip_executable,
        name="ntrip_client",
        output="screen",
        condition=UnlessCondition(use_external_gga),
        parameters=[ntrip_params],
        remappings=[
            # Fix-based path: feed ntrip_client a rate-limited NavSatFix.
            ("fix", ntrip_fix_topic),
            ("rtcm", rtcm_topic),
        ],
    )

    ntrip_group = GroupAction(
        condition=IfCondition(start_ntrip),
        actions=[
            ntrip_fix_rate_limiter_node,
            ntrip_gga_publisher_node,
            ntrip_node_external_gga,
            ntrip_node_fix,
        ],
    )

    return LaunchDescription(
        [
            LogInfo(msg=f"Prepending AMENT_PREFIX_PATH with: {prefixes_to_prepend}"),
            LogInfo(msg=f"Resolved ublox_gps prefix: {ublox_prefix}"),
            SetEnvironmentVariable(name="AMENT_PREFIX_PATH", value=am_prefix_path),
            DeclareLaunchArgument(
                "gps_device",
                default_value="/dev/ttyACM0",
                description=(
                    "Serial device for u-blox receiver (e.g., /dev/ttyACM0)"
                ),
            ),
            DeclareLaunchArgument(
                "gps_baudrate",
                default_value="115200",
                description="Serial baudrate for u-blox receiver",
            ),
            DeclareLaunchArgument(
                "gps_fix_topic",
                default_value="/gps/fix",
                description="Output NavSatFix topic (remaps from /fix)",
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
                    "YAML config file for ublox_gps_node. Edit the workspace copy at "
                    "tractor_safety_system_launch/config/zed_f9p_rover.yaml or override this arg."
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
                "use_external_gga",
                default_value="true",
                description=(
                    "If true, generate GGA at a fixed rate and feed it to ntrip_client via "
                    "its 'nmea' topic. This avoids ntrip_client's internal fix->GGA conversion, "
                    "which can be rejected by some casters."
                ),
            ),
            DeclareLaunchArgument(
                "ntrip_nmea_topic",
                default_value="/ntrip/nmea",
                description=(
                    "NMEA topic used by ntrip_client for sending sentences (used for external GGA)"
                ),
            ),
            DeclareLaunchArgument(
                "ntrip_gga_hz",
                default_value="1.0",
                description="GGA rate (Hz) sent to the caster when use_external_gga:=true",
            ),
            DeclareLaunchArgument(
                "enable_ntrip_fix_rate_limit",
                default_value="false",
                description=(
                    "If true, republish /gps/fix to a dedicated topic at a fixed rate for NTRIP. "
                    "This keeps NTRIP GGA at ~1 Hz while allowing high-rate /gps/fix "
                    "for upstream consumers."
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
                "ntrip_authenticate",
                default_value="false",
                description=(
                    "If true, ntrip_client will authenticate using username/password. "
                    "Set false for anonymous/public mountpoints."
                ),
            ),
            DeclareLaunchArgument(
                "ntrip_host",
                default_value="rtk2go.com",
                description="NTRIP caster hostname",
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
                    "Value for the NTRIP version header. RTK2go typically requires 'Ntrip/2.0'. "
                    "Set to empty to omit the header."
                ),
            ),
            DeclareLaunchArgument(
                "ntrip_mountpoint",
                default_value="Ranta",
                description="NTRIP mountpoint",
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
                description=(
                    "RTCM message package for ntrip_client: 'rtcm_msgs' or 'mavros_msgs'. "
                    "Default is 'rtcm_msgs' to avoid extra dependencies."
                ),
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
                default_value="tractor_safety_system_launch",
                description="Package providing the NTRIP client node",
            ),
            DeclareLaunchArgument(
                "ntrip_executable",
                default_value="ntrip_tcp_client",
                description="Executable for the NTRIP client node",
            ),
            ublox_node,
            ntrip_group,
        ]
    )
