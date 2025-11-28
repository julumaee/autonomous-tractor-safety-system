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
    ExecuteProcess,
    SetEnvironmentVariable
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_path('simulations')
    world_dir = os.path.join(pkg_share, 'worlds')
    models_src_dir = '/home/ekulmala20/playground/src/simulations/models'
    # --------------------
    # Common args
    # --------------------
    use_sim_time = DeclareLaunchArgument('use_sim_time',
                                         default_value='True')
    base_frame = DeclareLaunchArgument('base_frame',
                                       default_value='base_link')
    # Vehicle command topic (ROS side, bridged into Gazebo diff_drive)
    vehicle_cmd_topic = DeclareLaunchArgument('vehicle_cmd_topic',
                                              default_value='/vehicle_cmd_vel')

    # Camera topics (ROS side, after bridge)
    rgb_topic = DeclareLaunchArgument('rgb_topic',
                                      default_value='/sim/cam/rgb')
    depth_topic = DeclareLaunchArgument('depth_topic',
                                        default_value='/sim/cam/depth')
    camera_info_topic = DeclareLaunchArgument('camera_info_topic',
                                              default_value='/sim/cam/camera_info')

    # LiDAR topic (ROS side, after bridge)
    lidar_points_topic = DeclareLaunchArgument('lidar_points_topic',
                                               default_value='/sim/lidar/points')
    lidar_scan_topic = DeclareLaunchArgument('lidar_scan_topic',
                                             default_value='/sim/lidar')

    # Frames coming from SDF
    cam_opt_frame = DeclareLaunchArgument(
        'camera_optical_frame',
        default_value='tractor_simple/camera_link/front_depth_optical'
    )
    lidar_frame = DeclareLaunchArgument(
        'lidar_frame',
        default_value='tractor_simple/lidar_link/front_lidar'
    )

    # Control chain topics
    cmd_vel_in = DeclareLaunchArgument('cmd_vel_in',
                                       default_value='/cmd_vel')
    nav_cmd_out_topic = DeclareLaunchArgument('nav_cmd_out_topic',
                                              default_value='/control/agopen')
    safety_out_topic = DeclareLaunchArgument('safety_out_topic',
                                             default_value='/control')

    # Perception glue
    d2d_topic = DeclareLaunchArgument('dets2d_topic',
                                      default_value='/yolo/result')
    spatial_out_topic = DeclareLaunchArgument('spatial_out_topic',
                                              default_value='/color/yolov4_Spatial_detections')
    use_d2d_relay = DeclareLaunchArgument('use_d2d_relay',
                                          default_value='true')

    # Optional bridges for teleop keyboard
    start_teleop_bridges_arg = DeclareLaunchArgument(
        'start_teleop_bridges',
        default_value='true',
        description='Whether to start keyboard control bridges'
    )
    start_teleop_bridges = LaunchConfiguration('start_teleop_bridges')

    start_rviz_bridge_arg = DeclareLaunchArgument(
        'start_rviz_bridge',
        default_value='true',
        description='Whether to start RViz bridges'
    )
    start_rviz_bridge = LaunchConfiguration('start_rviz_bridge')

    # In VMs this can help stability of some image libs (harmless otherwise)
    env_qt_xcb = SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb')
    env_gz_resources = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        models_src_dir
    )
    # --------------------
    # Gazebo launch
    # --------------------
    # Scenario/world name argument
    scenario_arg = DeclareLaunchArgument('scenario',
                                         default_value='',
                                         description='Scenario/world name')
    scenario = LaunchConfiguration('scenario')
    world_path = [
        TextSubstitution(text=os.path.join(world_dir, 'simple_field')),
        scenario,
        TextSubstitution(text='.sdf'),
    ]
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '', world_path],
        output='screen'
    )

    # --------------------
    # Bridges (one process)
    # --------------------
    bridge_args = [
        # vehicle command to Gazebo
        [LaunchConfiguration('vehicle_cmd_topic'),
            TextSubstitution(text='@geometry_msgs/msg/Twist@gz.msgs.Twist')],

        # camera
        [LaunchConfiguration('rgb_topic'),
            TextSubstitution(text='@sensor_msgs/msg/Image@gz.msgs.Image')],
        [LaunchConfiguration('depth_topic'),
            TextSubstitution(text='@sensor_msgs/msg/Image@gz.msgs.Image')],
        [LaunchConfiguration('camera_info_topic'),
            TextSubstitution(text='@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo')],

        # lidar
        [LaunchConfiguration('lidar_points_topic'),
            TextSubstitution(text='@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked')],

        # clock (no LaunchConfiguration here)
        TextSubstitution(text='/world/simple_field/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'),
        
        # Odometry and pedestrian poses
        TextSubstitution(
            text='/model/tractor_simple/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'),

    ]

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        remappings=[
            ('/world/simple_field/clock', '/clock'),
            ('/model/tractor_simple/odometry', '/odom'),
        ],
        arguments=bridge_args
    )

    # --------------------
    # Static TFs (match your SDF poses)
    # --------------------
    # base_link -> camera optical (0.5, 0, 0.9) + optical rotation RPY(-90, 0, -90)
    tf_cam = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_optical',
        arguments=[
            '0.5', '0', '0.9',   '-1.5708', '0', '-1.5708',
            LaunchConfiguration('base_frame'),
            LaunchConfiguration('camera_optical_frame')
        ],
        output='screen'
    )

    # base_link -> lidar (0.6, 0, 0.6)
    tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_lidar',
        arguments=[
            '0.6', '0', '0.6',   '0', '0', '0',
            LaunchConfiguration('base_frame'),
            LaunchConfiguration('lidar_frame')
        ],
        output='screen'
    )

    # --------------------
    # Perception adapters
    # --------------------

    spatial_from_yolo = Node(
        package='simulations',
        executable='spatial_from_yolo',
        name='spatial_from_yolo',
        output='screen',
        parameters=[{
            'dets2d_topic':      LaunchConfiguration('dets2d_topic'),
            'depth_topic':       LaunchConfiguration('depth_topic'),
            'info_topic':        LaunchConfiguration('camera_info_topic'),
            'out_topic':         LaunchConfiguration('spatial_out_topic'),
            'sample_fraction':   0.2,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # LiDAR → simple radar detections
    lidar_to_radar = Node(
        package='simulations',
        executable='lidar_to_radar',
        name='lidar_to_radar',
        output='screen',
        parameters=[{
            'cloud_topic':  LaunchConfiguration('lidar_points_topic'),
            'base_frame':   LaunchConfiguration('base_frame'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # --------------------
    # Control shims
    # --------------------
    # /cmd_vel → ControlCommand
    twist_to_control = Node(
        package='simulations',
        executable='twist_to_control',
        name='twist_to_control',
        output='screen',
        parameters=[{
            'twist_topic': LaunchConfiguration('cmd_vel_in'),
            'out_topic':   LaunchConfiguration('nav_cmd_out_topic'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            # keep other params at node defaults (wheelbase, etc.)
        }],
        condition=IfCondition(start_teleop_bridges),
    )

    # ControlCommand (from safety monitor) → /vehicle_cmd_vel (Twist to Gazebo)
    control_to_twist = Node(
        package='simulations',
        executable='control_to_twist',
        name='control_to_twist',
        output='screen',
        parameters=[{
            'in_topic':   LaunchConfiguration('safety_out_topic'),
            'out_topic':  LaunchConfiguration('vehicle_cmd_topic'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        condition=IfCondition(start_teleop_bridges),
    )

    visualize_tracks = Node(
        package='simulations',
        executable='visualize_tracks',
        name='visualize_tracks',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        condition=IfCondition(start_rviz_bridge),
    )

    ego_odom_sim = Node(
        package='simulations',
        executable='ego_odom_sim',
        name='ego_odom_sim',
        output='screen',
        parameters=[{
            'in_topic': '/odom',
            'out_topic': '/ego_motion',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    return LaunchDescription([
        env_qt_xcb,
        env_gz_resources,
        # args
        use_sim_time, base_frame, scenario_arg,
        vehicle_cmd_topic,
        rgb_topic, depth_topic, camera_info_topic,
        lidar_points_topic,
        lidar_scan_topic,
        cam_opt_frame, lidar_frame,
        cmd_vel_in, nav_cmd_out_topic, safety_out_topic,
        d2d_topic, spatial_out_topic, use_d2d_relay,
        start_teleop_bridges_arg, start_rviz_bridge_arg,
        # gazebo
        gz_sim,
        # nodes
        bridge_node,
        tf_cam, tf_lidar,
        spatial_from_yolo,
        lidar_to_radar,
        twist_to_control,
        control_to_twist,
        visualize_tracks,
        ego_odom_sim,
    ])
