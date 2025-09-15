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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # --------------------
    # Common args
    # --------------------
    use_sim_time = DeclareLaunchArgument('use_sim_time',
                                         default_value='true')
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
    yolo_result_topic = DeclareLaunchArgument('yolo_result_topic',
                                              default_value='/yolo/result')
    d2d_topic = DeclareLaunchArgument('dets2d_topic',
                                      default_value='/sim/cam/detections2d')
    spatial_out_topic = DeclareLaunchArgument('spatial_out_topic',
                                              default_value='/color/yolov4_Spatial_detections')
    use_d2d_relay = DeclareLaunchArgument('use_d2d_relay',
                                          default_value='true')

    # Optional: also bridge depth point cloud if your sim publishes it
    bridge_depth_cloud = DeclareLaunchArgument('bridge_depth_cloud',
                                               default_value='false')

    # In VMs this can help stability of some image libs (harmless otherwise)
    env_qt_xcb = SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb')

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
        [LaunchConfiguration('lidar_scan_topic'),
            TextSubstitution(text='@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan')],

        # clock (no LaunchConfiguration here)
        TextSubstitution(text='/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'),
    ]

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        arguments=bridge_args
    )

    # Optional bridge for depth-derived point cloud (if you have /sim/cam/depth/points)
    bridge_depth_points = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge_depth_points',
        output='screen',
        arguments=['/sim/cam/depth/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'],
        condition=IfCondition(LaunchConfiguration('bridge_depth_cloud'))
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
        }]
    )

    laser_to_radar = Node(
        package='simulations',
        executable='laser_to_radar',
        name='laser_to_radar',
        parameters=[{
            'scan_topic': LaunchConfiguration('lidar_scan_topic'),
            'base_frame': LaunchConfiguration('base_frame'),
            'sensor_frame': LaunchConfiguration('lidar_frame'),
            'y_max': 0.6,
        }])

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
        }]
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
        }]
    )

    return LaunchDescription([
        env_qt_xcb,

        # args
        use_sim_time, base_frame,
        vehicle_cmd_topic,
        rgb_topic, depth_topic, camera_info_topic,
        lidar_points_topic,
        lidar_scan_topic,
        cam_opt_frame, lidar_frame,
        cmd_vel_in, nav_cmd_out_topic, safety_out_topic,
        yolo_result_topic, d2d_topic, spatial_out_topic, use_d2d_relay,
        bridge_depth_cloud,

        # nodes
        bridge_node,
        bridge_depth_points,
        tf_cam, tf_lidar,
        spatial_from_yolo,
        laser_to_radar,
        lidar_to_radar,
        twist_to_control,
        control_to_twist,
    ])
