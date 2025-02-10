from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    # List of all nodes to launch
    nodes = [
        # Object Simulator (Generates objects)
        Node(
            package='simulations',
            executable='object_simulator',
            name='object_simulator',
            parameters=['parameters.yaml'],
            output='screen'
        ),

        # Radar Simulator (Sends radar detections over CAN)
        Node(
            package='simulations',
            executable='radar_simulator_can',
            name='radar_simulator',
            output='screen'
        ),

        # Camera Simulator (Generates camera detections)
        Node(
            package='simulations',
            executable='camera_simulator',
            name='camera_simulator',
            output='screen'
        ),

        # AgOpen Simulator (Simulates AgOpenGPS control commands)
        Node(
            package='simulations',
            executable='agopen_simulator',
            name='agopen_simulator',
            output='screen'
        ),

        # Camera Node (Processes real or simulated camera detections)
        Node(
            package='camera_interface',
            executable='camera_node',
            name='camera_node',
            output='screen'
        ),

        # Radar Node (Processes real or simulated radar detections)
        Node(
            package='radar_interface',
            executable='radar_node_can',
            name='radar_node_can',
            output='screen'
        ),

        # Sensor Fusion Node (Combines camera & radar data)
        Node(
            package='sensor_fusion',
            executable='fusion_node',
            name='fusion_node',
            parameters=['parameters.yaml'],
            output='screen'
        ),

        # Safety Monitor (Monitors fused detections and controls vehicle state)
        Node(
            package='safety_monitor',
            executable='safety_monitor',
            name='safety_monitor',
            parameters=['parameters.yaml'],
            output='screen'
        )
    ]

    # Create shutdown handlers for all nodes
    shutdown_handlers = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=node,
                on_exit=[Shutdown()]  # Shut down all nodes when any one exits
            )
        )
        for node in nodes
    ]

    return LaunchDescription([
        # Log message to indicate test start
        LogInfo(msg="Launching all nodes..."),
        *nodes,
        *shutdown_handlers,
        # Log message to indicate test setup completion
        LogInfo(msg="All nodes launched successfully."),
    ])