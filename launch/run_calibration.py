from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    nodes = [
        Node(
            package='radar_interface',
            executable='radar_node',
            name='radar_node',
        ),
        Node(
            package='camera_interface',
            executable='camera_node',
            name='camera_node',
        ),
        Node(
            package='simulations',
            executable='calibration_logger',
            name='calibration_logger',
        ),
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