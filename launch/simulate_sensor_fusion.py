from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the target_simulator node
        Node(
            package='simulation_scripts',
            executable='target_simulator',
            name='target_simulator',
            parameters=['parameters.yaml'],
            output='screen'
        ),
        
        # Launch the agopen_simulator node
        Node(
            package='simulation_scripts',
            executable='agopen_simulator',
            name='agopen_simulator',
            output='screen'
        ),
        
        # Launch the fusion_node node
        Node(
            package='sensor_fusion',
            executable='fusion_node',
            name='fusion_node',
            parameters=['parameters.yaml'],
            output='screen'
        ),
        
        # Launch the safety_monitor node
        Node(
            package='safety_monitor',
            executable='safety_monitor',
            name='safety_monitor',
            parameters=['parameters.yaml'],
            output='screen'
        )
    ])