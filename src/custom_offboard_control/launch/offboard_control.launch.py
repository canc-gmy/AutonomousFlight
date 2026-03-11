from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='custom_offboard_control',
            executable='offboard_control_node',
            name='custom_offboard_control',
            output='screen'
        )
    ])
