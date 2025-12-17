from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='omni3_control',
            executable='omni_drive_node',
            name='omni_drive_node',
            output='screen',
        ),
    ])
