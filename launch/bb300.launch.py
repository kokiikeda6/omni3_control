from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
import os

def generate_launch_description():

    teleop_share = FindPackageShare('omni3_control').find('omni3_control')
    teleop_path = os.path.join(teleop_share, 'launch', 'teleop.launch.py')

    omni3_control_node = Node(
            package='omni3_control',
            executable='omni_drive_node',
            name='omni_drive_node',
            output='screen',
        )

    return LaunchDescription([
	omni3_control_node,
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(teleop_path)
    ),
    ])
