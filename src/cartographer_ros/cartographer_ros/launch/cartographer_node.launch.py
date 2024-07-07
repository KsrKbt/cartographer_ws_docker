import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    cartographer_prefix = get_package_share_directory('cartographer_ros')
    configuration_directory = os.path.join(cartographer_prefix, 'configuration_files')
    configuration_basename = 'my_config.lua'  # Replace with your actual configuration file name

    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }],
            arguments=[
                '-configuration_directory', configuration_directory,
                '-configuration_basename', configuration_basename
            ],
            remappings=[
                ('/scan', '/scan'),  # Ensure these match your actual topic names
                ('/odom', '/odom'),
                ('/tf', '/tf')
            ]
        )
    ])
