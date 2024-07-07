from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/home/user/cartographer_ws/src/cartographer_ros/cartographer_ros/rviz/config2.rviz'],
            output='screen'),
    ])
