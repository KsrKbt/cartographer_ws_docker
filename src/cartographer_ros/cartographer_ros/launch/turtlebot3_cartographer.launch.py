import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_cartographer_node(context):
    cartographer_config_dir = context.launch_configurations['cartographer_config_dir']
    configuration_basename = context.launch_configurations['configuration_basename']
    use_sim_time = context.launch_configurations['use_sim_time']

    pbstream_path = "/root/cartographer_ws_docker/map.pbstream"
    cartographer_arguments = [
        '-configuration_directory', cartographer_config_dir,
        '-configuration_basename', configuration_basename
    ]

    if os.path.exists(pbstream_path):
        cartographer_arguments.extend(['-load_state_filename', pbstream_path])

    return [
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time == 'true'}],
            arguments=cartographer_arguments
        )
    ]

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    cartographer_config_dir = LaunchConfiguration(
        'cartographer_config_dir',
        default=os.path.join(get_package_share_directory('cartographer_ros'), 'configuration_files')
    )
    configuration_basename = LaunchConfiguration('configuration_basename', default='turtlebot3.lua')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Directory containing Cartographer configuration files'
        ),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Cartographer configuration file basename'
        ),
        OpaqueFunction(function=generate_cartographer_node),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        )
    ])

