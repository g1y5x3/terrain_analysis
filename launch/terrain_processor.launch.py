import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'terrain_analysis'

    # Arguments
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value=os.path.join(
            get_package_share_directory('terrain_analysis'), 
            'pcd', 
            'map.pcd'
        ),
        description='Path to the global PCD map file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (bag) time if true'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='terrain_analysis.yaml',
        description='Terrain analysis config file name'
    )

    # Config path
    config_path = PathJoinSubstitution([
        get_package_share_directory(package_name),
        'config',
        LaunchConfiguration('config_file')
    ])

    # Terrain Processor
    terrain_processor_node = Node(
        package=package_name,
        executable='terrain_processor',
        name='terrain_processor',
        output='screen',
        parameters=[
            config_path,
            {
                'map_path': LaunchConfiguration('map_path'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ]
    )

    return LaunchDescription([
        map_path_arg,
        use_sim_time_arg,
        config_file_arg,
        terrain_processor_node
    ])
