import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_name = 'spawn_robots.yaml'
    default_config_file_path = os.path.join(get_package_share_directory('andino_fleet_manager'),'config', config_name)
    return LaunchDescription([
        DeclareLaunchArgument(
            'robots_config_file',
            default_value=default_config_file_path,
            description='Path to the YAML file containing robots in the fleet with their positions.'
        ),
        Node(
            package='andino_fleet_manager',
            executable='andino_fleet_manager',
            name='andino_fleet_manager_node',
            parameters=[{'robots_config_file': LaunchConfiguration('robots_config_file')}],
        ),
    ])
