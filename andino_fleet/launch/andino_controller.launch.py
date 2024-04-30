import os


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace


def generate_launch_description():

    config_file = 'controller.yaml'
    andino_server_node = Node(
        package='andino_fleet',
        executable='andino_server',
        name='andino_server_node',
        parameters= [os.path.join(get_package_share_directory('andino_fleet'), 'config', config_file)]
    )

    return LaunchDescription([
        andino_server_node        
    ])