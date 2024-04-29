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

    andino_server_node = Node(
        package='andino_fleet',
        executable='andino_server',
        name='andino_server_node'
    )

    return LaunchDescription([
        andino_server_node        
    ])