import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    robots_config_file = os.path.join(
        get_package_share_directory("andino_fleet_manager"), "config/spawn_robots.yaml"
    )

    fleet_manager = Node(
        package="andino_fleet_manager",
        name="fleet_manager_node",
        executable="fleet_manager",
        arguments=["-c", robots_config_file],
    )
    ld = LaunchDescription()
    ld.add_action(fleet_manager)

    return ld
