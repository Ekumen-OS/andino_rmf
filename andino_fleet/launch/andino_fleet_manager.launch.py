from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():

    fleet_manager = Node(
        package= 'andino_fleet',
        name= 'fleet_manager_node',
        executable= 'fleet_manager'
    )
    ld = LaunchDescription()
    ld.add_action(fleet_manager)

    return ld
