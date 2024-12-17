from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    nav2 = LaunchConfiguration('nav2')
    nav2_arg = DeclareLaunchArgument('nav2', default_value='False', description='Enable Nav2 Controller')
    
    fleet_manager = Node(
        package= 'andino_fleet',
        name= 'fleet_manager_node',
        executable= 'fleet_manager',
        parameters=[{'nav2': nav2}]
    )
    ld = LaunchDescription()
    ld.add_action(nav2_arg)
    ld.add_action(fleet_manager)

    return ld
