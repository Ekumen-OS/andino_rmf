import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():

    rmf_config_file = os.path.join(get_package_share_directory('andino_fleet_adapter'), 'config.yaml')
    nav_graph_file = os.path.join(get_package_share_directory('rmf_maps'), 'maps', 'andino_office', 'nav_graphs', '0.yaml')

    '''
    fleet_adapter = ExecuteProcess(
        cmd=[[
            'ros2 run andino_fleet_adapter fleet_adapter ',
            '-c ', rmf_config_file, ' ', '-n ', nav_graph_file 
            # also pass config_file and nav_graph
        ]],
        shell=True
    )
    '''
    
    fleet_adapter = Node(
        package= 'andino_fleet_adapter',
        executable= 'fleet_adapter',
        name= 'andino_fleet_adapter',
        arguments=['-c', rmf_config_file, '-n', nav_graph_file],
        output='screen',
    )
    
    ld = LaunchDescription()
    ld.add_action(fleet_adapter)

    return ld