'''
This is the main launch file for

1. Common launch - RMF resources | Fleet manager | Fleet adapter
2. Simulation launch - Andino fleet | world | custom controllers or nav2 controllers
'''
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    # Include Common launch
    common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('andino_rmf_sim'), 'launch'),
            '/common.launch.py'
        ]),
        launch_arguments={'map_path': os.path.join(get_package_share_directory('andino_rmf_maps'), 'maps','andino_office', 'andino_office.building.yaml'),
                          'viz_config_file': os.path.join(get_package_share_directory('andino_rmf_sim'), 'rviz_config', 'office.rviz'),
                          'map_name': 'L1',
                          'use_sim_time': 'true', 
                          }.items()
    )

    fleet_adapter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('andino_fleet_adapter'), 'launch'),
            '/andino_fleet_adapter.launch.py'
        ]),
    )
    
    ld = LaunchDescription()
    ld.add_action(common_launch)
    ld.add_action(fleet_adapter)
    
    
    return ld
