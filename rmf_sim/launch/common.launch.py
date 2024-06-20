'''
This is the launch file for

1. RMF resources
2. Fleet adapter
3. Fleet manager
'''
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Declare launch configuration
    map_path = LaunchConfiguration('map_path')
    map_arg = DeclareLaunchArgument('map_path', description='Building description file required by rmf_building_map_tools')
    
    
    fleet_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('andino_fleet'), 'launch'),
            '/andino_fleet_manager.launch.py'
        ])
    )

    fleet_adapter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('andino_fleet_adapter'), 'launch'),
            '/andino_fleet_adapter.launch.py'
        ])
    )
    
    # Traffic schedule
    traffic_schedule = Node(
        package= 'rmf_traffic_ros2',
        executable= 'rmf_traffic_schedule',
        name= 'rmf_traffic_schedule_primary',
        output= 'both',
        parameters= [{'use_sim_time': True}],
    )

    # Building map
    build_map = Node(
        package= 'rmf_building_map_tools',
        executable= 'building_map_server',
        arguments= [map_path],
        parameters= [{'use_sim_time': True}],
    )

    # Blockade moderator
    blockade_monitor = Node(
        package= 'rmf_traffic_ros2',
        executable= 'rmf_traffic_blockade',
        output= 'both'
    )

    # Visualizer

    
    
    
    
    ld = LaunchDescription()
    ld.add_action(map_arg)
    ld.add_action(fleet_manager)
    ld.add_action(fleet_adapter)
    ld.add_action(traffic_schedule)
    ld.add_action(build_map)
    ld.add_action(blockade_monitor)

    return ld