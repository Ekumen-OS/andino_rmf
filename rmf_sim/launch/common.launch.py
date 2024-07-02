'''
This is the launch file for

1. RMF resources
2. Fleet adapter
3. Fleet manager
'''
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Declare launch configuration
    map_path = LaunchConfiguration('map_path')
    map_name = LaunchConfiguration('map_name')
    map_arg = DeclareLaunchArgument('map_path', default_value=os.path.join(get_package_share_directory('rmf_maps'), 'maps', 'andino_office', 'andino_office.building.yaml'))
    map_name_arg = DeclareLaunchArgument('map_name', default_value='L1', description='Initial map name for the visualizer')
    
    viz_config = LaunchConfiguration('viz_config_file')
    viz_config_file_arg = DeclareLaunchArgument('viz_config_file', default_value=os.path.join(get_package_share_directory('rmf_sim'), 'rviz_config', 'office.rviz'))
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    
    # Traffic schedule
    traffic_schedule = Node(
        package= 'rmf_traffic_ros2',
        executable= 'rmf_traffic_schedule',
        name= 'rmf_traffic_schedule_primary',
        output= 'both',
        parameters= [{'use_sim_time': use_sim_time}],
    )

    # Building map
    build_map = Node(
        package= 'rmf_building_map_tools',
        executable= 'building_map_server',
        arguments= [map_path],
        parameters= [{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Blockade moderator
    blockade_monitor = Node(
        package= 'rmf_traffic_ros2',
        executable= 'rmf_traffic_blockade',
        output= 'both',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Visualizer
    visualizer = ExecuteProcess(
        cmd=[[
            'ros2 launch rmf_visualization visualization.launch.xml',
            ' use_sim_time:=', use_sim_time,
            ' map_name:=', map_name,
            ' viz_config_file:=', viz_config
        ]],
        shell=True
    )

    door_supervisor = Node(
        package= 'rmf_fleet_adapter',
        executable= 'door_supervisor',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    lift_supervisor = Node(
        package= 'rmf_fleet_adapter',
        executable= 'lift_supervisor',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    task_dispatcher = Node(
        package= 'rmf_task_ros2',
        executable= 'rmf_task_dispatcher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, 
                    {'bidding_time_window': 2.0},
                    {'use_unique_hex_string_with_task_id': 'true'},
                    {'server_uri': ''}]
    )
    
    ld = LaunchDescription()
    ld.add_action(map_arg)
    ld.add_action(map_name_arg)
    ld.add_action(viz_config_file_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(traffic_schedule)
    ld.add_action(blockade_monitor)
    ld.add_action(build_map)
    ld.add_action(visualizer)
    ld.add_action(door_supervisor)
    ld.add_action(lift_supervisor)
    ld.add_action(task_dispatcher)

    return ld
