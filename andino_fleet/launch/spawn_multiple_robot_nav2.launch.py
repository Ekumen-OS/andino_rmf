import os


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
import yaml

# helper function to convert dictionary to string for parsing to execution process
def convert_to_text(data: dict):
    text = '\"'
    for k,v in data.items():
        robot_data = str(k) + '=' + str(v) + ';'
        text += robot_data
    text += '\"'
    return text

def generate_launch_description():
    config_name = 'spawn_robots.yaml'
    config__file_path = os.path.join(get_package_share_directory('andino_fleet'),'config',config_name)
    with open(config__file_path,'r') as f:
        config = yaml.load(f, Loader=yaml.SafeLoader)
    
    # convert dictionary to text for using as an spawning argument
    config_txt = convert_to_text(config)
    # execute andino simulation with nav2 enabled
    robots = ExecuteProcess(
        cmd=[[
            'ros2 launch andino_gz andino_gz.launch.py ',
            'robots:=',
            config_txt,
            ' rviz:=', 'True',
            ' world_name:=', 'populated_office.sdf',
            ' nav2:=', 'True',
            ' map:=', 'office',
        ]],
        shell=True
    )

    ld = LaunchDescription()
    ld.add_action(robots)
    return ld
