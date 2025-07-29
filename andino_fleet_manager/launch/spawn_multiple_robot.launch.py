import os


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import yaml

# Helper function to convert dictionary to string for parsing to execution process
def convert_to_text(data: dict):
    text = '\"'
    for k,v in data.items():
        robot_data = str(k) + '=' + str(v) + ';'
        text += robot_data
    text += '\"'
    return text

def generate_launch_description():
    config_name = 'spawn_robots.yaml'
    config__file_path = os.path.join(get_package_share_directory('andino_fleet_manager'),'config',config_name)
    with open(config__file_path,'r') as f:
        config = yaml.load(f, Loader=yaml.SafeLoader)

    # Convert dictionary to text for using as an spawning argument
    config_txt = convert_to_text(config)
    # Execute andino simulation
    robots = ExecuteProcess(
        cmd=[[
            'ros2 launch andino_gz andino_gz.launch.py ',
            ' nav2:=', 'True',
            'robots:=',
            config_txt,
            ' rviz:=', 'False',
            ' world_name:=', 'populated_office.sdf',
        ]],
        shell=True
    )

    ld = LaunchDescription()
    ld.add_action(robots)

    return ld
