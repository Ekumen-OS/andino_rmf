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
    robot_config_name = 'spawn_robots.yaml'
    robot_config_file_path = os.path.join(get_package_share_directory('andino_fleet_manager'), 'config', robot_config_name)
    with open(robot_config_file_path,'r') as f:
        robot_config = yaml.load(f, Loader=yaml.SafeLoader)

    # Convert dictionary to text for using as an spawning argument
    robot_config_txt = convert_to_text(robot_config)
    robots = ExecuteProcess(
        cmd=[[
            'ros2 launch andino_gz andino_gz.launch.py ',
            'robots:=',
            robot_config_txt,
            ' rviz:=', 'True',
            ' world_name:=', 'office.sdf',
            ' nav2:=', 'True',
            ' map:=', 'office',
            ' autostart:=', 'True'
        ]],
        shell=True
    )

    ld = LaunchDescription()
    ld.add_action(robots)

    return ld
