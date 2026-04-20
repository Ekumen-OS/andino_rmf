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

    # Convert dictionary to text for using as a spawning argument
    robot_config_txt = convert_to_text(robot_config)

    # Launches the andino_gz simulation environment with the following configurations:
    # - robots: Specifies the initial poses for multiple robots from the config file.
    # - rviz: Enables the RViz visualization tool.
    # - world_name: Sets the Gazebo world to 'office.sdf'.
    # - map: Provides the 'office' map for Nav2.
    # - nav2: Enables Nav2 for robot navigation and control.
    # - autostart: Automatically starts the Gazebo simulation.
    robots = ExecuteProcess(
        cmd=[[
            'ros2 launch andino_gz andino_gz.launch.py ',
            'robots:=',
            robot_config_txt,
            ' rviz:=', 'True',
            ' world_name:=', 'office.sdf',
            ' map:=', 'office',
            ' nav2:=', 'True',
            ' autostart:=', 'True'
        ]],
        shell=True
    )

    ld = LaunchDescription()
    ld.add_action(robots)

    return ld
