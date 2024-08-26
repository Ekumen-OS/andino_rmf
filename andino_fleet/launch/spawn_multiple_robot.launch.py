import os


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
import yaml

# helper function to launch multiple controller servers
def launch_servers(config: dict):
    robot_actions = []

    for k,v in config.items():
        # topic remapping
        topic_remappings = {
            'velocity_topic': '/'+str(k)+'/cmd_vel',
            'odom_topic': '/'+str(k)+'/odom',
            'pose_topic': '/'+str(k)+'/current_pose',
        }
        
        # launch description for 1 action server
        action_server = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('andino_fleet'), 'launch'),
                '/andino_controller.launch.py'
            ]),
            launch_arguments=topic_remappings.items()
        )
        # wrap robot inside a namespace
        robot_w_namespace = GroupAction(
            actions=[
                PushRosNamespace(str(k)),
                action_server,
            ]
        )
        robot_actions.append(robot_w_namespace)

    return robot_actions

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
    # execute andino simulation
    robots = ExecuteProcess(
        cmd=[[
            'ros2 launch andino_gz andino_gz.launch.py ',
            'robots:=',
            config_txt,
            ' rviz:=', 'False',
            ' world_name:=', 'populated_office.sdf',
        ]],
        shell=True
    )
    # launch action servers
    controller_servers = launch_servers(config=config)

    ld = LaunchDescription()
    ld.add_action(robots)
    for g in controller_servers:
        ld.add_action(g)
    return ld
