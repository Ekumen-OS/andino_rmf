import os


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

'''This launch file is used to launch an action server for 1 robot.'''
def generate_launch_description():
    # Declare the launch configuration
    cmd_vel = LaunchConfiguration('velocity_topic')
    odom = LaunchConfiguration('odom_topic')
    pose = LaunchConfiguration('pose_topic')

    cmdvel_arg = DeclareLaunchArgument('velocity_topic', default_value='/cmd_vel')
    odom_arg = DeclareLaunchArgument('odom_topic', default_value='/odom')
    pose_arg = DeclareLaunchArgument('pose_topic', default_value='/current_pose')

    config_file = 'controller.yaml'
    andino_server_node = Node(
        package='andino_fleet',
        executable='andino_server',
        name='andino_server_node',
        parameters= [os.path.join(get_package_share_directory('andino_fleet'), 'config', config_file)],
        remappings=[
            ('/cmd_vel', cmd_vel),
            ('/odom', odom),
            ('/current_pose', pose)
        ]
    )

    return LaunchDescription([
        cmdvel_arg,
        odom_arg,
        pose_arg,
        andino_server_node,        
    ])
