import os


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import OpaqueFunction


def get_robots(context: LaunchContext, arg1: LaunchConfiguration, arg2: LaunchConfiguration):
    robot_num = context.perform_substitution(arg1)
    robot_name = context.perform_substitution(arg2)
    robot_actions = []

    for i in range(1, int(robot_num) + 1):
        robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('andino_gz'), 'launch'),
                '/spawn_robot.launch.py'
            ]),
            launch_arguments={
                'entity': TextSubstitution(text=robot_name + '_' + str(i)),
                'initial_pose_x': TextSubstitution(text=str(i)),sudo 
                'initial_pose_y': TextSubstitution(text=str(i))
            }.items()
        )
        robot_w_namespace = GroupAction(
            actions=[
                PushRosNamespace(robot_name + '_' + str(i)),
                robot
            ]
        )
        robot_actions.append(robot_w_namespace)

    return robot_actions

def generate_launch_description():

    robotNumArgs = DeclareLaunchArgument(
        'robot_num',default_value=TextSubstitution(text="2"),description='Number of the spawned robot'
    )
    robotNameArgs = DeclareLaunchArgument(
        'robot_name',default_value=TextSubstitution(text="andino"),description='Name of the robot'
    )

    return LaunchDescription([
        robotNumArgs,
        robotNameArgs,
        OpaqueFunction(function=get_robots, args=[LaunchConfiguration('robot_num'), LaunchConfiguration('robot_name')])
    ])
