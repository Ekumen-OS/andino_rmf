import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('andino_gz'), 'launch'),
            '/andino_gz.launch.py'
        ]),
        launch_arguments={
            'ros_bridge': 'true',
            'rviz': 'true'
        }.items()
    )

    action_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('andino_fleet'), 'launch'),
            '/andino_controller.launch.py'
        ])
    )
    
    return LaunchDescription([
        robot,
        action_server,
    ])