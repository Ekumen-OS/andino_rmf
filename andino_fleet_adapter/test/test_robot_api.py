'''
This test script is intended to test all functions available in RobotClientAPI module.

To run the test,

1. launch the simulation with controller servers.

    ros2 launch andino_fleet spawn_multiple_robot.launch.py

2. run the fleet manager.

    ros2 run andino_fleet fleet_manager
    
3. run test using the following command,

    pytest-3 ~/ros2_ws/src/andino_fleet_open_rmf/andino_fleet_adapter/test/test_robot_api.py
'''

import pytest
import rclpy
from rclpy.node import Node
from andino_fleet_adapter.RobotClientAPI import RobotAPI


rclpy.init()
@pytest.fixture
def setup():
    # create a new dummy node
    test_node = Node('test_robot_api_node')
    # create RobotAPI object which requires test_node as argument
    robot_api = RobotAPI(test_node)
    return robot_api

# test functions
def test_connection(setup):
    is_connected = setup.check_connection()
    assert is_connected == True


def test_get_position(setup):
    # get andino1 position
    robot1_pose = setup.position('andino1')
    assert robot1_pose is not None

    # get andino2 position
    robot2_pose = setup.position('andino2')
    assert robot2_pose is not None

def test_navigate(setup):
    # send goal to andino1
    is_goal1_sent = setup.navigate('andino1', [1.0, 1.0, 0.0], '')
    assert is_goal1_sent == True

    # send goal to andino2
    is_goal2_sent = setup.navigate('andino2', [-0.7, 0.2, 0.0], '')
    assert is_goal2_sent == True

def test_stop(setup):
    # cancel current goal
    is_robot1_stop = setup.stop('andino1')
    assert is_robot1_stop == True

    is_robot2_stop = setup.stop('andino2')
    assert is_robot2_stop == True

def test_navigation_duration(setup):
    # check remaining duration
    duration1 = setup.navigation_remaining_duration('andino1')
    assert duration1 is not None

    duration2 = setup.navigation_remaining_duration('andino2')
    assert duration2 is not None

def test_navigation_completed(setup):
    # check navigation completed
    is_completed1 = setup.navigation_completed('andino1')
    assert is_completed1 == False

    is_completed2 = setup.navigation_completed('andino2')
    assert is_completed2 == False
