# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


'''
    The RobotAPI class is a wrapper for API calls to the robot. Here users
    are expected to fill up the implementations of functions which will be used
    by the RobotCommandHandle. For example, if your robot has a REST API, you
    will need to make http request calls to the appropriate endpoints within
    these functions.
'''
import rclpy
from rclpy.node import Node
from fleet_msg.srv import RobotControl, SendGoal, CancelGoal, RequestRobotPosition, RemoveAllGoals

class RobotAPI:
    # The constructor below accepts parameters typically required to submit
    # http requests. Users should modify the constructor as per the
    # requirements of their robot's API
    def __init__(self, config_yaml, node: Node):
        self.prefix = config_yaml['prefix']
        self.user = config_yaml['user']
        self.password = config_yaml['password']
        self.timeout = 5.0
        self.debug = False
        self.connected = False
        self.node = node
        
        # Initialize fleet manager client to use ROS API
        self._add_goal_client = node.create_client(RobotControl, '/add_goal_server')
        self._send_goal_client = node.create_client(SendGoal, '/send_goal_server')
        self._cancel_goal_client = node.create_client(CancelGoal, '/cancel_goal_server')
        self._remove_goal_client = node.create_client(RemoveAllGoals, '/remove_goal_server')
        self._robot_state_client = node.create_client(RequestRobotPosition, '/robot_pose_server')

        # Test connectivity
        connected = self.check_connection()
        if connected:
            print("Successfully able to query API server")
            self.connected = True
        else:
            print("Unable to query API server")

        # Create empty service messages
        self._add_goal_req = RobotControl.Request()
        self._send_goal_req = SendGoal.Request()
        self._cancel_goal_req = CancelGoal.Request()
        self._remove_goal_req = RemoveAllGoals.Request()
        self._robot_state_req = RequestRobotPosition.Request()
        self._future = None

    def get_node(self):
        return self.node
    
    def check_connection(self):
        ''' Return True if connection to the robot API server is successful'''
        
        while not (self._add_goal_client.wait_for_service(timeout_sec=1.0) and self._send_goal_client.wait_for_service(timeout_sec=1.0) and self._cancel_goal_client.wait_for_service(timeout_sec=1.0)):
            self.node.get_logger().info('Fleet manager not available. Waiting again...')
        return True
    
    def navigate(
        self,
        robot_name: str,
        pose,
        map_name: str,
        speed_limit=0.0
    ):
        ''' Request the robot to navigate to pose:[x,y,theta] where x, y and
            and theta are in the robot's coordinate convention. This function
            should return True if the robot has accepted the request,
            else False '''
        self.node.get_logger().info(f'{robot_name} is navigating in {map_name}')
        # 1. cancel all goals
        self._cancel_goal_req.robot_name = robot_name
        resp = self._cancel_goal_client.call(self._cancel_goal_req)
        if resp.result == True:
            # 2. remove goals in queue
            resp = self._remove_goal_client.call(self._remove_goal_req)
            if resp.result == True:
                # 3. add current pose to goal
                self._add_goal_req.robot_name = robot_name
                self._add_goal_req.final_pose = pose
                resp = self._add_goal_client.call(self._add_goal_req)
                if resp.success == True:
                    # 4. send goal
                    self._send_goal_req.robot_name = robot_name
                    resp = self._send_goal_client.call(self._send_goal_req)
                    if resp.result == True:
                        return True
                    return False

                return False
            return False

        return False

    def start_activity(
        self,
        robot_name: str,
        activity: str,
        label: str
    ):
        ''' Request the robot to begin a process. This is specific to the robot
        and the use case. For example, load/unload a cart for Deliverybot
        or begin cleaning a zone for a cleaning robot.
        Return True if process has started/is queued successfully, else
        return False '''

        return False

    def stop(self, robot_name: str):
        ''' Command the robot to stop.
            Return True if robot has successfully stopped. Else False. '''
        #Command the robot to stop.
        self._cancel_goal_req.robot_name = robot_name
        resp = self._cancel_goal_client.call(self._cancel_goal_req)
        if resp.result == True:
            return True

        return False

    def position(self, robot_name: str):
        ''' Return [x, y, theta] expressed in the robot's coordinate frame or
        None if any errors are encountered '''
        self._robot_state_req.robot_name = robot_name
        resp = self._robot_state_client.call(self._robot_state_req)
        if resp.is_robot_connected is False:
            self.node.get_logger().warning(f'{robot_name} is not online!')
            return None
        return resp.current_position

    def battery_soc(self, robot_name: str):
        ''' Return the state of charge of the robot as a value between 0.0
        and 1.0. Else return None if any errors are encountered. '''
        self.node.get_logger().info(f'{robot_name} battery status is 1.0')
        battery_status = 1.0
        return battery_status

    def map(self, robot_name: str):
        ''' Return the name of the map that the robot is currently on or
        None if any errors are encountered. '''
        map_name = 'L1'
        self.node.get_logger().info(f'{robot_name} is currently on {map_name}')
        return map_name

    def is_command_completed(self, robot_name: str):
        ''' Return True if the robot has completed its last command, else
        return False. '''
        self._robot_state_req.robot_name = robot_name
        resp = self._robot_state_client.call(self._robot_state_req)
        if resp.is_robot_connected is False:
            self.node.get_logger().warning(f'{robot_name} is not online!')
            return False
        
        return resp.is_navigation_completed

    def get_data(self, robot_name: str):
        ''' Returns a RobotUpdateData for one robot if a name is given. Otherwise
        return a list of RobotUpdateData for all robots. '''
        map = self.map(robot_name)
        position = self.position(robot_name)
        battery_soc = self.battery_soc(robot_name)
        if not (map is None or position is None or battery_soc is None):
            return RobotUpdateData(robot_name, map, position, battery_soc)
        return None


class RobotUpdateData:
    ''' Update data for a single robot. '''
    def __init__(self,
                 robot_name: str,
                 map: str,
                 position: list[float],
                 battery_soc: float,
                 requires_replan: bool | None = None):
        self.robot_name = robot_name
        self.position = position
        self.map = map
        self.battery_soc = battery_soc
        self.requires_replan = requires_replan
