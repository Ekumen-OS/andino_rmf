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
import rclpy.executors
from rclpy.node import Node
from fleet_msg.srv import RobotControl, SendGoal, CancelGoal, RequestRobotPosition, RemoveAllGoals


class RobotAPI:
    # The constructor below accepts parameters typically required to submit
    # http requests. Users should modify the constructor as per the
    # requirements of their robot's API
    def __init__(self, prefix: str, user: str, password: str):
        self.prefix = prefix
        self.user = user
        self.password = password
        self.connected = False
        # Create a node for ROS2 service calls in this api
        self.node = Node('fleet_adapter_api')
        # Create an executor for this node to spin service in parallel
        self.executor = rclpy.executors.MultiThreadedExecutor()
        # Initialize fleet manager client to use ROS API
        self._add_goal_client = self.node.create_client(RobotControl, '/add_goal_server')
        self._send_goal_client = self.node.create_client(SendGoal, '/send_goal_server')
        self._cancel_goal_client = self.node.create_client(CancelGoal, '/cancel_goal_server')
        self._remove_goal_client = self.node.create_client(RemoveAllGoals, '/remove_goal_server')
        self._robot_state_client = self.node.create_client(RequestRobotPosition, '/robot_pose_server')

        # Test connectivity
        connected = self.check_connection()
        if connected:
            print("Successfully able to query API server")
            self.connected = True
        else:
            print("Unable to query API server")

        self.executor.add_node(self.node)

    def get_node(self):
        return self.node
    
    def check_connection(self):
        ''' Return True if connection to the robot API server is successful'''
        
        while not (self._add_goal_client.wait_for_service(timeout_sec=1.0) and self._send_goal_client.wait_for_service(timeout_sec=1.0) and self._cancel_goal_client.wait_for_service(timeout_sec=1.0)):
            self.node.get_logger().info('Fleet manager not available. Waiting again...')
        return True

    def position(self, robot_name: str):
        ''' Return [x, y, theta] expressed in the robot's coordinate frame or
            None if any errors are encountered'''
        robot_state_req = RequestRobotPosition.Request()
        robot_state_req.robot_name = robot_name
        future = self._robot_state_client.call_async(robot_state_req)

        self.executor.spin_until_future_complete(future)
        resp = future.result()
        if resp.is_robot_connected is False:
            self.node.get_logger().warning(f'{robot_name} is not online!')
            return None
        return resp.current_position

    def navigate(self, robot_name: str, pose, map_name: str):
        ''' Request the robot to navigate to pose:[x,y,theta] where x, y and
            and theta are in the robot's coordinate convention. This function
            should return True if the robot has accepted the request,
            else False'''
        # 1. cancel all goals
        cancel_goal_req = CancelGoal.Request()
        cancel_goal_req.robot_name = robot_name
        future = self._cancel_goal_client.call_async(cancel_goal_req)

        self.executor.spin_until_future_complete(future)
        resp = future.result()
        if resp.result == True:
            # 2. remove goals in queue
            remove_goal_req = RemoveAllGoals.Request()
            remove_goal_req.robot_name = robot_name
            future = self._remove_goal_client.call_async(remove_goal_req)

            self.executor.spin_until_future_complete(future)
            resp = future.result()
            if resp.result == True:
                # 3. add current pose to goal
                add_goal_req = RobotControl.Request()
                add_goal_req.robot_name = robot_name
                add_goal_req.final_pose = pose
                future = self._add_goal_client.call_async(add_goal_req)

                self.executor.spin_until_future_complete(future)
                resp = future.result()
                if resp.success == True:
                    # 4. send goal
                    send_goal_req = SendGoal.Request()
                    send_goal_req.robot_name = robot_name
                    future = self._send_goal_client.call_async(send_goal_req)

                    self.executor.spin_until_future_complete(future)
                    resp = future.result()
                    if resp.result == True:
                        return True
                    return False

                return False
            return False

        return False

    def start_process(self, robot_name: str, process: str, map_name: str):
        ''' Request the robot to begin a process. This is specific to the robot
            and the use case. For example, load/unload a cart for Deliverybot
            or begin cleaning a zone for a cleaning robot.
            Return True if the robot has accepted the request, else False'''
        # call the cancel_goal service server here
        return False

    def stop(self, robot_name: str):
        #Command the robot to stop.
        #Return True if robot has successfully stopped. Else False
        cancel_goal_req = CancelGoal.Request()
        cancel_goal_req.robot_name = robot_name
        future = self._cancel_goal_client.call_async(cancel_goal_req)

        self.executor.spin_until_future_complete(future)
        resp = future.result()
        if resp.result == True:
            return True

        return False
    
    def navigation_remaining_duration(self, robot_name: str):
        ''' Return the number of seconds remaining for the robot to reach its
            destination'''
        robot_state_req = RequestRobotPosition.Request()
        robot_state_req.robot_name = robot_name
        future = self._robot_state_client.call_async(robot_state_req)

        self.executor.spin_until_future_complete(future)
        resp = future.result()
        if resp.is_robot_connected is False:
            self.node.get_logger().warning(f'{robot_name} is not online!')
            return None
        
        # estimate duration(s) := t = distance_remaining(m) / max_velocity(m/s)
        duration = resp.distance_remaining / resp.max_lin_velocity

        return duration
    
    def navigation_completed(self, robot_name: str):
        ''' Return True if the robot has successfully completed its previous
            navigation request. Else False.'''
        robot_state_req = RequestRobotPosition.Request()
        robot_state_req.robot_name = robot_name
        future = self._robot_state_client.call_async(robot_state_req)

        self.executor.spin_until_future_complete(future)
        resp = future.result()
        if resp.is_robot_connected is False:
            self.node.get_logger().warning(f'{robot_name} is not online!')
            return False
        #self.node.get_logger().info(f'[{robot_name}] completed navigation: {resp.is_navigation_completed}')
        
        return resp.is_navigation_completed

    def process_completed(self, robot_name: str):
        ''' Return True if the robot has successfully completed its previous
            process request. Else False.'''
        # not used at the moment
        return False

    def battery_soc(self, robot_name: str):
        ''' Return the state of charge of the robot as a value between 0.0
            and 1.0. Else return None if any errors are encountered'''
        battery_status = 1.0
        return battery_status
