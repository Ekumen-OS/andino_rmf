'''
    The AndinoFleetManager class serves as the central coordinator for managing
    multiple Andino robots in a fleet. It provides ROS2 services for sending
    navigation goals, canceling goals, and querying robot positions. The fleet
    manager maintains a collection of RobotHandler instances, each responsible
    for managing individual robot state and communication.
'''

import argparse
import sys
import threading
import yaml

'''
    The AndinoFleetManager class serves as the central coordinator for managing
    multiple Andino robots in a fleet. It provides ROS2 services for sending
    navigation goals, canceling goals, and querying robot positions. The fleet
    manager maintains a collection of RobotHandler instances, each responsible
    for managing individual robot state and communication.
'''

import argparse
import sys
import threading
import yaml

import rclpy
import rclpy.executors as executors

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.service import SrvTypeRequest, SrvTypeResponse

from andino_fleet_manager.robot_handler import ReturnFlag, RobotHandler
from andino_fleet_msg.srv import SendGoal, CancelGoal, RequestRobotPosition


class AndinoFleetManager(Node):
    def __init__(self, config_yaml: dict(), node_name: str = "andino_fleet_manager"):

        super().__init__(node_name)
        self._lock = threading.Lock()

        self._goal_callback_group = MutuallyExclusiveCallbackGroup()
        self._pose_callback_group = MutuallyExclusiveCallbackGroup()

        self._initialize_services()

        self._initial_pose_timer = self.create_timer(1.0, self._initial_pose_timer_callback)

        self._robot_dict = dict()

        for robot_name, initial_pose in config_yaml.items():
            self._robot_dict[robot_name] = RobotHandler(
                self,
                robot_name,
                initial_pose,
                self._lock,
                self._goal_callback_group,
                self._pose_callback_group,
            )

        self.get_logger().info("Andino Fleet Manager Started")

    def _initialize_services(self):
        self._send_goal_client = self.create_service(
            SendGoal, "/send_goal_server", self._send_goal_callback
        )
        self._cancel_goal_client = self.create_service(
            CancelGoal, "/cancel_goal_server", self._cancel_goal_callback
        )
        self._robot_state_client = self.create_service(
            RequestRobotPosition, "/robot_pose_server", self._robot_pose_callback
        )

    def _send_goal_callback(self, request: SrvTypeRequest, response: SrvTypeResponse):
        if request.robot_name not in self._robot_dict:
            self.get_logger().warning(
                f"Robot {request.robot_name} not found in fleet manager"
            )
            response.result = False
            return response

        with self._lock:
            robot_handler = self._robot_dict[request.robot_name]
            send_goal_return = robot_handler.send_goal(request.final_pose)
            if send_goal_return == ReturnFlag.ROBOT_OFFLINE:
                self.get_logger().warning(
                    f"Robot {request.robot_name} is offline. Cannot send goal."
                )
                response.result = False
                return response
        response.result = True
        return response

    def _cancel_goal_callback(self, request: SrvTypeRequest, response: SrvTypeResponse):
        if request.robot_name not in self._robot_dict:
            self.get_logger().warning(
                f"Robot {request.robot_name} not found in fleet manager"
            )
            response.result = False
            return response

        with self._lock:
            robot_handler = self._robot_dict[request.robot_name]
            cancel_goal_return = robot_handler.cancel_goal()
            if cancel_goal_return == ReturnFlag.ROBOT_OFFLINE:
                self.get_logger().debug(
                    f"Robot {request.robot_name} is offline. Cannot cancel goal."
                )
                response.result = False
                return response
        response.result = True
        return response

    def _robot_pose_callback(self, request: SrvTypeRequest, response: SrvTypeResponse):
        if request.robot_name not in self._robot_dict:
            self.get_logger().warning(
                f"Robot {request.robot_name} not found in fleet manager"
            )
            response.current_position = [0.0, 0.0, 0.0]
            response.max_lin_velocity = 0.0
            response.distance_remaining = 0.0
            response.is_robot_connected = False
            response.is_navigation_completed = False
            return response

        with self._lock:
            robot_handler = self._robot_dict[request.robot_name]
            if robot_handler.current_pose is None:
                self.get_logger().warning(
                    f"Robot {request.robot_name} has no pose data available"
                )
                response.current_position = [0.0, 0.0, 0.0]
                response.max_lin_velocity = 0.0
                response.distance_remaining = 0.0
                response.is_robot_connected = False
                response.is_navigation_completed = False
                return response

            response.current_position = [
                robot_handler.current_pose.pose.pose.position.x,
                robot_handler.current_pose.pose.pose.position.y,
                robot_handler.current_pose.pose.pose.position.z,
            ]
            response.max_lin_velocity = 1.0
            response.distance_remaining = robot_handler.get_distance_remaining()
            response.is_robot_connected = robot_handler.is_robot_online() == ReturnFlag.SUCCESS
            response.is_navigation_completed = robot_handler.get_navigation_completed()
        return response

    def _initial_pose_timer_callback(self):
        all_initial_poses_published = True
        for robot_handler in self._robot_dict.values():
            if not robot_handler.initial_pose_published:
                robot_handler.publish_initial_pose()
                all_initial_poses_published = all_initial_poses_published and robot_handler.initial_pose_published
        if all_initial_poses_published:
            self._initial_pose_timer.cancel()


def main(argv=sys.argv):
    rclpy.init(args=argv)
    args_without_ros = rclpy.utilities.remove_ros_args(argv)
    parser = argparse.ArgumentParser(
        prog="fleet_adapter", description="Configure and spin up the fleet adapter"
    )
    parser.add_argument(
        "-c",
        "--config_file",
        type=str,
        required=True,
        help="Path to the config.yaml file",
    )
    args = parser.parse_args(args_without_ros[1:])

    config_path = args.config_file

    with open(config_path, "r") as f:
        config_yaml = yaml.safe_load(f)

    fleet_manager = AndinoFleetManager(config_yaml)
    executor = executors.MultiThreadedExecutor()
    executor.add_node(fleet_manager)

    try:
        executor.spin()
    except KeyboardInterrupt:
        fleet_manager.destroy_node()
        fleet_manager.get_logger().info("KeyboardInterrupt. Shutting Down...")
        fleet_manager.get_logger().info("KeyboardInterrupt. Shutting Down...")


if __name__ == "__main__":
    main()
