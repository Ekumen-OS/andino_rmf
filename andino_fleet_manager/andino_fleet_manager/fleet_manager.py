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
    """
    Central fleet management node for coordinating multiple Andino robots.

    This class manages a fleet of robots by maintaining RobotHandler instances
    for each robot and providing ROS2 services for navigation control and
    robot state queries. It serves as the main interface for external systems
    to interact with the robot fleet.
    """

    def __init__(self, config_yaml: dict(), node_name: str = "andino_fleet_manager"):
        """
        Initialize the Andino Fleet Manager.

        Creates a ROS2 node, initializes services for fleet management,
        and creates RobotHandler instances for each robot defined in the
        configuration.

        Args:
            config_yaml (dict): Configuration dictionary containing robot names
                              as keys and their initial poses as values
            node_name (str): Name for the ROS2 node (default: "andino_fleet_manager")
        """
        super().__init__(node_name)
        self._lock = threading.Lock()

        self._goal_callback_group = MutuallyExclusiveCallbackGroup()
        self._pose_callback_group = MutuallyExclusiveCallbackGroup()

        self._initialize_services()

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
        """
        Initialize ROS2 services for fleet management operations.

        Creates three main services:
        - /send_goal_server: For sending navigation goals to robots
        - /cancel_goal_server: For canceling active navigation goals
        - /robot_pose_server: For querying robot position and status
        """
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
        """
        Service callback for handling navigation goal requests.

        Processes requests to send robots to specific navigation goals.
        Currently logs the goal information and returns success.

        Args:
            request (SrvTypeRequest): Service request containing robot_name and final_pose
            response (SrvTypeResponse): Service response to be populated

        Returns:
            SrvTypeResponse: Response indicating success/failure of goal acceptance
        """
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
        """
        Service callback for handling goal cancellation requests.

        Processes requests to cancel active navigation goals for specific robots.
        Currently logs the cancellation request and returns success.

        Args:
            request (SrvTypeRequest): Service request containing robot_name
            response (SrvTypeResponse): Service response to be populated

        Returns:
            SrvTypeResponse: Response indicating success/failure of goal cancellation
        """
        self.get_logger().info(f"Cancelling goal for robot {request.robot_name}")

        response.result = True
        return response

    def _robot_pose_callback(self, request: SrvTypeRequest, response: SrvTypeResponse):
        """
        Service callback for handling robot position and status queries.

        Retrieves current position, velocity, and navigation status for a specific robot.
        Uses thread-safe access to robot data and provides comprehensive status information.

        Args:
            request (SrvTypeRequest): Service request containing robot_name
            response (SrvTypeResponse): Service response to be populated with robot data

        Returns:
            SrvTypeResponse: Response containing current_position, max_lin_velocity,
                           distance_remaining, is_robot_connected, and is_navigation_completed
        """
        # self.get_logger().info(f"Getting pose for robot {request.robot_name}")

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
            response.distance_remaining = 5.0
            response.is_robot_connected = True
            response.is_navigation_completed = True

        # self.get_logger().info(
        #     f"Robot {request.robot_name} position: "
        #     f"[{response.current_position[0]}, {response.current_position[1]}, {response.current_position[2]}]"
        # )
        return response


def main(argv=sys.argv):
    """
    Main entry point for the Andino Fleet Manager.

    Initializes ROS2, parses command line arguments for configuration file,
    creates the fleet manager instance, and runs the node with a multi-threaded
    executor for handling concurrent service requests.

    Args:
        argv (list): Command line arguments, defaults to sys.argv

    Command Line Arguments:
        -c, --config_file (str): Path to the configuration YAML file containing
                                robot names and initial poses (required)
    """
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
