import threading

from abc import ABC, abstractmethod
from enum import Enum

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.action import ActionClient
import queue

from controller_action_msg.action import AndinoController
from controller_action_msg.msg import RobotPose
from nav2_msgs.action import NavigateToPose

from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler

class ControllerType(Enum):
    Andino = "andino"
    Nav2 = "nav2"


class AndinoProxy(ABC, Node):
    """Base class for Andino Proxy"""

    def __init__(self, robot_name: str, action_client_callback_group: MutuallyExclusiveCallbackGroup, pose_subscriber_callback_group: MutuallyExclusiveCallbackGroup):
        super().__init__(robot_name)
        self._robot_name = robot_name
        self._action_client = None
        self._action_client_callback_group = action_client_callback_group
        self._goals: queue.Queue[list[float]] = queue.Queue()
        self._goals.put([1., 2., 3.])
        self.get_logger().info(f"Initializing goals with: {self._goals.queue}")
        self._lock = threading.Lock()
        self.remaining_distance = 0.0
        self._goal_handle = None
        self._pose_subscriber = None
        self._pose_subscriber_callback_group = pose_subscriber_callback_group
        self.current_pose = [0.0, 0.0, 0.0]
        self.navigation_completed = False
        self.max_lin_velocity = 10.0

        self._initialize_client_and_subscriber()
        self.get_logger().info(f"Creating Andino Proxy for {self._robot_name}")

    def _initialize_client_and_subscriber(self) -> None:
        """Abstract method to initialize the client and subscriber"""
        self._create_client()
        self._create_subscriber()

    @abstractmethod
    def _create_client(self) -> None:
        """Abstract method to create a controller client"""
        pass

    @abstractmethod
    def _create_subscriber(self) -> None:
        """Abstract method to create a subscriber"""
        pass

    def add_goal(self, goal):
        """Method to add a goal to the queue"""
        self._goals.put(goal)
        self.get_logger().info(f"Goal added: {goal}")

    def check_robot_availability(self, timeout_sec=60):
        """Method to check if the robot is available"""
        self.get_logger().info(f"Checking availability of {self._robot_name} with action {self._action_client._action_name}")
        return self._action_client.server_is_ready()


    @abstractmethod
    def send_goal(self) -> None:
        """Abstract method to create a controller client"""
        pass

    def cancel_goal(self):
        if self._goal_handle is None:
            self.get_logger().info("No goal to cancel")
            return
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self._cancel_response_callback())

    @abstractmethod
    def _feedback_callback(self, feedback_msg) -> None:
        """Abstract method to handle feedback from the action client"""
        pass

    def _goal_response_callback(self, future) -> None:
        """Abstract method to handle goal response from the action client"""
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self._goal_handle = None
            return

        self.get_logger().info('Goal accepted')

        get_result_future = self._goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def _get_result_callback(self, future) -> None:
        """Abstract method to handle result from the action client"""
        result = future.result().result
        self.navigation_completed = True
        self.get_logger().info(f'[{self._robot_name}] Result: {result}')

    def _cancel_response_callback(self, future) -> None:
        """Abstract method to handle cancel response from the action client"""
        result = future.result().result
        if result:
            self.get_logger().info('Goal canceled successfully')
        else:
            self.get_logger().info('Goal cancel failed')
        self._goal_handle = None

    def remove_all_goals(self):
        """Method to remove all goals from the queue"""
        with self._lock:
            self._goals.queue.clear()
        self.get_logger().info("All goals removed from the queue")


class AndinoCustomControllerProxy(AndinoProxy):
    """Class for the Andino Controller client"""

    def _create_client(self) -> None:
        """Method to create a client for the Andino Controller action"""
        action_name = "/" + self._robot_name + "/" + "andino_controller"
        self._action_client = ActionClient(self, AndinoController, action_name, callback_group=self._action_client_callback_group)
        self.get_logger().info(f"Creating Andino Controller client for {self._robot_name}")

    def _create_subscriber(self):
        topic_name = "/" + self._robot_name + "/" + "current_pose"
        self._pose_subscriber = self.create_subscription(RobotPose, topic_name, self._pose_callback, 10, callback_group=self._pose_subscriber_callback_group)
        self.get_logger().info(f"Creating Andino Pose subscriber for {self._robot_name}")

    def send_goal(self) -> None:
        if self._goals.empty():
            self.get_logger().info("No goals in the queue")
            return

        self.navigation_completed = False

        goal = self._goals.get()

        self.get_logger().info(f"Received goal: {goal}")
        # Create a goal message
        goal_msg = AndinoController.Goal()
        goal_msg.goal_pose.pose.position.x = goal[0]
        goal_msg.goal_pose.pose.position.y = goal[1]
        quaternion = quaternion_from_euler(0, 0, goal[2])
        orientation = Quaternion()
        orientation.x = quaternion[0]
        orientation.y = quaternion[1]
        orientation.z = quaternion[2]
        orientation.w = quaternion[3]
        goal_msg.goal_pose.pose.orientation = orientation

        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self._feedback_callback)
        send_goal_future.add_done_callback(self._goal_response_callback)

        self.get_logger().info(f"Sending goal: {goal}")

    def _feedback_callback(self, feedback_msg):
        with self._lock:
            feedback = feedback_msg.feedback
            self.remaining_distance = feedback.remaining_distance
            self.max_lin_velocity = feedback.max_lin_vel.linear.x

        self.get_logger().debug(f'[{self._robot_name}] Distance Remaining: {round(self.remaining_distance, 3)}')

    def _pose_callback(self, msg: RobotPose) -> None:
        """Callback method for the pose subscriber"""
        with self._lock:
            self.current_pose = msg.current_pose

        self.get_logger().debug(f'[{self._robot_name}] Current Pose: {self.current_pose}')

class AndinoNav2ControllerProxy(AndinoProxy):
    """Class for the Andino Controller client"""

    def _create_client(self) -> None:
        """Method to create a client for the Andino Controller action"""
        action_name = "/" + self._robot_name + "/" + "navigate_to_pose"
        self._action_client = ActionClient(self, NavigateToPose, action_name, callback_group=self._action_client_callback_group)
        self.get_logger().info(f"Creating Nav2 Controller client for {self._robot_name}")

    def send_goal(self) -> None:
        if self._goals.empty():
            self.get_logger().info("No goals in the queue")
            return

        self.navigation_completed = False

        goal = self._goals.get()

        self.get_logger().info(f"Received goal: {goal}")
        # Create a goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = goal[0]
        goal_msg.pose.pose.position.y = goal[1]

        quaternion = quaternion_from_euler(0, 0, goal[2])
        orientation = Quaternion()
        orientation.x = quaternion[0]
        orientation.y = quaternion[1]
        orientation.z = quaternion[2]
        orientation.w = quaternion[3]
        goal_msg.pose.pose.orientation = orientation

        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self._feedback_callback)
        send_goal_future.add_done_callback(self._goal_response_callback)

        self.get_logger().info(f"Sending goal: {goal}")


class AndinoProxyFactory:
    @staticmethod
    def create_andino_proxy(controller_type: ControllerType, robot_name: str, action_client_callback_group: MutuallyExclusiveCallbackGroup, pose_subscriber_callback_group: MutuallyExclusiveCallbackGroup) -> AndinoProxy:
        if controller_type == ControllerType.Andino:
            return AndinoCustomControllerProxy(robot_name, action_client_callback_group, pose_subscriber_callback_group)
        elif controller_type == ControllerType.Nav2:
            return AndinoNav2ControllerProxy(robot_name, action_client_callback_group, pose_subscriber_callback_group)
        else:
            raise ValueError("Invalid controller type")
