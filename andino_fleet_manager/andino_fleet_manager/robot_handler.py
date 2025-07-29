'''
    The RobotHandler class manages individual robot instances within the fleet
    manager. It handles pose tracking, initial pose setting, navigation goal
    management, and communication with individual robots through ROS2 topics.
    Each robot in the fleet has its own RobotHandler instance that subscribes
    to the robot's pose updates, manages its state information, and will handle
    sending navigation goals and canceling active goals for the robot.
'''

from enum import Enum
import threading

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.task import Future

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion, PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from tf_transformations import quaternion_from_euler


class ReturnFlag(Enum):
    SUCCESS = 0
    ROBOT_OFFLINE = 1



class RobotHandler:
    """
    Handles individual robot instances within the fleet manager.

    This class manages the state and communication for a single robot,
    including pose tracking, initial pose setting, navigation goal management,
    and subscribing to pose updates from the robot's AMCL localization system.
    It will also handle sending navigation goals to robots and canceling active
    goals when requested by the fleet manager.
    """

    def __init__(
        self,
        node: Node,
        robot_name: str,
        initial_pose: dict(),
        lock: threading.Lock,
        goal_callback_group: MutuallyExclusiveCallbackGroup,
        pose_callback_group: MutuallyExclusiveCallbackGroup,
    ):
        """
        Initialize a RobotHandler for managing a single robot.

        Args:
            node (Node): The ROS2 node instance to use for communication
            robot_name (str): Unique identifier for the robot
            initial_pose (dict): Dictionary containing robot's initial pose with keys:
                                'x', 'y', 'z' for position and 'yaw' for orientation
            lock (threading.Lock): Thread lock for safe concurrent access to robot data
        """
        self.node = node
        self.robot_name = robot_name
        self._lock = lock

        self._goal_handle: ClientGoalHandle = None
        self._current_pose: PoseWithCovarianceStamped = None
        self._reset_navigation_data()

        self._set_initial_pose(robot_name, initial_pose)

        # Create a subscriber to the robot's pose updates
        topic_name = "/" + robot_name + "/amcl_pose"
        self.pose_subscriber = self.node.create_subscription(
            PoseWithCovarianceStamped,
            topic_name,
            self._pose_callback,
            10,
            callback_group=pose_callback_group,
        )

        # Create an action client for sending navigation goals to the robot
        action_name = "/" + robot_name + "/navigate_to_pose"
        self._controller_client = ActionClient(
            self.node, NavigateToPose, action_name, callback_group=goal_callback_group
        )

        self.node.get_logger().info(f"Robot initialized with action client: {action_name}")

        self.node.get_logger().info(
            f"[{robot_name}] RobotHandler initialized with pose: [{initial_pose['x']}, {initial_pose['y']}, {initial_pose['z']}]"
        )

    def _set_initial_pose(self, robot_name : str, initial_pose: list()):
        """
        Set the initial pose of the robot and publish it to the robot's initialpose topic.

        This method creates and publishes a PoseWithCovarianceStamped message containing
        the robot's initial position and orientation to help with localization.

        Args:
            robot_name (str): The name of the robot
            initial_pose (dict): Dictionary containing initial pose data with keys:
                               'x', 'y', 'z' for position and 'yaw' for orientation
        """
        topic_name = "/" + robot_name + "/initialpose"
        initial_pose_publisher = self.node.create_publisher(PoseWithCovarianceStamped, topic_name, 10)
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.header.frame_id = 'map'

        initial_pose_msg.pose.pose.position.x = initial_pose["x"]
        initial_pose_msg.pose.pose.position.y = initial_pose["y"]
        initial_pose_msg.pose.pose.position.z = initial_pose["z"]
        orientation = quaternion_from_euler(0, 0, initial_pose["yaw"])

        initial_pose_msg.pose.pose.orientation.x = orientation[0]
        initial_pose_msg.pose.pose.orientation.y = orientation[1]
        initial_pose_msg.pose.pose.orientation.z = orientation[2]
        initial_pose_msg.pose.pose.orientation.w = orientation[3]

        initial_pose_publisher.publish(initial_pose_msg)
        self.node.get_logger().info(
            f"Initial pose set for robot {robot_name}: "
            f"[{initial_pose_msg.pose.pose.position.x}, "
            f"{initial_pose_msg.pose.pose.position.y}, "
            f"{initial_pose_msg.pose.pose.orientation.z}]"
        )

        self.current_pose = initial_pose_msg

    def is_robot_online(self) -> bool:
        """
        Check if the robot is online and responsive.

        Returns:
            bool: True if the robot is online, False otherwise
        """
        self.node.get_logger().info(f"Checking if robot {self.robot_name} is online")
        return self._controller_client.wait_for_server(timeout_sec=1.0)

    def _pose_callback(self, msg: PoseWithCovarianceStamped):
        """
        Callback function for processing incoming pose updates from the robot.

        This method is called whenever a new pose message is received from the
        robot's AMCL localization system. It updates the current pose with
        thread-safe access using the provided lock.

        Args:
            msg (PoseWithCovarianceStamped): The pose message received from the robot
        """
        with self._lock:
            self.node.get_logger().info(
                f"[{self.robot_name}] Current pose [{msg.pose.pose.position.x}, {msg.pose.pose.position.y}, {msg.pose.pose.orientation.z}]"
            )
            self._current_pose = msg

    def send_goal(self, goal: list()) -> ReturnFlag:
        """
        Send a navigation goal to the robot.

        This method creates a NavigateToPose action goal and sends it to the robot.
        It waits for the action server to be available and then sends the goal.

        Args:
            goal_pose (list): List containing the goal position with elements:
                              [x, y, z] for position and [yaw] for orientation
        """
        if not self.is_robot_online():
            self.node.get_logger().warning(f"Robot {self.robot_name} is offline")
            return ReturnFlag.ROBOT_OFFLINE

        self._reset_navigation_data()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = goal[0]
        goal_msg.pose.pose.position.y = goal[1]

        quaternion = quaternion_from_euler(0, 0, goal[2])
        orientation = Quaternion()
        orientation.x = quaternion[0]
        orientation.y = quaternion[1]
        orientation.z = quaternion[2]
        orientation.w = quaternion[3]
        goal_msg.pose.pose.orientation = orientation

        self.node.get_logger().info(
            f"Preparing to send goal to robot {self.robot_name}: ")
        send_goal_future = self._controller_client.send_goal_async(goal_msg, self._feedback_callback)
        send_goal_future.add_done_callback(self._goal_response_callback)

        self.node.get_logger().info(
            f"Sending goal to robot {self.robot_name}: "
            f"[{goal_msg.pose.pose.position.x}, {goal_msg.pose.pose.position.y}, {goal_msg.pose.pose.orientation.z}]"
        )
        return ReturnFlag.SUCCESS


    def _feedback_callback(self, feedback_msg : PoseStamped):
        """
        Callback function for processing feedback from the robot's navigation action.

        Args:
            feedback_msg: The feedback message received from the robot
        """
        self.node.get_logger().info(
            f"Received feedback from robot {self.robot_name}")
        feedback = feedback_msg.feedback
        self.current_pose.header = feedback.current_pose.header
        self.current_pose.pose.pose = feedback.current_pose.pose
        self._navigation_time = feedback.navigation_time
        self._estimated_time_remaining = feedback.estimated_time_remaining
        self._number_of_recoveries = feedback.number_of_recoveries
        self._distance_remaining = feedback.distance_remaining

    def _goal_response_callback(self, future: Future):
        """
        Callback function for processing the response after sending a navigation goal.

        Args:
            future: The future object containing the result of the goal sending operation
        """
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.node.get_logger().info("Goal rejected")
            return

        self.node.get_logger().info("Goal accepted")

        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future : Future):
        result = future.result().result
        if self._goal_handle.status == GoalStatus.STATUS_CANCELED:
               return
        if result.error_code != 0:
            self.node.get_logger().error(
                f"Navigation failed with error message: {result.error_msg}"
            )
            return
        self.node.get_logger().info("Navigation completed successfully")


    def cancel_goal(self, robot_name: str):
       future = self._goal_handle.cancel_goal_async()
       future.add_done_callback(self._cancel_response_callback)


    def _cancel_response_callback(self, future: Future):
       cancel_response = future.result()
       if len(cancel_response.goals_canceling) > 0:
           self.node.get_logger().info("Goal successfully cancelled")
       else:
           self.node.get_logger().info("Goal failed to cancel")


    def _reset_navigation_data(self):
        """
        Reset the navigation data for the robot.

        This method clears the navigation time, estimated time remaining,
        number of recoveries, and distance remaining to prepare for a new goal.
        """
        self.node.get_logger().info(f"Resetting navigation data for robot {self.robot_name}")
        self._navigation_time = 0
        self._estimated_time_remaining = 0
        self._number_of_recoveries = 0
        self._distance_remaining = 0
