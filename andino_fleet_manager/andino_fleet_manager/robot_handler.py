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

from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.task import Future

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion, PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from tf_transformations import quaternion_from_euler


class ReturnFlag(Enum):
    SUCCESS = 0
    ROBOT_OFFLINE = 1
    UNINITIALIZED_GOAL_HANDLE = 2
    NO_SUBSCRIBERS = 3


class RobotHandler:
    VAR_XY = 0.25
    VAR_YAW = 0.0685
    
    IDX_X = 0
    IDX_Y = 7
    IDX_YAW = 35

    def __init__(
        self,
        node: Node,
        robot_name: str,
        initial_pose: dict(),
        lock: threading.Lock,
        goal_callback_group: MutuallyExclusiveCallbackGroup,
        pose_callback_group: MutuallyExclusiveCallbackGroup,
    ):
        self.node = node
        self.robot_name = robot_name
        self._lock = lock

        self.initial_pose_published = False
        self.initial_pose = initial_pose
        self._goal_handle: ClientGoalHandle = None
        self.current_pose: PoseWithCovarianceStamped = None
        self._reset_navigation_data()

        self.initialize_publishers_and_subscribers(pose_callback_group)

        self.initialize_actions(goal_callback_group)

        self.current_pose = self._get_initial_pose_msg(self.initial_pose)

        self.node.get_logger().debug(f"RobotHandler initialized for robot {robot_name}")

    def initialize_publishers_and_subscribers(self, callback_group: MutuallyExclusiveCallbackGroup):
        # Publisher for the initial pose
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        initial_pose_topic = "/" + self.robot_name + "/initialpose"
        self.initial_pose_publisher = self.node.create_publisher(
            PoseWithCovarianceStamped, initial_pose_topic, qos_profile)

        # Subscriber to the robot's pose updates
        topic_name = "/" + self.robot_name + "/amcl_pose"
        self.pose_subscriber = self.node.create_subscription(
            PoseWithCovarianceStamped,
            topic_name,
            self._pose_callback,
            10,
            callback_group=callback_group,
        )

    def initialize_actions(self, callback_group: MutuallyExclusiveCallbackGroup):
        # Action client for sending navigation goals to the robot
        action_name = "/" + self.robot_name + "/navigate_to_pose"
        self._controller_client = ActionClient(
            self.node, NavigateToPose, action_name, callback_group=callback_group
        )

    def publish_initial_pose(self) -> None:
        if self.initial_pose_publisher.get_subscription_count() == 0:
            self.node.get_logger().error(f"Initial pose could not be published for robot {self.robot_name} because there are no subscribers for the topic")
            return
        initial_pose_msg = self._get_initial_pose_msg(self.initial_pose)
        self.initial_pose_publisher.publish(initial_pose_msg)
        self.initial_pose_published = True

    def _get_initial_pose_msg(self, initial_pose: dict) -> PoseWithCovarianceStamped:
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.header.frame_id = 'map'
        initial_pose_msg.header.stamp = self.node.get_clock().now().to_msg()

        # Set position from the configuration dictionary, not hardcoded values.
        initial_pose_msg.pose.pose.position.x = initial_pose["x"]
        initial_pose_msg.pose.pose.position.y = initial_pose["y"]
        initial_pose_msg.pose.pose.position.z = initial_pose["z"]

        # Calculate and set orientation from the yaw value in the configuration.
        orientation = quaternion_from_euler(0.0, 0.0, initial_pose["yaw"])
        initial_pose_msg.pose.pose.orientation.x = orientation[0]
        initial_pose_msg.pose.pose.orientation.y = orientation[1]
        initial_pose_msg.pose.pose.orientation.z = orientation[2]
        initial_pose_msg.pose.pose.orientation.w = orientation[3]

        # x and y are uncertain within a 0.5m radius and its z, roll, pitch and yaw values are completely unknown
        # The covariance matrix is a 6x6 matrix stored as a 36-element array.
        # The diagonal elements correspond to variance in x, y, z, roll, pitch, yaw.
        # Covariance[0] is variance for x, [7] is for y, [14] for z, etc.
        initial_pose_msg.pose.covariance[self.IDX_X] = self.VAR_XY
        initial_pose_msg.pose.covariance[self.IDX_Y] = self.VAR_XY
        initial_pose_msg.pose.covariance[self.IDX_YAW] = self.VAR_YAW
        return initial_pose_msg

    def is_robot_online(self) -> ReturnFlag:
        if self._controller_client.wait_for_server(timeout_sec=1.0):
            return ReturnFlag.SUCCESS
        return ReturnFlag.ROBOT_OFFLINE


    def _pose_callback(self, msg: PoseWithCovarianceStamped):
        with self._lock:
            self.current_pose = msg

    def send_goal(self, goal: list()) -> ReturnFlag:
        if not self.is_robot_online():
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

        send_goal_future = self._controller_client.send_goal_async(goal_msg, self._feedback_callback)
        send_goal_future.add_done_callback(self._goal_response_callback)
        return ReturnFlag.SUCCESS

    def _feedback_callback(self, feedback_msg : PoseStamped):
        feedback = feedback_msg.feedback
        self.current_pose.header = feedback.current_pose.header
        self.current_pose.pose.pose = feedback.current_pose.pose
        self._navigation_time = feedback.navigation_time
        self._estimated_time_remaining = feedback.estimated_time_remaining
        self._number_of_recoveries = feedback.number_of_recoveries
        self._distance_remaining = feedback.distance_remaining

    def _goal_response_callback(self, future: Future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.node.get_logger().error(f"Goal rejected")
            return

        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future : Future):
        if self._goal_handle.status == GoalStatus.STATUS_CANCELED:
            self._navigation_completed = False
            return
        self._distance_remaining = 0.0
        self._navigation_completed = True


    def cancel_goal(self) -> ReturnFlag:
       if not self.is_robot_online():
            return ReturnFlag.ROBOT_OFFLINE
       if self._goal_handle is None:
            return ReturnFlag.UNINITIALIZED_GOAL_HANDLE
       future = self._goal_handle.cancel_goal_async()
       future.add_done_callback(self._cancel_response_callback)
       self._reset_navigation_data()
       return ReturnFlag.SUCCESS

    def _cancel_response_callback(self, future: Future):
       self._reset_navigation_data()

    def _reset_navigation_data(self):
        self._navigation_completed = False
        self._navigation_time = 0.0
        self._estimated_time_remaining = 0.0
        self._number_of_recoveries = 0
        self._distance_remaining = 0.0

    def get_navigation_completed(self) -> bool:
        return self._navigation_completed
    
    def get_distance_remaining(self):
        return self._distance_remaining
