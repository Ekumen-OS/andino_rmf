'''
    The RobotHandler class manages individual robot instances within the fleet
    manager. It handles pose tracking, initial pose setting, navigation goal
    management, and communication with individual robots through ROS2 topics.
    Each robot in the fleet has its own RobotHandler instance that subscribes
    to the robot's pose updates, manages its state information, and will handle
    sending navigation goals and canceling active goals for the robot.
'''

import threading

from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import quaternion_from_euler


class RobotHandler:
    """
    Handles individual robot instances within the fleet manager.

    This class manages the state and communication for a single robot,
    including pose tracking, initial pose setting, navigation goal management,
    and subscribing to pose updates from the robot's AMCL localization system.
    It will also handle sending navigation goals to robots and canceling active
    goals when requested by the fleet manager.
    """

    def __init__(self, node: Node, robot_name: str, initial_pose: dict(), lock: threading.Lock):
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
        self.current_pose: PoseWithCovarianceStamped = None
        self._lock = lock

        self._set_initial_pose(robot_name, initial_pose)

        topic_name = "/" + robot_name + "/amcl_pose"
        self.pose_subscriber = self.node.create_subscription(
            PoseWithCovarianceStamped,
            topic_name,
            self._pose_callback,
            10,
        )

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
            self.current_pose = msg
