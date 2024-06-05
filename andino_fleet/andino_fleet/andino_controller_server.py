import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy import executors
from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from tf_transformations import euler_from_quaternion
from typing import List

import rclpy

from controller_action_msg.action import AndinoController
from controller_action_msg.msg import RobotPose

class AndinoControllerServer(Node):
    
    def __init__(self, node_name: str = 'andino_controller_node', *, context: rclpy.Context = None, cli_args: rclpy.List[str] = None, namespace: str = None, use_global_arguments: bool = True, enable_rosout: bool = True, start_parameter_services: bool = True, parameter_overrides: rclpy.List[rclpy.Parameter] = None, allow_undeclared_parameters: bool = False, automatically_declare_parameters_from_overrides: bool = False) -> None:
        super().__init__(node_name)
        rclpy.logging.set_logger_level(self.get_name(), LoggingSeverity.INFO)
        self._action_server = ActionServer(self,AndinoController,'andino_controller',self._execute_callback, cancel_callback=self._cancel_callback)
        
        # declare controller tuning parameters
        self.declare_parameter('k_rho', 0.3)
        self.declare_parameter('k_alpha', 0.8)
        self.declare_parameter('k_beta', -0.15)
        self.declare_parameter('controller_frequency', 0.01)
        self.declare_parameter('distance_tolerance', 0.01)

        # odom topic
        self._odom_sub = self.create_subscription(Odometry,'/odom',self._odom_callback,50)
        # velocity topic
        self._vel_pub = self.create_publisher(Twist,'/cmd_vel',10)
        # current pose topic
        self._pose_pub = self.create_publisher(RobotPose, '/current_pose', 10)
        self._pose_topic = ''
        self.get_logger().info('Andino Controller Server Started')
    # This callback is called by the action server to execute tasks for a specific goal handle
    def _execute_callback(self,goal_handle: ServerGoalHandle):
        self.get_logger().info('Andino Start Moving...')
        ##################################################
        # controller implementation
        goal_msg = AndinoController.Goal()
        feedback_msg = AndinoController.Feedback()

        goal_msg.goal_pose = goal_handle.request.goal_pose
        goal_x = goal_msg.goal_pose.pose.position.x
        goal_y = goal_msg.goal_pose.pose.position.y
        quat = (goal_msg.goal_pose.pose.orientation.x, goal_msg.goal_pose.pose.orientation.y, goal_msg.goal_pose.pose.orientation.z, goal_msg.goal_pose.pose.orientation.w)
        
        (_,_,goal_yaw,_) = self._quaternion_to_euler(quat)
        self.get_logger().info(f'[Goal] X: {goal_x} m | Goal Y: {goal_y} m | Goal Yaw:{goal_yaw} rad')
        
        self._go_to(goal_x, goal_y, quat, goal_handle, feedback_msg)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return AndinoController.Result()
        
        goal_handle.succeed()
        # send result message
        result = AndinoController.Result()
        result.success = True
        return result

    def _cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info('Received cancel request :(')
        return CancelResponse.ACCEPT
    # This method implements the controller logic and is called by the execution callback given a goal request
    def _go_to(self, xg: float, yg: float, orientation: List, goal_handle: ServerGoalHandle, feedback):
        
        # set control parameters and states
        # get controller gains from parameters and frequency (k_rho, k_alpha, k_beta, freq, tolerance)
        k_rho = self.get_parameter('k_rho').get_parameter_value().double_value
        k_alpha = self.get_parameter('k_alpha').get_parameter_value().double_value
        k_beta = self.get_parameter('k_beta').get_parameter_value().double_value
        freq = self.get_parameter('controller_frequency').get_parameter_value().double_value
        tol = self.get_parameter('distance_tolerance').get_parameter_value().double_value

        # get error states (delta_x, delta_y, theta)
        dx,dy,theta = self._update_states(xg,yg)
        rho = self._update_rho(dx,dy)
        alpha = self._update_alpha(dx,dy,theta)
        beta = self._update_beta(alpha, theta)

        while rho > tol:

            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal is canceled, stopping feedback loop.")
                
                # stop robot
                self.stop_robot()
                break
            
            dx,dy,theta = self._update_states(xg,yg)
            rho = self._update_rho(dx,dy)
            alpha = self._update_alpha(dx,dy,theta)
            beta = self._update_beta(alpha, theta)
            # calculates the linear and angular velocity
            v = k_rho * rho
            w = k_alpha * alpha + k_beta * beta
            # defines the values of the feedback message
            feedback.current_pose.header.stamp = self.get_clock().now().to_msg()
            feedback.current_pose.pose.position.x = self._curr_x
            feedback.current_pose.pose.position.y = self._curr_y
            feedback.current_pose.pose.position.z = 0.0
            feedback.current_pose.pose.orientation.x = self._quat_tf[0]
            feedback.current_pose.pose.orientation.y = self._quat_tf[1]
            feedback.current_pose.pose.orientation.z = self._quat_tf[2]
            feedback.current_pose.pose.orientation.w = self._quat_tf[3]
            
            # current velocity
            feedback.current_vel.linear.x = v
            feedback.current_vel.linear.y = 0.0
            feedback.current_vel.linear.z = 0.0
            feedback.current_vel.angular.x = 0.0
            feedback.current_vel.angular.y = 0.0
            feedback.current_vel.angular.z = w
            feedback.distance_remaining = rho
            
            self.get_logger().info(f'[Feedback] Current X: {feedback.current_pose.pose.position.x} m | Current Y: {feedback.current_pose.pose.position.y} m | Yaw:{self._yaw} rad')
            goal_handle.publish_feedback(feedback)

            self.move_robot(v,w)
            time.sleep(freq)
            
        self.stop_robot()
            
    def _odom_callback(self,msg: Odometry):
        self._curr_x = msg.pose.pose.position.x
        self._curr_y = msg.pose.pose.position.y
        # orientation
        quat_tf = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (_, _, self._yaw, self._quat_tf) = self._quaternion_to_euler(quat_tf)

        # publish current pose
        self.publish_pose()
        self.get_logger().debug(f'[Odom] Current X: {self._curr_x} m | Current Y: {self._curr_y} m | Yaw:{self._yaw} rad')

    def _quaternion_to_euler(self, quaternion: List):
        (roll,pitch,yaw) = euler_from_quaternion(quaternion)
        return (roll,pitch,yaw,quaternion)
    
    def _normalize_angle(self, theta):
        '''
        Normalize theta(radian) to be between (-pi,pi]
        '''
        norm_theta = theta
        while norm_theta < (-1) * math.pi: norm_theta += 2*math.pi
        while norm_theta >= math.pi: norm_theta -= 2*math.pi
        return norm_theta

    def _update_rho(self, x: float, y: float):
        rho = math.sqrt(x**2 + y**2)
        return rho
    
    def _update_alpha(self, x: float, y: float, theta: float):
        alpha = (-1)*theta + math.atan2(y,x)
        return self._normalize_angle(alpha)
    
    def _update_beta(self, alpha:float, theta: float):
        beta = (-1)*theta - alpha
        return self._normalize_angle(beta)
    
    def _update_states(self, x_goal : float, y_goal: float):
        delta_x = x_goal - self._curr_x
        delta_y = y_goal - self._curr_y
        theta = self._yaw
        return (delta_x,delta_y,theta)
    
    def _update_topic_names(self):
        topic_names_and_types = self.get_topic_names_and_types()
        namespace = self.get_namespace()
        topic = namespace+'/current_pose'
        for name, type in topic_names_and_types:
            if topic in name:
                self._pose_topic = name
    
    def stop_robot(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0

        self._vel_pub.publish(vel_msg)

    def move_robot(self, linear_twist: float , angular_twist: float):
        vel_msg = Twist()
        vel_msg.linear.x = linear_twist
        vel_msg.angular.z = angular_twist

        self._vel_pub.publish(vel_msg)

    def publish_pose(self):
        self._update_topic_names()
        pose_msg = RobotPose()
        pose_msg.topic_name = self._pose_topic
        pose_msg.current_pose = [self._curr_x, self._curr_y, self._yaw]

        self._pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)

    andino_server = AndinoControllerServer('andino_controller_server')

    executor = executors.MultiThreadedExecutor()
    executor.add_node(andino_server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        andino_server.destroy_node()
        andino_server.get_logger().info('KeyboardInterrupt. Shutting Down...')
    
if __name__== '__main__':
    main()
