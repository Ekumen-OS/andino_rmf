import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy import executors
from controller_action_msg.action import AndinoController
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from tf_transformations import euler_from_quaternion
import time
from typing import List

class AndinoControllerServer(Node):
    
    def __init__(self, node_name: str, *, context: rclpy.Context = None, cli_args: rclpy.List[str] = None, namespace: str = None, use_global_arguments: bool = True, enable_rosout: bool = True, start_parameter_services: bool = True, parameter_overrides: rclpy.List[rclpy.Parameter] = None, allow_undeclared_parameters: bool = False, automatically_declare_parameters_from_overrides: bool = False) -> None:
        super().__init__(node_name)
        rclpy.logging.set_logger_level(self.get_name(), LoggingSeverity.DEBUG)
        self._action_server = ActionServer(self,AndinoController,'andino_controller',self._execute_callback, cancel_callback=self._cancel_callback)
        # need to implement callback group as reentrant
        # odom topic
        self._odom_sub = self.create_subscription(Odometry,'/odom',self._odom_callback,10)
        self._odom_sub
        # velocity topic
        self.__vel_pub = self.create_publisher(Twist,'/cmd_vel',10)
        self.get_logger().info('Starting Andino Controller Server...')

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
        
        (_,_,goal_yaw) = self._quaternion_to_euler(quat)
        self.get_logger().info(f'[Goal] X: {goal_x} m | Goal Y: {goal_y} m | Goal Yaw:{goal_yaw} rad')
        
        self._go_to(goal_x, goal_y, quat, goal_handle, feedback_msg)
        #################################################

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return AndinoController.Result()
        
        goal_handle.succeed()
        # send result message
        result = AndinoController.Result()
        ##################################
        # result wrapper
        result.success = True
        ##################################
        return result

    def _cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info('Received cancel request :(')
        return CancelResponse.ACCEPT

    def _go_to(self, xg: float, yg: float, orientation: List, goal_handle: ServerGoalHandle, feedback):
        
        # set control parameters and states

        
        ###### Dummy Loop for testing receiving odom message##################
        count = 0
        while count <= 50:

            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal is canceled, stopping feedback loop.")
                
                # stop robot
                self.stop_robot()
                break

            feedback.current_pose.header.stamp = self.get_clock().now().to_msg()
            feedback.current_pose.pose.position.x = self._curr_x
            feedback.current_pose.pose.position.y = self._curr_y
            feedback.current_pose.pose.position.z = 0.0
            feedback.current_pose.pose.orientation.x = orientation[0]
            feedback.current_pose.pose.orientation.y = orientation[1]
            feedback.current_pose.pose.orientation.z = orientation[2]
            feedback.current_pose.pose.orientation.w = orientation[3]
            
            # current velocity
            feedback.current_vel.linear.x = 0.0
            feedback.current_vel.linear.y = 0.0
            feedback.current_vel.linear.z = 0.0

            (_,_,yaw) = self._quaternion_to_euler(orientation)
            self.get_logger().info(f'[Feedback] Current X: {feedback.current_pose.pose.position.x} m | Current Y: {feedback.current_pose.pose.position.y} m | Yaw:{yaw} rad')
            goal_handle.publish_feedback(feedback)
            
            count+= 1
            time.sleep(1)

    def _odom_callback(self,msg: Odometry):
        self._curr_x = msg.pose.pose.position.x
        self._curr_y = msg.pose.pose.position.y
        # orientation
        quat_tf = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (_, _, self._yaw) = self._quaternion_to_euler(quat_tf)

        self.get_logger().debug(f'[Odom] Current X: {self._curr_x} m | Current Y: {self._curr_y} m | Yaw:{self._yaw} rad')

    def _quaternion_to_euler(self, quaternion: List):
        (roll,pitch,yaw) = euler_from_quaternion(quaternion)
        return (roll,pitch,yaw)
    
    def _normalize_angle(self, theta):
        pass

    def _update_rho(self):
        rho = 0
        return rho
    
    def _update_alpha(self):
        alpha = 0
        return alpha
    
    def stop_robot(self):
        pass

def main(args=None):
    rclpy.init(args=args)

    andino_server = AndinoControllerServer('andino_controller_server')

    executor = executors.MultiThreadedExecutor()
    executor.add_node(andino_server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        andino_server.get_logger().info('KeyboardInterrupt. Shutting Down...')
    

if __name__== '__main__':
    main()