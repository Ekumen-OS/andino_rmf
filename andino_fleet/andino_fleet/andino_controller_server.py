import rclpy
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from controller_action_msg.action import AndinoController
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from tf_transformations import euler_from_quaternion

class AndinoControllerServer(Node):
    
    def __init__(self, node_name: str, *, context: rclpy.Context = None, cli_args: rclpy.List[str] = None, namespace: str = None, use_global_arguments: bool = True, enable_rosout: bool = True, start_parameter_services: bool = True, parameter_overrides: rclpy.List[rclpy.Parameter] = None, allow_undeclared_parameters: bool = False, automatically_declare_parameters_from_overrides: bool = False) -> None:
        super().__init__('andino_controller_server')
        self._action_server = ActionServer(self,AndinoController,'andino_controller',self.execute_callback)

        # odom topic
        self.__odom_sub = self.create_subscription(Odometry,'/odom',self.__odom_callback,10)
        self.__odom_sub
        # velocity topic
        self.__vel_pub = self.create_publisher(Twist,'/cmd_vel',10)

    def execute_callback(self,goal_handle: ServerGoalHandle):
        self.get_logger().info('Andino Start Moving...')
        ##################################################
        # controller implementation

        #################################################

        goal_handle.succeed()

        # send result message
        result = AndinoController.Result()
        ##################################
        # result wrapper

        ##################################
        return result

    def __odom_callback(self,msg: Odometry):
        self._curr_x = msg.pose.pose.position.x
        self._curr_y = msg.pose.pose.position.y
        # orientation
        quat_tf = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, self._yaw) = euler_from_quaternion(quat_tf)

        self.get_logger().debug(f'Current X: {self._curr_x} m | Current Y: {self._curr_y} m | Yaw:{self._yaw} rad')

def main(args=None):
    rclpy.init(args=args)

    andino_server = AndinoControllerServer()

    rclpy.spin(andino_server)

if __name__== '__main__':
    main()