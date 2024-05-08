import rclpy

from rclpy import executors
from rclpy.node import Node
from rclpy.logging import set_logger_level, LoggingSeverity
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.service import SrvTypeRequest, SrvTypeResponse
from rclpy.action import ActionClient

from fleet_msg.srv import RobotControl
from controller_action_msg.action import AndinoController

from collections import deque
from turtle_tf2_py.turtle_tf2_broadcaster import quaternion_from_euler

class AndinoFleetManager(Node):
    def __init__(self, node_name: str = 'andino_fleet_manager', *, context: rclpy.Context = None, cli_args: rclpy.List[str] = None, namespace: str = None, use_global_arguments: bool = True, enable_rosout: bool = True, start_parameter_services: bool = True, parameter_overrides: rclpy.List[rclpy.Parameter] = None, allow_undeclared_parameters: bool = False, automatically_declare_parameters_from_overrides: bool = False) -> None:
        super().__init__(node_name)
        set_logger_level(self.get_name(), LoggingSeverity.DEBUG)
        # define callback groups
        self._group1 = MutuallyExclusiveCallbackGroup()
        self._group2 = MutuallyExclusiveCallbackGroup()
        # define a server for setting robot control information
        self._info_server = self.create_service(RobotControl, 'robot_control_info_server', self._control_info_callback, callback_group=self._group1)
        # a queue to contain tuples of robot name and final pose
        self._robot_goals = deque()
        # controller clients set
        self._controller_clients = set()
        self._robots = set()
        # goal future
        self._send_goal_futures = []
        # result future
        self._get_result_futures = []
        # logging
        self.get_logger().info('Andino Fleet Manager Started')
    
    # callback process for querying requested action server
    def _control_info_callback(self, req: SrvTypeRequest, resp: SrvTypeResponse):
        self._robot_goals.append(tuple((req.robot_name, req.final_pose)))
        if rclpy.ok():
            resp.success = True
            self.get_logger().info(f'Received final pose [{req.final_pose[0]}, {req.final_pose[1]}, {req.final_pose[2]}] for {req.robot_name}')
        else:
            resp.success = False
            self.get_logger().info(f'Cannot set final pose [{req.final_pose[0]}, {req.final_pose[1]}, {req.final_pose[2]}] for {req.robot_name}')
        return resp
    
    # Check if the controlelr server available given a robot name
    def _check_robot_online(self, robot_name: str):
        # get the list of actions
        action_list = self.get_node_names_and_namespaces()
        for action in action_list:
            if robot_name in action[1]:
                return True
        return False
    
    # Create controller client given a robot name
    def create_controller_client(self, robot_name: str):
        if robot_name in self._robots:
            self.get_logger().info(f'Unable to create new client. {robot_name} client exists.')
            return
        action_name = '/'+robot_name+'/andino_controller'
        if self._check_robot_online(robot_name=robot_name):
            # create controller client
            controller_client = ActionClient(self, AndinoController, action_name, callback_group=self._group1)
            self._controller_clients.add(tuple(robot_name, controller_client))
            self._robots.add(robot_name)
            self.get_logger().info(f'Client for {action_name} created')
        else:
            self.get_logger().info(f'{action_name} server is unavailable!')

    # Send action goal given a robot name
    def send_goal(self):
        # while deque not empty
        while len(self._robot_goals) != 0:
            # 1. pop front goal from deque
            robot, goal = self._robot_goals.popleft()
            # 2. check whether robot is online
            if robot not in self._robots:
                self.get_logger().info(f'Unable to send goal. {robot} controller server is unavailable!')
                continue
            # send goal to appropriate controller server
            for client in self._controller_clients:
                if robot == client[0]:
                    goal_msg = AndinoController.Goal()
                    goal_msg.goal_pose.pose.position.x = goal[0]
                    goal_msg.goal_pose.pose.position.y = goal[1]
                    goal_msg.goal_pose.pose.orientation = quaternion_from_euler(0,0,goal[2])
                    
                    send_goal_future = client[1].send_goal_async(goal_msg, feedback=self._feedback_callback)
                    send_goal_future.add_done_callback(self._goal_response_callback)
                    self._send_goal_futures.append(tuple(robot, send_goal_future))
        self.get_logger().info(f'All goals in buffer sent.')

    def _goal_response_callback(self, future):
        pass

    def _get_result_callback(self, future):
        pass

    def _feedback_callback(self, feedback_msg):
        pass

def main(args=None):
    rclpy.init(args=args)

    fleet_manager = AndinoFleetManager()

    executor = executors.MultiThreadedExecutor()
    executor.add_node(fleet_manager)

    try:
        # test check robot online function
        print('Is andino1 available?', fleet_manager.check_robot_online('andino1'))
        executor.spin()
    except KeyboardInterrupt:
        fleet_manager.destroy_node()
        fleet_manager.get_logger().info('KeyboardInterrupt. Shutting Down...')
    

if __name__== '__main__':
    main()
    