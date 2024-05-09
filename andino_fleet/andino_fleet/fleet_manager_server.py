import rclpy

from rclpy import executors
from rclpy.node import Node
from rclpy.logging import set_logger_level, LoggingSeverity
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.service import SrvTypeRequest, SrvTypeResponse
from rclpy.action import ActionClient
from rclpy.task import Future

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
        # a robot_goals containing robot as key and goals as value
        self._robot_goals = dict() # self._robot_goals[robot] = [goal1, goal2, ...., goaln]
        # controller clients containing robot as key and client as value
        self._controller_clients = dict() # self._controller_clients[robot] = [client1, client2, ..., clientn]
        # logging
        self.get_logger().info('Andino Fleet Manager Started')
    
    # callback process for querying requested action server
    def _control_info_callback(self, req: SrvTypeRequest, resp: SrvTypeResponse):
        # check if robot is online
        if self._check_robot_online(req.robot_name) is False:
            resp.success = False
            self.get_logger().info(f'Cannot set final pose [{req.final_pose[0]}, {req.final_pose[1]}, {req.final_pose[2]}] for {req.robot_name}')
            return resp
        # check if robot_name exists in collection
        if req.robot_name not in self._robot_goals:
            self._robot_goals[req.robot_name] = deque()
            # create new controller client
            self._create_controller_client(req.robot_name)
        else:
            self._robot_goals[req.robot_name].append(req.final_pose)

        resp.success = True
        self.get_logger().info(f'Received final pose [{req.final_pose[0]}, {req.final_pose[1]}, {req.final_pose[2]}] for {req.robot_name}')
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
    def _create_controller_client(self, robot_name: str):
        action_name = '/'+robot_name+'/andino_controller'
        # create controller client
        controller_client = ActionClient(self, AndinoController, action_name, callback_group=self._group1)
        self._controller_clients[robot_name] = controller_client
        self.get_logger().info(f'Client for {action_name} created')
        
    # Send action goal given a robot name
    def send_goal(self, robot_name: str):
        if robot_name not in self._robot_goals:
            self.get_logger().info(f'Unable to send goal. {robot_name} controller server is unavailable!')
            return
        # pop left goal 
        goal = self._robot_goals[robot_name].popleft()
        # create goal msg
        goal_msg = AndinoController.Goal()
        goal_msg.goal_pose.pose.position.x = goal[0]
        goal_msg.goal_pose.pose.position.y = goal[1]
        goal_msg.goal_pose.pose.orientation = quaternion_from_euler(0,0,goal[2])
        # send goal async
        if not self._controller_clients[robot_name].server_is_ready():
            self.get_logger().info(f'{robot_name} controller server is not ready!')
            return
        self._send_goal_future = self._controller_clients[robot_name].send_goal_async(goal_msg, feedback=self._feedback_callback)
        self._send_goal_future.add_done_callback(self._goal_response_callback)
        
    def _goal_response_callback(self, future: Future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future: Future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.success))

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Distance Remaining: {0}'.format(feedback.distance_remaining))

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
    