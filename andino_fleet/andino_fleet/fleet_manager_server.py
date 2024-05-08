import rclpy
from fleet_msg.srv import RobotControl
from rclpy import executors
from rclpy.node import Node
from rclpy.logging import set_logger_level, LoggingSeverity
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.service import SrvTypeRequest, SrvTypeResponse

from collections import deque

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
    def check_robot_online(self, robot_name: str):
        # get the list of actions
        action_list = self.get_node_names_and_namespaces()
        for action in action_list:
            if robot_name in action[1]:
                return True
        return False

def main(args=None):
    rclpy.init(args=args)

    fleet_manager = AndinoFleetManager()

    executor = executors.MultiThreadedExecutor()
    executor.add_node(fleet_manager)

    try:
        executor.spin()
    except KeyboardInterrupt:
        fleet_manager.destroy_node()
        fleet_manager.get_logger().info('KeyboardInterrupt. Shutting Down...')
    

if __name__== '__main__':
    main()
    