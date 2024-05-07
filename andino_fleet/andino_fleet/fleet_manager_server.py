import rclpy
from fleet_msg.srv import QueryController
from rclpy import executors
from rclpy.node import Node
from rclpy.logging import set_logger_level, LoggingSeverity
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.service import SrvTypeRequest, SrvTypeResponse

class AndinoFleetManager(Node):
    def __init__(self, node_name: str = 'andino_fleet_manager', *, context: rclpy.Context = None, cli_args: rclpy.List[str] = None, namespace: str = None, use_global_arguments: bool = True, enable_rosout: bool = True, start_parameter_services: bool = True, parameter_overrides: rclpy.List[rclpy.Parameter] = None, allow_undeclared_parameters: bool = False, automatically_declare_parameters_from_overrides: bool = False) -> None:
        super().__init__(node_name)
        set_logger_level(self.get_name(), LoggingSeverity.DEBUG)
        # define callback groups
        self._group1 = MutuallyExclusiveCallbackGroup()
        self._group2 = MutuallyExclusiveCallbackGroup()
        # define a server for querying action server
        self._query_action = self.create_service(QueryController, 'query_action_server', self._query_action_callback, callback_group=self._group1)
        
        # logging
        self.get_logger().info('Andino Fleet Manager Started')
    # callback process for querying requested action server
    def _query_action_callback(self, req: SrvTypeRequest, resp: SrvTypeResponse):
        self.get_logger().info(f'Querying controller server for /{req.robot_name}/{req.action_server_name}')

        namespace = '/' + req.robot_name
        node_name = req.action_server_name
        # get the list of actions
        node_list = self.get_node_names_and_namespaces()
        
        for node in node_list:
            if node_name in node[0]:
                if namespace in node[1]:
                    resp.available = True
                    return resp

        resp.available = False
        return resp
    

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
    