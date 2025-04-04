import threading
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.service import SrvTypeRequest, SrvTypeResponse

from andino_fleet_msg.srv import RobotControl, SendGoal, CancelGoal, RequestRobotPosition, RemoveAllGoals
from andino_fleet_manager.andino_proxy import AndinoProxy, AndinoProxyFactory, ControllerType

from pathlib import Path
import yaml

class AndinoFleetManager(Node):

    def __init__(self):
        super().__init__('andino_fleet_manager_node')

        self._lock = threading.Lock()

        # Get the controller parameter
        self.declare_parameter('controller', 'andino')
        controller = self.get_parameter('controller').get_parameter_value().string_value
        self.get_logger().info(f'Received controller: {controller}')

        # define callback groups
        self._robot_controller_callback_group = MutuallyExclusiveCallbackGroup()
        self._robot_pose_callback_group = MutuallyExclusiveCallbackGroup()
        self._adapter_servers_callback_group = MutuallyExclusiveCallbackGroup()

        # Get the robots configuration file parameter
        self.declare_parameter('robots_config_file', '')
        robots_config_file = self.get_parameter('robots_config_file').get_parameter_value().string_value
        self.get_logger().info(f'Received robots configuration file: {robots_config_file}')

        robot_data = self._load_robot_configurations(Path(robots_config_file))
        self.get_logger().info(f"Loaded robot configurations: {robot_data}")

        self._robot_proxies_map : dict[AndinoProxy] = {}

        for robot_name, _ in robot_data.items():
            # Create an AndinoProxy for each robot
            self.get_logger().info(f'Creating proxy for {robot_name}')
            controller_type = ControllerType(controller)
            self._robot_proxies_map[robot_name] = AndinoProxyFactory.create_andino_proxy(
                controller_type, robot_name, self._robot_controller_callback_group, self._robot_pose_callback_group
            )

        self._initialize_services()

    def _load_robot_configurations(self, yaml_file_path: Path) -> dict:
        try:
            with yaml_file_path.open('r') as f:
                return yaml.safe_load(f)
        except FileNotFoundError:
            self.get_logger().error(f"YAML configuration file not found: {yaml_file_path}")
            return {}
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error parsing YAML file: {e}")
            return {}

    def _initialize_services(self):
        self._add_goal_srv = self.create_service(RobotControl, 'add_goal_server', self._add_goal_callback)
        self._send_goal_srv = self.create_service(SendGoal, 'send_goal_server', self._send_goal_callback)
        self._cancel_goal_srv = self.create_service(CancelGoal, 'cancel_goal_server', self._cancel_goal_callback)
        self._remove_goal_srv = self.create_service(RemoveAllGoals, 'remove_goal_server', self._remove_goal_callback)
        self._robot_pose_srv = self.create_service(RequestRobotPosition, 'robot_pose_server', self._robot_pose_callback)


    def _add_goal_callback(self, req: SrvTypeRequest, resp: SrvTypeResponse) -> SrvTypeResponse:
        self.get_logger().info(f'Received a request to add a goal to {req.robot_name}')
        if self._validate_robot(req.robot_name) is False:
           resp.success = False
           return resp
        # Send a goal to the controller server
        self.get_logger().info(f'Adding a goal to {req.robot_name}')
        self._robot_proxies_map[req.robot_name].add_goal(req.final_pose)

        resp.success = True
        return resp


    def _send_goal_callback(self, req: SrvTypeRequest, resp: SrvTypeResponse) -> SrvTypeResponse:
       self.get_logger().info(f'Received a request to send a goal to {req.robot_name}')
       if self._validate_robot(req.robot_name) is False:
           resp.result = False
           return resp
       # Send a goal to the controller server
       self.get_logger().info(f'Sending a goal to {req.robot_name}')
       self._robot_proxies_map[req.robot_name].send_goal()

       resp.result = True
       return resp

    def _cancel_goal_callback(self, req: SrvTypeRequest, resp: SrvTypeResponse) -> SrvTypeResponse:
        self.get_logger().info(f'Received a request to cancel a goal from {req.robot_name}')
        if self._validate_robot(req.robot_name) is False:
           resp.result = False
           return resp
        self.get_logger().info(f'Canceling goal from {req.robot_name}')
        self._robot_proxies_map[req.robot_name].cancel_goal()
        resp.result = True
        return resp

    def _remove_goal_callback(self, req: SrvTypeRequest, resp: SrvTypeResponse) -> SrvTypeResponse:
        self.get_logger().info(f'Received a request to remove all goals from {req.robot_name}')
        if self._validate_robot(req.robot_name) is False:
           resp.result = False
           return resp
        self.get_logger().info(f'Removing all goals from {req.robot_name}')
        self._robot_proxies_map[req.robot_name].remove_all_goals()
        resp.result = True
        return resp

    def _robot_pose_callback(self, req: SrvTypeRequest, resp: SrvTypeResponse) -> SrvTypeResponse:
        # self.get_logger().info(f'Received a request to get the pose of {req.robot_name}')
        with self._lock:
            # self.get_logger().info(f'Pose callback: Lock acquired for {req.robot_name}')
            if self._check_robot_existence(req.robot_name) is False:
                self.get_logger().info(f'No robot info for {req.robot_name}')
                resp.current_position = [0.0, 0.0, 0.0]
                resp.max_lin_velocity = 2.0
                resp.remaining_distance = 0.0
                resp.is_robot_connected = False
                resp.is_navigation_completed = False
                return resp
            # self.get_logger().info(f'Getting pose of {req.robot_name}')
            robot = self._robot_proxies_map[req.robot_name]
            resp.current_position = robot.current_pose
            resp.max_lin_velocity = robot.max_lin_velocity
            resp.remaining_distance = robot.remaining_distance
            resp.is_robot_connected = True
            resp.is_navigation_completed = robot.navigation_completed
        # self.get_logger().info(f'Pose callback: Lock released for {req.robot_name}')
        return resp


    def _check_robot_existence(self, robot_name: str) -> bool:
        # Check if robot_name exists in collection
        if robot_name not in self._robot_proxies_map.keys():
            self.get_logger().info(f'{robot_name} does not exist in the fleet.')
            return False
        return True

    def _validate_robot(self, robot_name: str) -> bool:
        # Check if the robot is online
       if self._robot_proxies_map[robot_name].check_robot_availability() is False:
           self.get_logger().info(f'Cannot send goal. {robot_name} is not online!')
           return False
       # Check if robot_name exists in collection
       return self._check_robot_existence(robot_name)

def main(args=None):
    rclpy.init(args=args)

    andino_fleet_manager = AndinoFleetManager()

    rclpy.spin(andino_fleet_manager)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
