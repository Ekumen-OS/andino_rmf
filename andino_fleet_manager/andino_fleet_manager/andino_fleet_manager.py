import rclpy
from rclpy.node import Node
from rclpy.service import SrvTypeRequest, SrvTypeResponse

from andino_fleet_msg.srv import RobotControl, SendGoal, CancelGoal, RequestRobotPosition, RemoveAllGoals

class AndinoFleetManager(Node):

    def __init__(self):
        super().__init__('andino_fleet_manager_node')

        self._initialize_services()

    def _initialize_services(self):
        self._add_goal_srv = self.create_service(RobotControl, 'add_goal_server', self._add_goal_callback)
        self._send_goal_srv = self.create_service(SendGoal, 'send_goal_server', self._send_goal_callback)
        self._cancel_goal_srv = self.create_service(CancelGoal, 'cancel_goal_server', self._cancel_goal_callback)
        self._remove_goal_srv = self.create_service(RemoveAllGoals, 'remove_goal_server', self._remove_goal_callback)
        self._robot_pose_srv = self.create_service(RequestRobotPosition, 'robot_pose_server', self._robot_pose_callback)


    def _add_goal_callback(self, req: SrvTypeRequest, resp: SrvTypeResponse) -> SrvTypeResponse:
        self.get_logger().info(f'Adding a goal to {req.robot_name}')
        resp.success = True
        return resp

    def _send_goal_callback(self, req: SrvTypeRequest, resp: SrvTypeResponse) -> SrvTypeResponse:
       self.get_logger().info(f'Sending a goal to {req.robot_name}')
       resp.result = True
       return resp

    def _cancel_goal_callback(self, req: SrvTypeRequest, resp: SrvTypeResponse) -> SrvTypeResponse:
        self.get_logger().info(f'Canceling goal from {req.robot_name}')
        resp.result = True
        return resp

    def _remove_goal_callback(self, req: SrvTypeRequest, resp: SrvTypeResponse) -> SrvTypeResponse:
        self.get_logger().info(f'Removing all goals from {req.robot_name}')
        resp.result = True
        return resp

    def _robot_pose_callback(self, req: SrvTypeRequest, resp: SrvTypeResponse) -> SrvTypeResponse:
        self.get_logger().info(f'Getting the pose of {req.robot_name}')
        return resp

def main(args=None):
    rclpy.init(args=args)

    andino_fleet_manager = AndinoFleetManager()

    rclpy.spin(andino_fleet_manager)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
