import rclpy
import rclpy.executors as executors
from rclpy.node import Node
from rclpy.service import SrvTypeRequest, SrvTypeResponse

from andino_fleet_msg.srv import (
    RobotControl,
    SendGoal,
    CancelGoal,
    RequestRobotPosition,
    RemoveAllGoals,
)


class AndinoFleetManager(Node):
    def __init__(self, node_name: str = "andino_fleet_manager"):
        super().__init__(node_name)
        self._add_goal_client = self.create_service(
            RobotControl, "/add_goal_server", self._add_goal_callback
        )
        self._send_goal_client = self.create_service(
            SendGoal, "/send_goal_server", self._send_goal_callback
        )
        self._cancel_goal_client = self.create_service(
            CancelGoal, "/cancel_goal_server", self._cancel_goal_callback
        )
        self._remove_goal_client = self.create_service(
            RemoveAllGoals, "/remove_goal_server", self._remove_goals_callback
        )
        self._robot_state_client = self.create_service(
            RequestRobotPosition, "/robot_pose_server", self._robot_pose_callback
        )

        self.get_logger().info('Andino Fleet Manager Started')

    def _add_goal_callback(self, request: SrvTypeRequest, response: SrvTypeResponse):
        self.get_logger().info(
            f"Add goal [{request.final_pose[0]}, {request.final_pose[1]}, {request.final_pose[2]}] for robot {request.robot_name}")

        response.success = True
        return response

    def _send_goal_callback(self, request: SrvTypeRequest, response: SrvTypeResponse):
        self.get_logger().info(
            f"Send robot {request.robot_name} to goal")

        response.result = True
        return response

    def _cancel_goal_callback(self, request: SrvTypeRequest, response: SrvTypeResponse):
        self.get_logger().info(
            f"Cancelling goal for robot {request.robot_name}")

        response.result = True
        return response

    def _remove_goals_callback(self, request: SrvTypeRequest, response: SrvTypeResponse):
        self.get_logger().info(
            f"Removing all goals for robot {request.robot_name}")

        response.result = True
        return response

    def _robot_pose_callback(self, request: SrvTypeRequest, response: SrvTypeResponse):
        self.get_logger().info(
            f"Getting pose for robot {request.robot_name}")

        response.current_position = [1.0, 2.0, 0.0]
        response.max_lin_velocity = 1.0
        response.distance_remaining = 5.0
        response.is_robot_connected = True
        response.is_navigation_completed = True
        return response


def main():
    rclpy.init()

    fleet_manager = AndinoFleetManager()
    executor = executors.MultiThreadedExecutor()
    executor.add_node(fleet_manager)

    try:
        executor.spin()
    except KeyboardInterrupt:
        fleet_manager.destroy_node()
        fleet_manager.get_logger().info('KeyboardInterrupt. Shutting Down...')


if __name__ == "__main__":
    main()
