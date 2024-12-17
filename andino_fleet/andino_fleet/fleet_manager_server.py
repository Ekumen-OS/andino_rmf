import rclpy
import threading
import time


from rclpy import executors
from rclpy.node import Node
from rclpy.logging import set_logger_level, LoggingSeverity
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.service import SrvTypeRequest, SrvTypeResponse
from rclpy.action import ActionClient
from rclpy.task import Future


from andino_fleet_msg.srv import RobotControl, SendGoal, CancelGoal, RemoveAllGoals, RequestRobotPosition, InitPose

from controller_action_msg.action import AndinoController
from controller_action_msg.msg import RobotPose

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose

from collections import deque
from tf_transformations import quaternion_from_euler, euler_from_quaternion



class AndinoFleetManager(Node):
   def __init__(self, node_name: str = 'andino_fleet_manager', *, context: rclpy.Context = None, cli_args: rclpy.List[str] = None, namespace: str = None, use_global_arguments: bool = True, enable_rosout: bool = True, start_parameter_services: bool = True, parameter_overrides: rclpy.List[rclpy.Parameter] = None, allow_undeclared_parameters: bool = False, automatically_declare_parameters_from_overrides: bool = False) -> None:
       super().__init__(node_name)
       set_logger_level(self.get_name(), LoggingSeverity.INFO)
       # parameter for nav2 flag. Default value is false means using a custom controller
       self.declare_parameter('nav2', False)
       
       # define callback groups
       self._group1 = MutuallyExclusiveCallbackGroup()
       self._group2 = MutuallyExclusiveCallbackGroup()
       
       # define a server for setting robot control information for custom controllers
       self._add_goal_srv = self.create_service(RobotControl, 'add_goal_server', self._add_goal_callback, callback_group=self._group1)
       self._send_goal_srv = self.create_service(SendGoal, 'send_goal_server', self._send_goal_callback, callback_group=self._group1)
       self._cancel_goal_srv = self.create_service(CancelGoal, 'cancel_goal_server', self._cancel_goal_callback, callback_group=self._group1)
       self._remove_goal_srv = self.create_service(RemoveAllGoals, 'remove_goal_server', self._remove_goal_callback, callback_group=self._group1)
       self._robot_pose_srv = self.create_service(RequestRobotPosition, 'robot_pose_server', self._robot_pose_callback, callback_group=self._group1)
       
       # a robot_goals containing robot as key and goals as value
       self._robot_goals = dict() # self._robot_goals[robot] = [goal1, goal2, ...., goaln]
       # controller clients containing robot as key and client as value
       self._controller_clients = dict() # self._controller_clients[robot] = [client1, client2, ..., clientn]
       # goal handles
       self._goal_handles = dict()
       # pose subscriptions
       self._pose_subs = dict() # self._pose_subs[robot1] -> sub1
       # pose topics
       self._pose_topics = dict() # self._pose_topics[robot1] -> topic1
       # current pose
       self._current_poses = dict() # self._current_poses[robot1] -> [x1, y1, yaw1]
       # max linear velocity
       self._max_lin_velocity = 0.001
       # distance remaining
       self._distance_remainings = dict() # self._distance_remainings[robot1] -> curr_dist1
       # navigation result
       self._navigation_results = dict() # self._navigation_results[robot1] -> false
       
       # logging
       self._lock = threading.Lock()
       # get parameter
       self._nav2_enabled = self.get_parameter('nav2').get_parameter_value().bool_value
       # instantiate init_pose server and init_pose publishers if nav2 enabled
       if self._nav2_enabled:
           self._init_pose_srv = self.create_service(InitPose, 'initial_pose_server', self._init_pose_callback, callback_group=self._group1)
           self._initialpose_pubs = dict()
       
       if self._nav2_enabled:
           self.get_logger().info('Andino Fleet Manager Started with Nav2 Controller')
       else:
           self.get_logger().info('Andino Fleet Manager Started with Custom Controller')
       
  
   # callback process for adding goals to manager
   def _add_goal_callback(self, req: SrvTypeRequest, resp: SrvTypeResponse):
       # check if robot is online
       if self._check_robot_online(req.robot_name) is False:
           resp.success = False
           self.get_logger().info(f'{req.robot_name} is not online!')
           return resp
       # check if robot_name exists in collection
       if req.robot_name not in self._robot_goals:
           self._robot_goals[req.robot_name] = deque([req.final_pose])
           self.get_logger().debug(f'{req.robot_name} goal: {self._robot_goals[req.robot_name]}')
           # create new controller client
           self.create_client_and_subscription(req.robot_name)
           self.get_logger().info(f'Created new client for {req.robot_name}')
       else:
           self._robot_goals[req.robot_name].append(req.final_pose)
       resp.success = True
       self.get_logger().info(f'Added a final pose [{req.final_pose[0]}, {req.final_pose[1]}, {req.final_pose[2]}] for {req.robot_name}')
       return resp
   
   # callback process for sending goals
   def _send_goal_callback(self, req: SrvTypeRequest, resp: SrvTypeResponse):
       # check if the robot is online
       if self._check_robot_online(req.robot_name) is False:
           resp.result = False
           self.get_logger().info(f'Cannot send goal. {req.robot_name} is not online!')
           return resp
       # check if robot_name exists in collection
       if req.robot_name not in self._robot_goals:
           resp.result = False
           self.get_logger().info(f'{req.robot_name} does not exist in queue. Add this robot to the queue before sending the goal')
           return resp
       # send a goal to server
       self.send_goal(req.robot_name)
       self.get_logger().info(f'Sent a final pose for {req.robot_name}')
       resp.result = True
       return resp
    
    # callback process for canceling goals
   def _cancel_goal_callback(self, req: SrvTypeRequest, resp: SrvTypeResponse):
       # check if the robot is online
       if self._check_robot_online(req.robot_name) is False:
           resp.result = False
           self.get_logger().info(f'Cannot cancel goal. {req.robot_name} is not online!')
           return resp
       # check if robot_name exists in collection
       if req.robot_name not in self._robot_goals:
           resp.result = True
           self.get_logger().info(f'Cancel goal aborted. {req.robot_name} does not exist in queue')
           return resp
       
       self.cancel_goal(req.robot_name)
       resp.result = True
       return resp
   
   # callback process for remove goals
   def _remove_goal_callback(self, req: SrvTypeRequest, resp: SrvTypeResponse):
       # check if robot is online
       if self._check_robot_online(req.robot_name) is False:
           resp.result = False
           self.get_logger().info(f'{req.robot_name} is not online!')
           return resp 
       # check if robot_name exists in collection
       if req.robot_name not in self._robot_goals:
           resp.result = True
           self.get_logger().info(f'Remove goals aborted. {req.robot_name} is already empty.')
           return resp
       else:
           self._robot_goals[req.robot_name] = deque()
           resp.result = True
           return resp
    
   # callback process for requesting robot pose
   def _robot_pose_callback(self, req: SrvTypeRequest, resp: SrvTypeResponse):
       # check if robot is online
       if self._check_robot_online(req.robot_name) is False:
           resp.current_position = [0.0 ,0.0 ,0.0]
           resp.max_lin_velocity = 0.001
           resp.distance_remaining = 0.0
           resp.is_robot_connected = False
           resp.is_navigation_completed = False
           self.get_logger().info(f'{req.robot_name} is not online!')
           return resp
       # check if robot_name exists in collection
       if (req.robot_name not in self._pose_subs) or (req.robot_name not in self._controller_clients):
           # create new robot pose subscription and initialize feedback variables
           self.create_client_and_subscription(req.robot_name)
           self._distance_remainings[req.robot_name] = 0.0
       # get current robot pose
       with self._lock:
            resp.current_position = self._current_poses[req.robot_name]
            resp.max_lin_velocity = self._max_lin_velocity
            resp.distance_remaining = self._distance_remainings[req.robot_name]
            resp.is_robot_connected = True
            resp.is_navigation_completed = self._navigation_results[req.robot_name]
       return resp
   
   # callback process for publishing init pose
   def _init_pose_callback(self, req: SrvTypeRequest, resp: SrvTypeResponse):
       # publish init_pose topic to all robots
       robots = req.robot_names
       # define publisher
       for robot in robots:
           topic = '/'+robot+'/initialpose'
           self._initialpose_pubs[robot] = self.create_publisher(PoseWithCovarianceStamped, topic, 10)
           msg = PoseWithCovarianceStamped()
           msg.header.frame_id = 'map'
           
           match robot:
               case 'andino1':
                   msg.pose.pose.position.x = -2.1
                   msg.pose.pose.position.y = 4.7
               case 'andino2':
                   msg.pose.pose.position.x = 1.7
                   msg.pose.pose.position.y = 4.7
               case 'andino3':
                   msg.pose.pose.position.x = -2.1
                   msg.pose.pose.position.y = 1.4
               case 'andino4':
                   msg.pose.pose.position.x = 1.5
                   msg.pose.pose.position.y = 1.8    
               case _:
                   msg.pose.pose.position.x = 0.0
                   msg.pose.pose.position.y = 0.0
           
           self._initialpose_pubs[robot].publish(msg)
       self.get_logger().info('Initial poses are set!')
       resp.result = True
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
       
       if self._nav2_enabled:
           # create controller client for nav2 controller
           action_name = '/'+robot_name+'/navigate_to_pose'
           controller_client = ActionClient(self, NavigateToPose, action_name, callback_group=self._group1)
       else:
           # create controller client for custom controllers
           action_name = '/'+robot_name+'/andino_controller'
           controller_client = ActionClient(self, AndinoController, action_name, callback_group=self._group1)
       
       self._controller_clients[robot_name] = controller_client
       self.get_logger().info(f'Client for {action_name} created')

   # Create robot pose subscription given a robot name
   def _create_pose_subscription(self, robot_name: str):
       
       if self._nav2_enabled:
           # current position from amcl_pose estimation
           topic_name = '/'+robot_name+'/amcl_pose'
           sub_client = self.create_subscription(PoseWithCovarianceStamped, topic_name, self._pose_callback, 10, callback_group=self._group2)
       else:
           # create subscription client
           topic_name = '/'+robot_name+'/current_pose'
           sub_client = self.create_subscription(RobotPose, topic_name, self._pose_callback, 10, callback_group=self._group2)
       
       self._pose_topics[robot_name] = topic_name
       self._current_poses[robot_name] = [0.0 ,0.0 ,0.0]    
       time.sleep(0.2)
       self._pose_subs[robot_name] = sub_client
       self.get_logger().info(f'Subscription for {topic_name} created')
   
   # Wrapper method to create controller client and pose subscriber
   def create_client_and_subscription(self, robot_name: str):
       self._create_controller_client(robot_name=robot_name)
       self._create_pose_subscription(robot_name=robot_name)

       # initialize navigation result
       self._navigation_results[robot_name] = False
      
   # Send action goal given a robot name
   def send_goal(self, robot_name: str):
       if robot_name not in self._robot_goals:
           self.get_logger().info(f'Unable to send goal. {robot_name} controller server is unavailable!')
           return
       if not self._robot_goals[robot_name]:
           self.get_logger().info(f'No goals available for {robot_name}.')
           return
       # get a goal value
       goal = self._robot_goals[robot_name][0]
       self.get_logger().debug(f'Goal to send: [{goal[0]}, {goal[1]}, {goal[2]}]\n')
       self.get_logger().debug(f'Queue length: {len(self._robot_goals[robot_name])}')
       
       if self._nav2_enabled:
           # create goal msg for nav2 controller
           goal_msg = NavigateToPose.Goal()
           goal_msg.pose.header.frame_id = 'map'
           goal_msg.pose.pose.position.x = goal[0]
           goal_msg.pose.pose.position.y = goal[1]
           
           quaternion = quaternion_from_euler(0, 0, goal[2])
           orientation = Quaternion()
           orientation.x = quaternion[0]
           orientation.y = quaternion[1]
           orientation.z = quaternion[2]
           orientation.w = quaternion[3]
           goal_msg.pose.pose.orientation = orientation
           
       else:
           # create goal msg for custom controller
           goal_msg = AndinoController.Goal()
           goal_msg.goal_pose.pose.position.x = goal[0]
           goal_msg.goal_pose.pose.position.y = goal[1]
           quaternion = quaternion_from_euler(0, 0, goal[2])
           orientation = Quaternion()
           orientation.x = quaternion[0]
           orientation.y = quaternion[1]
           orientation.z = quaternion[2]
           orientation.w = quaternion[3]
           goal_msg.goal_pose.pose.orientation = orientation
       # send goal async
       if not self._controller_clients[robot_name].server_is_ready():
           self.get_logger().info(f'{robot_name} controller server is not ready!')
           return
       
       self._send_goal_future = self._controller_clients[robot_name].send_goal_async(goal_msg, feedback_callback= lambda feedback_msg: self._feedback_callback(robot_name, feedback_msg))
       self._send_goal_future.add_done_callback(lambda future: self._goal_response_callback(robot_name, future))
       # pop the goal
       self._robot_goals[robot_name].popleft()
       self.get_logger().debug(f'Removed the goal from queue. Queue length: {len(self._robot_goals[robot_name])}')
   
   # Cancel goal given a robot name
   def cancel_goal(self, robot_name: str):
       future = self._goal_handles[robot_name].cancel_goal_async()
       future.add_done_callback(lambda future: self._cancel_response_callback(robot_name, future))

   def _goal_response_callback(self, robot_name: str, future: Future):
       goal_handle = future.result()
       if not goal_handle.accepted:
           self.get_logger().info('Goal rejected :(')
           return
       
       self._goal_handles[robot_name] = goal_handle
       self.get_logger().info('Goal accepted :)')
       self._get_result_future = goal_handle.get_result_async()
       self._get_result_future.add_done_callback(lambda future: self._get_result_callback(robot_name, future))

   def _get_result_callback(self, robot_name: str, future: Future):
       result = future.result().result
       
       if self._nav2_enabled:
           # result from nav2 controller
           if self._goal_handles[robot_name].status == GoalStatus.STATUS_CANCELED:
               self._navigation_results[robot_name] = False
           else:
               self._navigation_results[robot_name] = True         
           self._distance_remainings[robot_name] = 0.0
           self.get_logger().info('Result: {0}'.format(result.result))
       else:
           # result from custom controller
           self._navigation_results[robot_name] = result.success
           self._distance_remainings[robot_name] = 0.0
           self.get_logger().info('Result: {0}'.format(result.success))

   def _feedback_callback(self, robot_name, feedback_msg):
       with self._lock:
            feedback = feedback_msg.feedback
            # get feedback and save it in class variables
            if self._nav2_enabled:
                self._max_lin_velocity = 0.4 # by-pass value
                self._distance_remainings[robot_name] = feedback.distance_remaining
            else:
                self._max_lin_velocity = feedback.max_lin_vel.linear.x
                self._distance_remainings[robot_name] = feedback.distance_remaining

       self.get_logger().debug(f'[{robot_name}] Distance Remaining: {round(self._distance_remainings[robot_name],3)}')

   def _cancel_response_callback(self, robot_name: str, future: Future):
       cancel_response = future.result()
       self._navigation_results[robot_name] = False
       self.get_logger().info('Goal successfully canceled :)')

   def _pose_callback(self, msg):
       with self._lock:
            for robot, topic in self._pose_topics.items():
                if self._nav2_enabled:
                    x = msg.pose.pose.position.x
                    y = msg.pose.pose.position.y
                    orientation_list = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
                    (r,p,yaw) = euler_from_quaternion(orientation_list)
                    self._current_poses[robot] = [x,y,yaw]
                else:
                    if msg.topic_name == topic:
                        self._current_poses[robot] = msg.current_pose
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
