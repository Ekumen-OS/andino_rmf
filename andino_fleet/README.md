# Andino Fleet Package
This package consists of the implementation of a controller server and a fleet manager for Andino robots.

## Controller Server
### Summary
The goal of a controller server is to move an Andino robot from position A to B considering a feedback control loop. By specifying a goal position (x, y, yaw), the robot is able to move to that position. The theory behind this controller is obtained from *Introduction to Autonomous Mobile Robots by Roland Siegwart and Illah R. Nourbakhsh* 

### Implementation
The controller is implemented as a ROS2 node that creates an action server for a specific andino bot. This action server takes clients requests that specify a Pose as it's goal, send the position information as the action feedback and returns True or False whether the bot arrived at the goal or not due to it's movement being completed or a goal being canceled.
<img src="../resources/controller_server_diagram.png" alt="controller diagram" title="controller diagram" width="650">

In order for a controller to function properly, the following functionalities are implemented.

- Be able to process a goal request, feedback message, result response, and cancel goal request following [ROS2 action server design standard](https://design.ros2.org/articles/actions.html)
- Use [custom action message](https://github.com/ekumenlabs/andino_fleet_open_rmf/tree/main/controller_action_msg/action) for this controller

## Nav2 Controller Server
### Summary
Users can choose whether to use the custom controller (as above section), or Nav2 controller server integration for general perposes. Thanks to [andino_gz](https://github.com/Ekumen-OS/andino_gz/tree/humble?tab=readme-ov-file#compass-navigation) package, Nav2 controller is integrated and is ready to be used. The package facilitates users by having a flag that enables Nav2 features. This package intends to be a wrapper to launch Nav2-enabled controller(s).

### Implementation
<img src="../resources/nav2_controller_diagram.png" alt="nav2 diagram" title="nav2 diagram" width="650">

## Fleet Manager
### Summary

The goal of a fleet manager is to manage multiple robots so that it is able to send commands and receive information from each robot. 

### Implementation
The manager is implemented as a ROS2 node that contains multiple services to control the andino fleet and monitor the robot states. These services include
- add a goal to the manager
- send a goal to the manager
- cancel a current goal
- remove all goals in queue
- read a robot position

Each service requires a robot name in order to manage individual robots.

<img src="../resources/fleet_manager_diagram.png" alt="fleet manager diagram" title="fleet manager diagram" width="750">

The fleet manager node have the following features implemented.

- Be able to implement relevant services to manage the andino fleet
- Use [custom service messages](https://github.com/ekumenlabs/andino_fleet_open_rmf/tree/main/andino_fleet_msg/srv) for service interface
- Be able to get states of each robot

## Usage
To launch multiple robots with corresponding controller servers,

```
ros2 launch andino_fleet spawn_multiple_robot.launch.py
```

<img src="../resources/multi_robot.png" alt="Multi-robot simulation" title="Multi-robot simulation" width="300"/>

*<b>Note: </b> To add/remove robot(s), edit <b>spawn_robots.yaml</b> under <b>[andino_fleet/config](https://github.com/ekumenlabs/andino_fleet_open_rmf/tree/main/andino_fleet/config)</b> folder. There are two robots by default.*

To run the implemented fleet manager,

```
ros2 run andino_fleet fleet_manager
```

After the fleet manager node is running, it allows users to interact with the robot fleet as the following.

### Add a goal
Add a goal to the manager for a specific robot, given the robot name and the final pose,

```
ros2 service call /add_goal_server andino_fleet_msg/srv/RobotControl "{robot_name: 'andino2', final_pose: [0.1,0,0]}"
```

### Send a goal
Start moving a robot by sending a goal to the manager, assuming that the goal is already added,

```
ros2 service call /send_goal_server andino_fleet_msg/srv/SendGoal "{robot_name: 'andino2'}"
```

### Cancel a goal
Once a goal is being executed, users can cancel the goal given a robot name by,

```
ros2 service call /cancel_goal_server andino_fleet_msg/srv/CancelGoal "{robot_name: 'andino2'}"
```

### Remove all goals
Users can remove all goals in fleet manager that belong to a specified robot,

```
ros2 service call /remove_goal_server andino_fleet_msg/srv/RemoveAllGoals "{robot_name: 'andino2'}"
```

### Request for current states
Users can retrieve a robot states as the following
- robot position defined by [x, y, yaw]
- robot connectivity
- navigation status

```
ros2 service call /robot_pose_server andino_fleet_msg/srv/RequestRobotPosition "{robot_name: 'andino2'}"
```