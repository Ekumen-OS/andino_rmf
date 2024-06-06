# Andino Fleet Package
This package consists of the implementation of a controller server and a fleet manager for Andino robots.

## Controller Server
### Summary

The goal of a controller server is to move an Andino robot from position A to B considering a feedback control loop. By specifying a goal position (x, y, yaw), the robot is able to move to that position. The theory behind this controller is obtained from *Introduction to Autonomous Mobile Robots by Roland Siegwart and Illah R. Nourbakhsh* 

### Implementation
The controller is implemented as a ROS2 node that creates an action server for a specific andino bot. This action server takes clients requests that specify a Pose as it's goal, send the position information as the action feedback and returns True or False whether the bot arrived at the goal or not due to it's movement being completed or a goal being canceled.
<img src="https://github.com/ekumenlabs/andino_fleet_open_rmf/blob/main/docs/controller_server_diagram.png" alt="controller diagram" title="controller diagram" width="650">

In order for a controller to function properly, the following functionalities are implemented.

- Be able to process a goal request, feedback message, result response, and cancel goal request following [ROS2 action server design standard](https://design.ros2.org/articles/actions.html)
- Use [custom action message](https://github.com/ekumenlabs/andino_fleet_open_rmf/tree/main/controller_action_msg/action) for this controller


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

<img src="https://github.com/ekumenlabs/andino_fleet_open_rmf/blob/main/docs/fleet_manager_diagram.png" alt="fleet manager diagram" title="fleet manager diagram" width="750">

The fleet manager node have the following features implemented.

- Be able to implement relevant services to manage the andino fleet
- Use [custom service messages](https://github.com/ekumenlabs/andino_fleet_open_rmf/tree/main/fleet_msg/srv) for service interface
- Be able to get states of each robot
