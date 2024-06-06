# Andino Fleet Package
This package consists of the implementation of a controller server and a fleet manager for Andino robots.

## Controller Server
### Summary

The goal of a controller server is to move an Andino robot from position A to B considering a feedback control loop. By specifying a goal position (x, y, yaw), the robot is able to move to that position. The theory behind this controller is obtained from *Introduction to Autonomous Mobile Robots by Roland Siegwart and Illah R. Nourbakhsh* 

### Implementation

<img src="https://github.com/ekumenlabs/andino_fleet_open_rmf/blob/readme-writing/docs/controller_server_diagram.png" alt="controller diagram" title="controller diagram" width="500">

In order for a controller to function properly, the following functionalities are to be implemented.

- Be able to process a goal request, feedback message, result response, and cancel goal request following [ROS2 action server design standard](https://design.ros2.org/articles/actions.html)
- Use [custom action message](https://github.com/ekumenlabs/andino_fleet_open_rmf/tree/readme-writing/controller_action_msg/action) for this controller


## Fleet Manager
### Summary

The goal of a fleet manager is to manage multiple robots so that it is able to send commands and receive information from each robot. 

### Implementation

<img src="https://github.com/ekumenlabs/andino_fleet_open_rmf/blob/readme-writing/docs/fleet_manager_diagram.png" alt="fleet manager diagram" title="fleet manager diagram" width="500">

The followings are design consideration to be used inside the fleet manager node.

- Be able to add, send, and cancel a goal to individual robots
- Multiple ROS2 services are used in order to add/send/cancel a goal
- Use [custom service messages]([/home/ubuntu/andino_fleet_open_rmf/src/andino_fleet_open_rmf/fleet_msg/srv](https://github.com/ekumenlabs/andino_fleet_open_rmf/tree/readme-writing/fleet_msg/srv)) for adding/sending/canceling a goal
- Be able to get a current position of each robot
