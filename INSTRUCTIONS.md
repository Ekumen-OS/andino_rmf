# Fleet Management Implementation Guide with Open RMF and Andino

This guide provides instructions for implementing a fleet management system using Open RMF and the Andino robot platform with Nav2. The implementation consists of two core components: a Fleet Adapter and a Fleet Manager.

## 1. Fleet Adapter

The [Fleet Adapter](andino_fleet_adapter/) acts as the bridge between Open RMF and the Fleet Manager. For a detailed understanding of the Fleet Adapter, refer to the [README.md](andino_fleet_adapter/README.md) file within its corresponding folder.

To begin the implementation for ROS2 Humble, the Open RMF team provides a [fleet adapter template](https://github.com/open-rmf/fleet_adapter_template/tree/humble). This template includes the necessary communication with Open RMF, allowing you to focus on the specific implementation details.

As explained in the [fleet adapter template `README.md`](https://github.com/open-rmf/fleet_adapter_template/blob/b8632e442d9385d836a79bc6e58b9a870544d6a0/README.md), there are two primary areas to configure:

* Communication with the Fleet Manager
* Robot Fleet Configuration

### 1.1. Communication with the Fleet Manager

This implementation uses ROS2 services for communication between the Fleet Adapter and the Fleet Manager. The Fleet Adapter will have service clients for the following purposes:

*   **Get Position:** Retrieve the position and other relevant data for a specific robot from the Fleet Manager.
*   **Navigate:** Send a goal to a robot.
*   **Stop:** Cancel a robot's current goal.

### 1.2. Robot Fleet Configuration

The robot fleet configuration has three main sections:

*   **rmf_fleet:** Defines parameters for the RMF fleet, including the fleet name, robot limitations, and battery system information.
*   **robots:** Contains the configuration for each robot in the fleet. Each robot has a unique name (e.g., `andino1`) and its own set of parameters:
    *   **robot_config:** Robot-specific configurations, such as the maximum allowed delay.
    *   **rmf_config:** RMF-specific configurations, such as the robot's starting position and charger waypoint.
*   **reference_coordinates:** Used to compute transforms between the RMF and robot coordinate systems. This is defined by two matrices: one with the values of several points in the RMF system and another with the values of the same points in the robot's system.
    *   For this project, the spawning points of the four Andino robots were used. The RMF coordinate values were obtained using the [Traffic Editor](https://osrf.github.io/ros2multirobotbook/traffic-editor.html) tool. The robot coordinate values are taken from the spawning position of the robots in the Nav2 frame.

## 2. Fleet Manager

Once the Fleet Adapter is set up, the next step is to implement the [Fleet Manager](andino_fleet_manager/). This is the primary implementation for this project and has two main responsibilities:

*   Communication with the Fleet Adapter
*   Management of each robot

### 2.1. Communication with the Fleet Adapter

The Fleet Manager will have three main ROS2 service servers to:

*   **Get position:** Send the position and corresponding information about each robot.
*   **Receive goal:** Receive a goal from the adapter and send it to the robot.
*   **Cancel goal:** Cancel a robot's goal.

### 2.2. Robot Management

To communicate with the robots, the Fleet Manager uses a map that links robot names to a [`RobotHandler`](andino_fleet_manager/andino_fleet_manager/robot_handler.py) class. The robot name is used as the key because it is the unique identifier for each robot throughout the project, starting from the Fleet Adapter configuration.

The `RobotHandler` class will handle the communication for a specific robot and has two main attributes:

*   **Action to send and cancel goals:** Since we are using Andino robots with Nav2, the action is `/navigate_to_pose`.
*   **Topic to read pose from:** The pose is read from the `/amcl_pose` topic.