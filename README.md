# Andino Fleet with OpenRMF
This package intends to utilize OpenRMF to send a high-level task planning to Andino robot fleet. To achieve this goal, three main developments are included as the following.
<ol>
  <li>[Controller server](/home/ubuntu/andino_fleet_open_rmf/src/andino_fleet_open_rmf/andino_fleet/README.md) for an Andino robot</li>
  <li>[Fleet manager](/home/ubuntu/andino_fleet_open_rmf/src/andino_fleet_open_rmf/andino_fleet/README.md) to manage multiple robots</li>
  <li>Fleet adapter to bridge a fleet manager with OpenRMF API</li>
</ol>

## Project dependencies
- <b>ROS 2</b>: Humble Hawksbill
- <b>OS</b>: Ubuntu 22.04 Jammy Jellyfish
- <b>Simulation</b>: 
    - [andino_gz](https://github.com/Ekumen-OS/andino_gz/tree/humble?tab=readme-ov-file): a simulation environment for [Andino](https://github.com/Ekumen-OS/andino) in Gazebo Fortress
- <b>OpenRMF</b>

## Build
1. To build this package, ensure that ROS workspace is already in your system. If not, then create a workspace

```sh
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Clone this repository in ```src``` folder

```
https://github.com/ekumenlabs/andino_fleet_open_rmf.git
```

3. Install this package dependencies

```
rosdep install --from-paths src -i -y
```

4. Build this package and source the install workspace

```sh
colcon build
source install/setup.bash
```

## Usage
To launch multiple robots with corresponding controller servers,

```
ros2 launch andino_fleet spawn_multiple_robot.launch.py
```

<img src="https://github.com/ekumenlabs/andino_fleet_open_rmf/blob/readme-writing/docs/Screenshot%20from%202024-05-21%2010-18-39.png" alt="Multi-robot simulation" title="Multi-robot simulation" width="300"/>

*<b>Note: </b> To add/remove robot(s), edit <b>spawn_robots.yaml</b> under <b>[andino_fleet/config](https://github.com/ekumenlabs/andino_fleet_open_rmf/tree/readme-writing/andino_fleet/config)</b> folder. There are two robots by default.*

To run the implemented fleet manager,

```
ros2 run andino_fleet fleet_manager
```

After the fleet manager node is running, it allows users to interact with the robot fleet as the following.

### Add a goal
Add a goal to the manager for a specific robot, given the robot name and the final pose,

```
ros2 service call /add_goal_server fleet_msg/srv/RobotControl "{robot_name: 'andino2', final_pose: [0.1,0,0]}"
```

### Send a goal
Start moving a robot by sending a goal to the manager, assuming that the goal is already added,

```
ros2 service call /send_goal_server fleet_msg/srv/SendGoal "{robot_name: 'andino2'}"
```

### Cancel a goal
Once a goal is being executed, users can cancel the goal given a robot name by,

```
ros2 service call /cancel_goal_server fleet_msg/srv/CancelGoal "{robot_name: 'andino2'}"
```
