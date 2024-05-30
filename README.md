# Andino_fleet_open_rmf
This package intends to utilize OpenRMF to send a high-level task planning to Andino robot fleet. To achieve this goal, three main developments are included as the following.
<ol>
  <li>controller server for an Andino robot</li>
  <li>fleet manager to manage multiple robots</li>
  <li>fleet adapter to bridge a fleet manager with OpenRMF API</li>
</ol>

## Project dependencies
<ul>
  <li><b>ROS 2</b>: Humble Hawksbill</li>
  <li><b>OS</b>: Ubuntu 22.04 Jammy Jellyfish</li>
  <li><b>Simulation</b>: andino_gz</li>
  <li><b>OpenRMF</b></li>
</ul>

## Usage
To launch multiple robots with corresponding controller servers,

```ros2 launch andino_fleet spawn_multiple_robot.launch.py```

<img src="https://github.com/ekumenlabs/andino_fleet_open_rmf/blob/readme-writing/docs/Screenshot%20from%202024-05-21%2010-18-39.png" alt="Multi-robot simulation" title="Multi-robot simulation" width="300"/>

*<b>Note: </b> To add/remove robot(s), edit <b>spawn_robots.yaml</b> under <b>andino_fleet/config</b> folder. There are two robots by default.*

To run a fleet manager,

```ros2 run andino_fleet fleet_manager```

After the fleet manager node is running, it allows users to interact with the robot fleet as the following.

### Add a goal
Add a goal to the manager for a specific robot, given the robot name and the final pose,

```ros2 service call /add_goal_server fleet_msg/srv/RobotControl "{robot_name: 'andino2', final_pose: [0.1,0,0]}"```

### Send a goal
Start moving a robot by sending a goal to the manager, assuming that the goal is already added,

```ros2 service call /send_goal_server fleet_msg/srv/SendGoal "{robot_name: 'andino2'}"```

### Cancel a goal
Once a goal is being executed, users can cancel the goal given a robot name by,

```ros2 service call /cancel_goal_server fleet_msg/srv/CancelGoal "{robot_name: 'andino2'}"```


