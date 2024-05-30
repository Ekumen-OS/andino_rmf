# Andino_fleet_open_rmf
This package intends to utilize OpenRMF to send a high-level task planning to Andino robot fleet. To achieve this goal, three main developments are included as the following.
<ol>
  <li>controller server for an Andino robot</li>
  <li>fleet manager to manage multiple robots</li>
  <li>fleet adapter to bridge a fleet manager with OpenRMF API</li>
</ol>

## Project dependencies
<ul>
  <li>ROS 2: Humble Hawksbill</li>
  <li>OS: Ubuntu 22.04 Jammy Jellyfish</li>
  <li>Simulation: andino_gz</li>
  <li>OpenRMF</li>
</ul>

## Usage
To launch multiple robots with corresponding controller servers,

```ros2 launch andino_fleet spawn_multiple_robot.launch.py```

<b>Note: </b> To add/remove robot(s), edit *spawn_robots.yaml* under andino_fleet/config folder. There are two robots by default.

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


