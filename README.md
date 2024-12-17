# Andino Fleet with OpenRMF

<img src="docs/andino_rmf.png"/>

This package intends to utilize OpenRMF to send a high-level task planning to [Andino](https://github.com/Ekumen-OS/andino) robot fleet. To achieve this goal, three main developments are included as the following.
  - [Controller server](https://github.com/ekumenlabs/andino_fleet_open_rmf/blob/main/andino_fleet/README.md#controller-server) for an Andino robot
  - [Fleet manager](https://github.com/ekumenlabs/andino_fleet_open_rmf/blob/main/andino_fleet/README.md#fleet-manager) to manage multiple robots
  - [Fleet adapter](https://github.com/ekumenlabs/andino_fleet_open_rmf/tree/main/andino_fleet_adapter) to bridge a fleet manager with OpenRMF API
  - [RMF map](https://github.com/ekumenlabs/andino_fleet_open_rmf/tree/main/andino_rmf_maps) to contains map files required by RMF navigation
  - [RMF sim](https://github.com/ekumenlabs/andino_fleet_open_rmf/tree/main/andino_rmf_sim) to launch everything

## Project dependencies
- <b>ROS 2</b>: Humble Hawksbill
- <b>OS</b>: Ubuntu 22.04 Jammy Jellyfish
- <b>Simulation</b>: 
    - [andino_gz](https://github.com/Ekumen-OS/andino_gz/tree/humble?tab=readme-ov-file): a simulation environment for [Andino](https://github.com/Ekumen-OS/andino) in Gazebo Fortress
- <b>Task Planning</b>:
  - [OpenRMF](https://github.com/open-rmf/rmf) : The Open-RMF platform for multi-fleet robot management
- <b>Python module</b>:
    - [nudged](https://pypi.org/project/nudged/) : A Python lib to estimate scale, rotation, and translation between two sets of 2D points required by RMF system
    - [transform3d](https://pypi.org/project/transforms3d/) : Code to convert between various geometric transformations

## :inbox_tray: Installation

1. Clone this repository

```sh
git clone https://github.com/ekumenlabs/andino_rmf
```

2. Set up docker environment:
Refer to [docker readme](docker/README.md)

Once the container is running and dependencies have been installed you can proceed to package building.

## :package: Build

The package contains some dependencies that must be installed in order to build it:

```
rosdep install --from-paths src -i -y
```

Then build the package and source the install workspace. To do so run the following commands:

```sh
colcon build
source install/setup.bash
```

## Usage
This launch file will start everything for the fleet management. Specifically, it will start simulation with robots, controllers for each robot, manager for the fleet, the adapter that connects to RMF and the core nodes from RMF.

The system has two options for controllers - custom controller and Nav2 controller - each has different ways of launching the system.

### Custom Controller
To launch system with custom controllers

```
ros2 launch andino_rmf_sim andino_manager.launch.py
```

In a new terminal, launch RMF

```
ros2 launch andino_rmf_sim andino_office_rmf.launch.py
```

### Nav2

To launch system with Nav2 controllers

```
ros2 launch andino_rmf_sim andino_manager.launch.py nav2:=True
```

In a new terminal, set robot initial positions and run RMF

```
source install/setup.bash
ros2 service call /initial_pose_server andino_fleet_msg/srv/InitPose "{robot_names: ['andino1', 'andino2', 'andino3', 'andino4']}"
ros2 launch andino_rmf_sim andino_office_rmf.launch.py
```
## Demo
We need to install [rmf_demos](https://github.com/open-rmf/rmf_demos) package as we are utilizing the task command from that package.
At this point, we can send the configured command to the fleet and that we are using one of the default tasks for our example.

In a new terminal,
```
source install/setup.bash
```
For example, send a patrol task from point `room2` to `room6`, only 1 (n=1) round.
```
ros2 run rmf_demos_tasks dispatch_patrol -p room2 room6 -n 1 --use_sim_time
```
