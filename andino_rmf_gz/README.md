# RMF GZ Package
This package contains high-level launch files to launch the gazebo system with spawned Andino robots

This package launches the following ...
- RMF tools *(common.launch.py)*
- Simulation with spawned robot fleet
- Fleet manager
- Fleet adapter

## Usage
To launch multiple robots with corresponding controller servers,

```
ros2 launch andino_rmf_gz spawn_multiple_robot.launch.py
```

<img src="../resources/multi_robot.png" alt="Multi-robot simulation" title="Multi-robot simulation" width="300"/>

*<b>Note: </b> To add/remove robot(s), edit <b>spawn_robots.yaml</b> under <b>[andino_fleet/config](https://github.com/ekumenlabs/andino_fleet_open_rmf/tree/main/andino_rmf_gz/config)</b> folder. There are four robots by default.*