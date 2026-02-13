# RMF GZ Package
This package contains high-level launch files to launch the gazebo system with spawned Andino robots

This package launches the simulation with a spawned robot fleet

## Usage
To launch multiple robots with corresponding controller servers, run:

```
ros2 launch andino_rmf_gz spawn_multiple_robot.launch.py
```

<img src="../resources/multi_robot.png" alt="Multi-robot simulation" title="Multi-robot simulation" width="300"/>

*<b>Note: </b> To add/remove robot(s), edit <b>spawn_robots.yaml</b> under <b>[andino_fleet/config](https://github.com/ekumenlabs/andino_fleet_open_rmf/tree/main/andino_rmf_gz/config)</b> folder. There are four robots by default.*
