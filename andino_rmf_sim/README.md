# RMF Sim Package
This package contains high-level launch files to launch the entire system with Andino fleet and RMF

This package launches the following ...
- RMF tools *(common.launch.py)*
- Simulation with spawned robot fleet with Nav2-enabled controllers or custom controllers
- Fleet manager
- Fleet adapter

## Usage
Launch the manager with custom controller (default parameter) using the following command.

```
ros2 launch andino_rmf_sim andino_manager.launch.py
```

If you want to use Nav2 controller instead, use the nav2 argument

```
ros2 launch andino_rmf_sim andino_manager.launch.py nav2:=True
```

Launch the RMF system using the following command,

```
ros2 launch andino_rmf_sim andino_office_rmf.launch.py
```