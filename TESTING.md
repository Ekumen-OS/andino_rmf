# Andino RMF Testing

This document outlines a series of tests to verify the functionality of the Andino fleet management with Open RMF.

## Prerequisites

First of all, the docker container needs to be running. To achieve this, follow the instructions on the installation and package sections of the [README](README.md) file

```bash
source install/setup.bash
ros2 launch andino_rmf andino_office.launch.py
```

All subsequent test commands should be run in a new, separate terminal. To start a new terminal inside the docker container, use the [join.sh](docker/join.sh) script inside the docker folder.

---

## Test Scenarios

### Test 1: Send a Single Goal to a Robot

**Purpose:** This test verifies that a single robot can be dispatched on a simple A-to-B task.

**Instructions:**
1.  Open a new terminal and source the workspace:
    ```bash
    source install/setup.bash
    ```
2.  Send a patrol task for a robot to move from `room2` to `room6`:
    ```bash
    ros2 run rmf_demos_tasks dispatch_patrol -p room2 room6 -n 1 --use_sim_time
    ```

**Expected Outcome:**
*   One of the robots in the simulation will be assigned the task.
*   The robot will navigate from the waypoint `room2` to the waypoint `room6`.
*   After reaching `room6`, the robot will complete the task and become available for new tasks.

---

### Test 2: Simultaneous Robot Movement

**Purpose:** This test verifies that multiple robots can operate concurrently without conflicts.

**Instructions:**
1.  Open two new terminals and source the workspace in each.
2.  In the first terminal, send a patrol task from `room2` to `room6`:
    ```bash
    ros2 run rmf_demos_tasks dispatch_patrol -p room2 room6 -n 1 --use_sim_time
    ```
3.  Quickly, in the second terminal, send another patrol task from `room3` to `room5` (assuming `room3` and `room5` are valid waypoints):
    ```bash
    ros2 run rmf_demos_tasks dispatch_patrol -p room3 room5 -n 1 --use_sim_time
    ```

**Expected Outcome:**
*   Two different robots should be assigned these tasks.
*   The robots will start navigating simultaneously to their respective destinations.
*   Open RMF will manage their paths to avoid collisions. You should see them navigate around each other if their paths intersect.

---

### Test 3: One Robot with Multiple Goals

**Purpose:** This test verifies that a single robot can execute a task composed of a sequence of waypoints.

**Instructions:**
1.  Open a new terminal and source the workspace.
2.  Send a patrol task with multiple waypoints, for example, from `room2` to `room6`, then to `room1`, and finally to `room4`.
    ```bash
    ros2 run rmf_demos_tasks dispatch_patrol -p room2 room6 room1 room4 -n 1 --use_sim_time
    ```

**Expected Outcome:**
*   A single robot will be assigned the task.
*   The robot will navigate sequentially through the specified waypoints: `room2` -> `room6` -> `room1` -> `room4`.
*   The robot will only complete its task after visiting the final waypoint in the sequence.
