# System Architecture

This document outlines the high-level architecture of the Andino RMF integration, explaining the main components and their interactions.

## High-Level Overview

The purpose of this system is to integrate a fleet of Andino robots with the [Robot Middleware Framework (RMF)](https://www.open-rmf.org/). This allows RMF to manage and coordinate the Andino robots for tasks such as navigation and delivery.

The system consists of three main components:

*   **Fleet Adapter**: Connects the RMF core with the Fleet Manager.
*   **Fleet Manager**: Manages the individual robots in the fleet.
*   **Robot Handler**: An internal component of the Fleet Manager that manages the state and communication for a single robot.

## Components

### Fleet Adapter

The Fleet Adapter is the bridge between RMF and the Andino fleet. It receives commands from RMF and translates them into commands that the Fleet Manager can understand. It also provides RMF with updates on the state of the robots.

### Fleet Manager

The Fleet Manager is the central coordinator for the Andino fleet. It provides a single point of contact for controlling the robots and abstracts the details of communicating with individual robots. It exposes its functionality through ROS 2 services.

### Robot Handler

The Robot Handler is an internal component of the Fleet Manager. Each robot in the fleet has its own Robot Handler instance, which is responsible for:

*   Managing the state of the robot (e.g., position, battery level).
*   Communicating with the robot through ROS 2 topics and actions.
*   Sending navigation goals and canceling active goals for the robot.

## Communication

The components communicate with each other using a combination of ROS 2 services, topics, and actions.

*   **RMF Core to Fleet Adapter**: RMF sends high-level commands (e.g., `follow_new_path`, `dock`) to the Fleet Adapter.
*   **Fleet Adapter to Fleet Manager**: The Fleet Adapter uses the following ROS 2 services to communicate with the Fleet Manager:
    *   `/send_goal_service`: To send a navigation goal to a robot.
    *   `/cancel_goal_service`: To cancel a robot's current goal.
    *   `/robot_pose_service`: To request the position of a robot.
*   **Fleet Manager to Robot Handler**: The Fleet Manager (through the Robot Handler) communicates with individual robots using ROS 2 topics and actions. For example, for a robot named `andino1`, the following topics are used:
    *   `/andino1/navigate_to_pose`: An action to send a navigation goal to the robot.
    *   `/andino1/amcl_pose`: A topic to receive pose updates from the robot.
