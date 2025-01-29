# Mycobot 630 Pro Mezzotint ROS2 Project

<div align="center">
    <img src="images/ROS_2_logo.png" width="250" alt="ROS2 Jazzy Jalisco" style="margin-right: 20px; margin-top: 30px; margin-bottom: 35px"/>
    <img src="images/moveit-2-logo-dark.png" width="300" alt="MoveIt"/>
</div>

![Path Planning Demo](images/path_planning_demo.png)

## Overview
This project implements an automated mezzotint printmaking process using the Mycobot 630 Pro robotic arm. The system replicates the traditional hand-rocking motion used in mezzotint printmaking on copper plates, leveraging ROS2's Jazzy Jalisco distribution and MoveIt2 for motion planning and control.

## System Requirements

### Operating System
- Ubuntu 24.04 LTS (*Noble Numbat*)

### ROS2 Distribution
- ROS2 Jazzy Jalisco

### Hardware
- Mycobot 630 Pro Robotic Arm
- Modified End-effector (Mezzotint Rocker Tool)

### Key Dependencies
- MoveIt2
- Open Motion Planning Library (OMPL)
- KDL Kinematics Plugin

## Project Structure

### Packages Overview

#### 1. mycobot630pro_description
Contains the robot arm model and associated files:
- URDF (Unified Robot Description Format) model
- DAE (COLLADA) files for robot parts
- Modified link6.dae incorporating the mezzotint rocker tool
- Launch files for model visualization

#### 2. cobot630pro_moveit_config
MoveIt2 configuration package including:
- KDL inverse kinematics solver setup
- Controller manager configuration
- OMPL planning pipeline settings
- Planning groups configuration:
  - cobot630_arm
  - cobot_tool
- Predefined robot states
- RViz configurations for visualization

#### 3. cobot_interfaces
Custom interfaces package containing:
- Service definitions
- Message definitions
- Action definitions

#### 4. mycobot_path_planner
Path planning implementation package:
- Path generation service server for mezzotint operation
- RViz visualization using MarkerArray
- Motion planning algorithms
- Path optimization utilities

## Installation

For detailed installation instructions for both Linux and MacOS environments, please refer to our [Setup Manual](setup-manual.md).

## Usage

### Starting the System
```bash
# Source the workspace
source ~/colcon_ws/install/setup.bash

# In a new terminal, launch demo.launch
ros2 launch cobot630pro_moveit_config demo.launch.py
```

### Launching Path Planner Server
```bash
# Source the workspace
source ~/colcon_ws/install/setup.bash

# Start the path planning service
ros2 launch mycobot_path_planner path_planner.launch.py

# In a new terminal, launch rqt_service_caller to request /plan_path service with request variables
ros2 run rqt_service_caller rqt_service_caller
```

## Development Status
- ✅ Robot model and configuration
- ✅ MoveIt2 integration
- ✅ Basic path planning
- ✅ Visualization tools
- 🚧 [Future packages/features will be listed here]

---
<div align="center">
  <small>Built with ROS2 Jazzy Jalisco and MoveIt2</small>
</div>
