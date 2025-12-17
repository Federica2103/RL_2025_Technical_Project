# RL_2025_HW03

# 🤖 Robotics Lab - Homework 3: Fly your drone
 
## Introduction
This repository contains the source code and configuration files developed for the simulation of a custom multi-rotor drone and the implementation of control logic and trajectory planning using **PX4-Autopilot** and the Offboard mode.
 
---
 
 
## ⚙️ Getting Started
 
To successfully set up and test the project, follow these steps within your ROS 2 workspace.
 
1.  **Clone the Repository: Navigate to your workspace source directory and clone the repository.**
    ```shell
    cd /ros2_ws
    git clone https://github.com/Federica2103/RL_2025_HW03.git
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive
    cd PX4-Autopilot
    git checkout v1.16.0
    cd ..
    git clone https://github.com/PX4/px4_msgs.git
    cd px4_msgs
    git checkout release/1.16
    ```

2.  **Replace the correct files**
    Update the original **PX4-Autopilot** folder by copying the corresponding files from **RL2025HW03/PX4-Autopilot** over them.
    
3.  **Build the Workspace: Return to the workspace root, build the packages, and source the environment.**
    ```shell
     cd ..
     colcon build
     . install/setup.bash
    ```
    
4.  **Launch QGroundControl: Ensure your ground control station is running to monitor the drone status.**
 
 
## 🏃 Execution Instructions
 
This section details the commands to launch and test the custom y6 hexacopter drone model and the developed control nodes.
 
## **1. Basic Flight and Custom Airframe Setup**
Start the drone simulation in Gazebo using the custom airframe configuration. This will put the drone in a Position Flight Mode for manual control and allow you to verify the custom model.
**Launch PX4 SITL and Gazebo**
```shell
cd src/PX4-Autopilot
make px4_sitl gz_y6_hexacopter
```
In another terminal, source the DDS Configuration: Execute the necessary script to correctly set up the DDS communication between ROS 2 and PX4:
```shell
. install/setup.bash
cd src
. DDS_run.sh 
```
 
The drone can now be controlled via virtual joystick (QGroundControl) or commands from the PX4 shell.
 
## **2. Enhanced Landing Safety Logic**
A specific modification has been implemented in the landing node logic to enhance robustness. This change prevents the automatic forced-landing procedure from being re-triggered if a pilot temporarily retakes manual control but fails to complete the landing immediately.
 
After launching the drone in Gazebo, source the enviroment and DDS configuration (repeat step 1), run this node in a new terminal:
 
```shell
ros2 run force_land force_land
```
 
## **3. Trajectory Planner Execution**
This section demonstrates the Offboard control capabilities. The trajectory planner guides the drone through a sequence of complex waypoints, maintaining a continuous, non-zero velocity profile between intermediate points for smooth flight.
 
After the drone is flying and armed (usually done via QGroundControl after launching step 1), run the planner in a new terminal:
 
```shell
ros2 run offboard_rl trajectory_planner
```
