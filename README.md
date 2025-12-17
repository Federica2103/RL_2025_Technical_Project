# RL_2025_TECHNICAL PROJECT

# 🤖 Robotics Lab - Final Project: Robotic Warehouse
 
## Introduction
This project presents a collaborative warehouse automation system developed within the ROS 2 framework and simulated using Gazebo. 

It orchestrates a seamless workflow between a KUKA IIWA manipulator and a Fra2Mo mobile robot to handle autonomous sorting and delivery tasks. 

The IIWA arm identifies specific parcels—such as medicine, toys, or clothes—using Aruco markers (IDs 1, 2, and 3) and manages the picking process through a KDL-based action server. Once the parcel is loaded, the mission is handed over to the Fra2Mo robot, which utilizes the Nav2 stack and Lidar data for autonomous navigation and obstacle avoidance. For final delivery, the mobile robot performs precision docking at designated zones (IDs 11-14) using visual servoing.

---
 
 
## ⚙️ Getting Started
 
To successfully set up and test the project, follow these steps within your ROS 2 workspace. 
 
1.  **Clone the Repository**
    ```shell
    cd /ros2_ws
    git clone https://github.com/Federica2103/RL_2025_Technical_Project.git
    ```

2.  **Build the Workspace: Return to the workspace root, build the packages, and source the environment.**
    ```shell
     colcon build
     source install/setup.bash
    ```
 
 
## 🏃 Execution Instructions
 
To properly initiate the collaborative mission between the two robots, the following commands must be executed in separate terminals.
 
## **1. Environment and Gazebo Simulation Setup**
This command initializes the warehouse environment, loads the robot models (IIWA and Fra2Mo), and starts aruco nodes.
```shell
ros2 launch warehouse_project warehouse_launch.py
```
 
## **2. KDL Action Server Configuration (IIWA)**
This launches the action server based on the KDL library to manage the kinematics and motion control for the IIWA manipulator arm.
```shell
ros2 launch ros2_kdl_package kdl_action.launch.py
```
 
## **3. Autonomous Navigation Activation (Fra2Mo)**
This initializes the Nav2 stack, loading the warehouse map and configuring the localization systems required for the mobile robot's movement.
```shell
ros2 launch ros2_fra2mo fra2mo_navigation.launch.py
```

## **4. Fra2Mo Task Manager Execution**
This starts the node responsible for delivery logic, visual docking via Aruco markers, and synchronization with the manipulator arm.
```shell
ros2 run warehouse_project fra2mo_task_manager.cpp
```

## **5. IIWA Task Manager Execution**
This activates the robotic arm supervisor, which coordinates the object-picking phases and triggers the transportation missions.
```shell
ros2 run warehouse_project iiwa_task_manager.cpp
```
