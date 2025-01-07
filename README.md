<p align="center">
<img src=./images/logo.png width=40% height=40%>
</p>

# grasp-factory
![ROS](https://img.shields.io/badge/ros-%230A0FF9.svg?style=for-the-badge&logo=ros&logoColor=white)
![CMake](https://img.shields.io/badge/CMake-%23008FBA.svg?style=for-the-badge&logo=cmake&logoColor=white)
![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

Automatic Grasp Success/Failure Checker based on Gazebo (ROS1). Depending on user's inputs (e.g., list of planned grasps), the program spawns an object and a gripper ([Gripkit-CR-Plus-L](https://weiss-robotics.com/gripkit/)) models to check if each planned grasp succeeds or fails.

<p align="center">
<img src=./images/demo.gif>
</p>

The project was done while [Hojun Lee](https://www.linkedin.com/in/hjunlee94/) was working for Barton Research Group ([BRG](https://brg.engin.umich.edu/)) at the University of Michigan.

## Table of Contents

- [Repository Structure](#repository-structure)
- [Download Process](#download-process)
- [Simulation](#simulation)
    - [Gazebo Only](#gazebo-only)
    - [MoveIt Only](#moveit-only)
    - [MoveIt and Gazebo](#moveit-and-gazebo)
- [Automated Grasping](#automated-grasping)
    - [How to Run](#how-to-run)
    - [Grasp Representation](#grasp-representation)
- [ToDo Lists](#todo-lists)

---

## Repository Structure

    ├── auto_grasp
    │   └── src                           # Python source codes
    ├── gripkit_cr_plus_l_bb_description
    │   ├── launch                        
    │   ├── meshes                        # STL files
    |   └── urdf                          # URDF description
    ├── gripkit_cr_plus_l_bb_moveit
    │   ├── config                        # config files
    │   └── launch                        # ROS-launch files
    └── models
        ├── dict                          # grasp dictionaries
        └── obj_05                        # object folder

## Download Process

> [!NOTE]
This repository has been tested on [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) and [Ubuntu 20.04](https://releases.ubuntu.com/focal/).
It also depends on **numpy**, **scipy**, and **tqdm**:

    cd ~/catkin_ws/src
    git clone https://github.com/kidpaul94/grasp-factory.git
    cd grasp-factory/
    pip3 install -r requirements.txt
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash

## Simulation

### Gazebo Only:

> [!NOTE]
This launch file is only for **model visualization** purpose. No ROS controllers will get initialized.

    roslaunch gripkit_cr_plus_l_bb_description gazebo.launch
    
### MoveIt Only:

    roslaunch gripkit_cr_plus_l_bb_moveit_config demo.launch

### MoveIt and Gazebo:

    roslaunch gripkit_cr_plus_l_bb_moveit_config demo_gazebo.launch
    

## Automated Grasping

### How to Run:

> [!NOTE]
`simulation.py` receives several different arguments. Run the `--help` command to see everything it receives.

    cd auto_grasp/src
    python3 simulation.py --help

### Grasp Representation:

> [!NOTE]
The representation is defined based on a coordinate system of each object 3D model.

<p align="center">
<img src=./images/representation.png width=25% height=25%>
</p>

The image above shows the **grasping center & direction & approach vector** format that we use to spawn an object at specific poses. In [`simulation.py`](https://github.com/kidpaul94/grasp-factory/blob/main/auto_grasp/src/simulation.py), we utilize a list of this representation as a grasp dictionary associated with a specific object model and check success/failure of each planned grasp.

## ToDo Lists

| **Model & Controller parameters tuning** | ![Progress](https://geps.dev/progress/100) |
| --- | --- |
| **Documentation** | ![Progress](https://geps.dev/progress/100) |
