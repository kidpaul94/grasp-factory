# grasp-factory
Automatic Grasp Success/Failure Checker based on Gazebo

## Table of Contents

- [Repository Structure](#repository-structure)
- [Download Process](#download-process)
- [Simulation](#simulation)
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

> **Note**
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

    roslaunch gripkit_cr_plus_l_bb_moveit_config demo_gazebo.launch

## Automated Grasping

### How to Run:

> **Note**
`simulation.py` receives several different arguments. Run the `--help` command to see everything it receives.

    cd auto_grasp/src
    python3 simulation.py --help

### Grasp Representation:

> **Note**
The representation is defined based on a coordinate system of each object 3D model.

## ToDo Lists

| **Model & Controller parameters tuning** | ![Progress](https://progress-bar.dev/90) |
| --- | --- |
| **Documentation** | ![Progress](https://progress-bar.dev/10) |
