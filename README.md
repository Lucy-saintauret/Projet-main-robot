# Robotic Hand Prototype

Author : Lucy SAINT-AURET  
Starting date : 29/04/2025  
Ending date : --/--/----  

This is a personal project aiming to design and built a robotic hand using ROS2, RViz and the Gazebo simulator. In this repository, you will find all the code for this project. 


## Table of contents

- [Robotic Hand Prototype](#robotic-hand-prototype)
  - [Table of contents](#table-of-contents)
  - [Demo and visuals](#demo-and-visuals)
  - [Project overview](#project-overview)
  - [Installation](#installation)
  - [Usage](#usage)

## Demo and visuals



## Project overview

This project was implemented on ROS2 Jazzy. 

Here is the package `hand_description`, 

```
.
└── hand_description
    ├── CMakeLists.txt
    ├── config
    │   └── controllers.yaml
    ├── include
    │   └── hand_description
    ├── launch
    │   └── index.launch.py
    ├── LICENSE
    ├── meshes
    │   ├── bearing_25x42x11.stl
    │   ├── forearm.stl
    │   ├── index_base_left.stl
    │   ├── index_base_right.stl
    │   ├── index_end.stl
    │   ├── index_middle_left.stl
    │   ├── index_middle_right.stl
    │   ├── middle_base_left.stl
    │   ├── middle_base_right.stl
    │   ├── middle_end.stl
    │   ├── middle_middle_left.stl
    │   ├── middle_middle_right.stl
    │   ├── palm.stl
    │   ├── servo_circle_horn.stl
    │   ├── Servo_Hitec_HS_645MG.stl
    │   ├── SG90_arm.stl
    │   ├── SG90.stl
    │   ├── steeringlink_end_v2.stl
    │   └── wrist.stl
    ├── package.xml
    ├── rviz
    │   ├── config.rviz
    │   └── rviz_config.rviz
    ├── src
    └── urdf
        ├── common.xacro
        ├── forearm.xacro
        ├── hand.xacro
        ├── index.xacro
        ├── middle.xacro
        ├── palm.xacro
        ├── ros2_control.xacro
        └── wrist.xacro

```

## Installation 

Make sure you have ROS2, RViz and Gazebo installed in you machine.  
Clone this repository.
You will need to build this project : 
```bash
cd ~/hand_robot_ws
source /opt/ros/jazzy/setup.bash
colcon build 
source install/setup.bash
```

Then yoou can launch : 
```bash
ros2 launch hand_description demo.launch.py
```



## Usage


