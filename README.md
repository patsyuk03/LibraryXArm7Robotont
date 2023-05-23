# Kitting Station of the Learning Factory
## Contents: 
* [1. Introduction](#1-introduction)
* [2. Preparations before using this package](#2-preparations-before-using-this-package)
* [3. Kitting Station Setup](#3-kitting-station-setup)
* [4. Using kitting_station package](#4-using-kitting_station-package)
## 1. Introduction
Learning Factory is a work-based learning approach that provides students with hands-on experience working with industrial robots in a simulated factory environment. This repository is a ROS package that provides software for the manipulator robot in the kitting station of the learning factory. The software is developed for the library use case where a mobile robot carries the books to the manipulator robot which then stores them on the shelf. This package supports two types of setup:
1. With one manipulator robot (demo: //link)
2. With two manipulator robots (demo: //link)
## 2. Preparations before using this package
### 2.1. Create catkin workspace
Follow the instructions in [this page](http://wiki.ros.org/catkin/Tutorials/create_a_workspace). 
This readme instruction assumes the user continues to use '~/catkin_ws' as directory of the workspace.
### 2.2. Install dependencies to the created workspace
* xarm_ros: <https://github.com/xArm-Developer/xarm_ros.git>
* ar_track_alvar: <https://github.com/ros-perception/ar_track_alvar.git>
* multimaster_fkie: <https://github.com/fkie/multimaster_fkie.git>
* dual_xarm7_moveit_config: <https://github.com/patsyuk03/dual_xarm7_moveit_config.git>
### 2.3. Clone 'kitting_station'
Clone kitting_station to the same workspace :
```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/patsyuk03/kitting_station.git
$ cd ~/catkin_ws
$ catkin build
```
## 3. Kitting Station Setup
### 3.1. Hardware requirements for the demo
* UFACTORY xArm 7 
* Intel RealSense 
* WiFi router
### 3.2 Settting up the environment
1. Connect xArm 7 and PC to the router using ethernet cables (router's network should be 192.168.1.X)
2. Connect RealSense camera to the PC.
3. Place shelf near xArm 7.
## 4. Using kitting_station package
### 4.1 Source the workspace with kitting_station package
```bash
$ cd ~/catkin_ws
$ source devel/setup.bash
```
### 4.2 Run the demo
If setup with two manipulator robots are used set argument 'dual_arm' to 'true' (default is 'false')
```bash
$ roslaunch kitting_station director.launch [dual_arm:=true]
```
It will ask what you want to run Find Shelf or Main Program. If it is a new setup choose Find Shelf to let the robot know the location of the shelf. After the shelf is found you can choose the Main Program which will initialize the moveit commander and wait for the request from the mobile robot to start transferring the books.
### 4.3 Fake mobile robot request
If you want to run a demo without mobile robot fake_request node can be used to initialize the transfer of the books:
```bash
$ rosrun kitting_station fake_request.py
```
