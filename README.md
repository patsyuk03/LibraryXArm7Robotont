# Kitting Station of the Learning Factory
## 1. Introduction
This repository is a 
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
Connect xArm 7 and PC to the router using ethernet cables. Routers network should be 192.168.1.X.
Place shelf near xArm 7.
## 4. Using kitting_station package
### 4.1 Source the workspace with kitting_station package
```bash
$ cd ~/catkin_ws
$ source devel/setup.bash
```
### 4.2 Run the demo
If setup with two manipulator robots are used set argument dual_arm to true (default is false)
```bash
$ roslaunch kitting_station director.launch [dual_arm:=true]
```
It will ask for the input of what you want to run Find Shelf or Main Program. If it is a new setup choose Find Shelf to let robot know its location. After the shelf is found you can choose the Main Program which will initialize the moveit commander and wait for the request from mobile robot to start transfering the books.
### 4.3 Fake mobile robot request
If you want to run a demo without mobile robot fake_request node can be used to initialize the transfer of the books:
```bash
$ rosrun kitting_station fake_request.py
```
