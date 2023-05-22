cmake_minimum_required(VERSION 3.0.2)
project(kitting_station)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  message_generation
)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(DIRECTORY srv
  FILES
  Robotont.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

###########
## Build ##
###########

include_directories()

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  src/director.py
  src/find_shelf.py
  src/single_arm.py
  src/dual_arm.py
  
  src/transform_pose_marker.py
  src/fake_request.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)