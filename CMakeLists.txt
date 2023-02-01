cmake_minimum_required(VERSION 2.8.3)
project(jp_exjobb)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  skiros2_msgs
  genmsg
  actionlib_msgs
  actionlib
)

catkin_python_setup()

add_action_files(
  DIRECTORY
  action
  FILES
  TestMsg.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS 
#  LIBRARIES 
 CATKIN_DEPENDS rospy skiros2_msgs
#  DEPENDS 
)
