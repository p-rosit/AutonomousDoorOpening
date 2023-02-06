cmake_minimum_required(VERSION 2.8.3)
project(jp_exjobb)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  skiros2_msgs
  cv_bridge
  image_transport
  sensor_msgs
  genmsg
  actionlib_msgs
  actionlib
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  CameraCalibrationMsg.msg
)

add_action_files(
  DIRECTORY
  action
  FILES
  TestMsg.action
  Empty.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS 
#  LIBRARIES 
 CATKIN_DEPENDS rospy skiros2_msgs message_runtime
#  DEPENDS 
)
