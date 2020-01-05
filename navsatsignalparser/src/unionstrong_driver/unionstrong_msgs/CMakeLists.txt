cmake_minimum_required(VERSION 2.8.3)
project(unionstrong_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
  nav_msgs
  roscpp
  rospy
)

add_message_files(
  FILES
  GPFPD.msg 
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime nav_msgs std_msgs
)

