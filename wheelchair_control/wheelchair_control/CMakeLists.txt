cmake_minimum_required(VERSION 2.8.3)
project(wheelchair_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
