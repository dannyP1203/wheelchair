cmake_minimum_required(VERSION 2.8.3)
project(wheelchair_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  tf2_ros
  std_msgs
)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(wheel_odometry src/wheel_odometry.cpp)
target_link_libraries(wheel_odometry ${catkin_LIBRARIES})

add_executable(velocity_from_bf_to_map src/velocity_from_bf_to_map.cpp)
target_link_libraries(velocity_from_bf_to_map ${catkin_LIBRARIES})
