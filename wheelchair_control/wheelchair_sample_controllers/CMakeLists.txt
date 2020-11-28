cmake_minimum_required(VERSION 2.8.3)
project(wheelchair_sample_controllers)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  control_toolbox
  controller_interface
  geometry_msgs
  realtime_tools
  pluginlib
  roscpp
  tf
  tf2_geometry_msgs
  wheelchair_msgs
  )


catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp wheelchair_msgs
    DEPENDS
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
  src/sample_diff_controller.cpp
  src/sample_effort_controller.cpp
  )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


