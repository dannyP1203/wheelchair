cmake_minimum_required(VERSION 2.8.3)
project(wheelchair_gazebo_interface)

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  gazebo_ros_control
  roscpp
  std_msgs
  control_toolbox
  controller_manager
  hardware_interface
  transmission_interface
  pluginlib
  joint_limits_interface
  urdf
  angles
  )

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
		roscpp
		std_msgs
		controller_manager
		control_toolbox
		pluginlib
		hardware_interface
		transmission_interface
		gazebo_ros_control
)

add_library(${PROJECT_NAME}
  src/sample_interface.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
