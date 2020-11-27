cmake_minimum_required(VERSION 2.8.3)
project(wheelchair_bringup)

find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	rospy 
	std_msgs
	nav_msgs
	tf
)

catkin_package()


include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(automatic src/automatic.cpp)
target_link_libraries(automatic ${catkin_LIBRARIES})

add_executable(comando src/comando.cpp)
target_link_libraries(comando ${catkin_LIBRARIES})

add_executable(goal_setting src/goal_setting.cpp)
target_link_libraries(goal_setting ${catkin_LIBRARIES})

add_executable(robot_setup_tf src/robot_setup_tf.cpp)
target_link_libraries(robot_setup_tf ${catkin_LIBRARIES})