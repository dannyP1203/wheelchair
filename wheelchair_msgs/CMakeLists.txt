cmake_minimum_required(VERSION 2.8.3)
project(wheelchair_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  geometry_msgs 
  sensor_msgs 
  nav_msgs 
  message_generation
  actionlib_msgs
  )
    
add_message_files(
  FILES
  Command.msg
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
)
  
catkin_package(
    CATKIN_DEPENDS 
		roscpp 
		geometry_msgs 
		sensor_msgs 
		nav_msgs 
		actionlib 
		actionlib_msgs 
		message_runtime 
	DEPENDS
)
