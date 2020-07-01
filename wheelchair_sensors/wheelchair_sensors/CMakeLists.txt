cmake_minimum_required(VERSION 2.8.3)
project(wheelchair_sensors)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS gazebo_ros roscpp)
find_package(gazebo REQUIRED)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(
	${catkin_INCLUDE_DIRS}
	${GAZEBO_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})
## Permette di compilare i file in c++ e includere le librerie .hh
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_executable(read_pos_ag src/read_pos_ag.cpp)
add_executable(read_sensors src/read_sensors.cpp)
add_executable(read_qr_code src/read_qr_code.cpp)
add_executable(read_image src/read_image.cpp)

target_link_libraries(read_pos_ag ${catkin_LIBRARIES})
target_link_libraries(read_sensors ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(read_qr_code ${catkin_LIBRARIES})
target_link_libraries(read_image ${catkin_LIBRARIES})

