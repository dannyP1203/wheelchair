cmake_minimum_required(VERSION 2.8.3)
project(wheelchair_navigation)

find_package(catkin REQUIRED COMPONENTS
  gazebo
  roscpp
  tf
  sensor_msgs
  nav_msgs
  geometry_msgs
)
  

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp
 DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
## Permette di compilare i file in c++ e includere le librerie .hh
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_executable(encoder_sim src/encoder_sim.cpp)
add_executable(encoder_imu_sim src/encoder_imu_sim.cpp)
add_executable(wheel_odometry src/wheel_odometry.cpp)

target_link_libraries(encoder_sim ${catkin_LIBRARIES})
target_link_libraries(encoder_imu_sim ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(wheel_odometry ${catkin_LIBRARIES})
