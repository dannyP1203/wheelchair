cmake_minimum_required(VERSION 2.8.3)
project(wheelchair_navigation)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  std_msgs
  tf
)
find_package(gazebo REQUIRED)
find_package(OpenCV REQUIRED)
  

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES wheelchair_localization
#  CATKIN_DEPENDS nav_msgs roscpp std_msgs tf
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
## Permette di compilare i file in c++ e includere le librerie .hh
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_executable(ukf_sim src/ukf_sim.cpp)
add_executable(encoder_sim src/encoder_sim.cpp)
add_executable(encoder_imu_sim src/encoder_imu_sim.cpp)
add_executable(qr_pose_sim src/qr_pose_sim.cpp)
add_executable(ukf_qr_sim src/ukf_qr_sim.cpp)
add_executable(lines src/lines.cpp)
add_executable(ukf_solo_qr_sim src/ukf_solo_qr_sim.cpp)

target_link_libraries(ukf_sim ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(encoder_sim ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(encoder_imu_sim ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(qr_pose_sim ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(ukf_qr_sim ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(lines ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(ukf_solo_qr_sim ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})