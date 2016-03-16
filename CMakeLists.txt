cmake_minimum_required(VERSION 2.8.3)
project(robotnik_tilt_laser)

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    tf
    self_test
    diagnostic_updater
    sensor_msgs 
    std_srvs
    robotnik_msgs
    )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
#add_message_files(
#   FILES
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
 #  DEPENDENCIES
 #  sensor_msgs
 #  robotnik_msgs
 #)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES 
  CATKIN_DEPENDS 
    roscpp 
    tf
    self_test
    diagnostic_updater
    sensor_msgs 
    std_srvs
    laser_assembler
    robotnik_msgs
#  DEPENDS system_lib
)

# include_directories(include)
include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(~/catkin_ws/src/robotnik_tilt_laser)

## Declare a cpp executable
add_executable(robotnik_tilt_laser_node src/robotnik_tilt_laser.cpp)
add_executable(robotnik_tilt_laser_cloud_node src/robotnik_tilt_laser_cloud.cpp)
add_executable(periodic_snapshotter src/periodic_snapshotter.cpp)
  
#add_dependencies(robotnik_tilt_laser_node robotnik_tilt_laser_node_gencpp)

## Specify libraries to link a library or executable target against
target_link_libraries(robotnik_tilt_laser_node ${catkin_LIBRARIES} -ldxl)
target_link_libraries(robotnik_tilt_laser_cloud_node ${catkin_LIBRARIES} -ldxl)
target_link_libraries(periodic_snapshotter ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(periodic_snapshotter ${PROJECT_NAME}_gencpp)



