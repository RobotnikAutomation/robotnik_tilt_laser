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
    laser_assembler
    )


catkin_package(
#  INCLUDE_DIRS
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

## Specify libraries to link a library or executable target against
target_link_libraries(robotnik_tilt_laser_node ${catkin_LIBRARIES} -ldxl)
target_link_libraries(robotnik_tilt_laser_cloud_node ${catkin_LIBRARIES} -ldxl)

add_dependencies(robotnik_tilt_laser_node ${catkin_EXPORTED_TARGETS})
add_dependencies(robotnik_tilt_laser_cloud_node ${catkin_EXPORTED_TARGETS})