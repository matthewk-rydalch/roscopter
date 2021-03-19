cmake_minimum_required(VERSION 2.8.3)
project(roscopter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  rosflight_msgs
  rosflight_utils
  roscopter_utils
  std_msgs
  nav_msgs
  sensor_msgs
  rosbag
  roslib
)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(GTest REQUIRED)
#can possibly add these required packages above

find_package(inertial_sense QUIET)

if (${inertial_sense_FOUND})
    message(WARNING "found inertial sense library")
    add_definitions(-DINERTIAL_SENSE) #defines Intertial Sense
else()
    message(WARNING "Did NOT find inertial sense library")
endif()

find_package(ublox QUIET)

if (${ublox_FOUND})
    message(WARNING "found ublox library")
    add_definitions(-DUBLOX) #defines UBLOX
else()
    message(WARNING "Did NOT find ublox library")
endif()

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp rospy dynamic_reconfigure eigen_conversions
    rosflight_msgs rosflight_utils std_msgs nav_msgs sensor_msgs roslib
  DEPENDS EIGEN3
)

add_definitions(-DROSCOPTER_DIR="${CMAKE_CURRENT_LIST_DIR}")

###########
## Build ##
###########

include_directories(include
  ${EIGEN3_INCLUDE_DIRS} 
  ${catkin_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
  )
message (STATUS "catkin include dirs" ${catkin_INCLUDE_DIRS})

## Declare cpp executables

add_executable(ekf_node src/ekf/ekf_node.cpp)
target_link_libraries(ekf_node ekf_ros ${catkin_LIBRARIES})
add_dependencies(ekf_node ${catkin_EXPORTED_TARGETS})

add_library(ekf_ros
  src/ekf/ros/ekf_ros.cpp
  src/ekf/ros/ekf_cb_ros.cpp
  )
target_link_libraries(ekf_ros ekf ${catkin_LIBRARIES})
add_dependencies(ekf_ros ${catkin_EXPORTED_TARGETS})

add_library(ekf
  src/ekf/state.cpp
  src/ekf/dynamics.cpp
  src/ekf/ekf.cpp
  src/ekf/meas.cpp
  src/ekf/log.cpp
  src/ekf/sensors.cpp
)
target_include_directories(ekf PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(ekf ${YAML_CPP_LIBRARIES} stdc++fs)
