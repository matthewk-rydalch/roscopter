cmake_minimum_required(VERSION 3.10)
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

generate_dynamic_reconfigure_options(
 cfg/Controller.cfg
)

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

# Declare cpp executables
add_executable(controller_node src/controller/controller_node.cpp)
add_dependencies(controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(controller_node controller_ros ${catkin_LIBRARIES})


add_executable(vel_cntrl_node src/controller/vel_cntrl/vel_cntrl_node.cpp)
add_dependencies(vel_cntrl_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(vel_cntrl_node vel_cntrl_ros ${catkin_LIBRARIES})


add_executable(ff_cntrl_node src/controller/feed_forward_cntrl/ff_cntrl_node.cpp)
add_dependencies(ff_cntrl_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(ff_cntrl_node ff_cntrl_ros ${catkin_LIBRARIES})


add_executable(ekf_node src/ekf/ekf_node.cpp)
target_link_libraries(ekf_node ekf_ros ${catkin_LIBRARIES})
add_dependencies(ekf_node ${catkin_EXPORTED_TARGETS})

add_executable(target_ekf_node src/ekf/target_ekf_node.cpp)
target_link_libraries(target_ekf_node target_ekf_ros ${catkin_LIBRARIES})
add_dependencies(target_ekf_node ${catkin_EXPORTED_TARGETS})

add_executable(ekf_rosbag src/ekf/ros/ekf_rosbag.cpp)
target_link_libraries(ekf_rosbag ekf_ros ${catkin_LIBRARIES})
add_dependencies(ekf_rosbag ${catkin_EXPORTED_TARGETS})

option (BUILD_ROSCOPTER_TESTS "build roscopter tests" 0FF)
if (BUILD_ROSCOPTER_TESTS)
   add_executable(rtk_compassing_test
   # tests/rtk_compassing/state_test.cpp
   tests/rtk_compassing/covariance_test.cpp
   tests/rtk_compassing/compassing_utilities.cpp
   tests/ros_test_setup/compassing_setup.cpp
   include/tests/rtk_compassing_test.h
   )
   target_link_libraries(rtk_compassing_test ekf_ros ${catkin_LIBRARIES} ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} pthread)


   add_executable(ff_test
     tests/feed_forward/feed_forward_test.cpp
     include/tests/feed_forward_test.h
     )
   target_link_libraries(ff_test controller ff_cntrl ${catkin_LIBRARIES} ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} pthread)

   add_executable(ekf_test
      tests/ekf/ekf_load.cpp
      include/tests/ekf/ekf_load.h
      tests/ekf/ekf_initialize.cpp
      include/tests/ekf/ekf_initialize.h
      tests/ekf/ekf_dynamics.cpp
      include/tests/ekf/ekf_dynamics.h
      tests/ekf/ekf_propagate.cpp
      include/tests/ekf/ekf_propagate.h
      tests/ekf/ekf_update.cpp
      include/tests/ekf/ekf_update.h
   )
   target_link_libraries(ekf_test ekf ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} pthread)
endif (BUILD_ROSCOPTER_TESTS)

add_library(controller_ros
 src/controller/controller_ros.cpp
 include/controller/controller_ros.h
)
add_dependencies(controller_ros ${catkin_EXPORTED_TARGETS})
target_link_libraries(controller_ros controller ${catkin_LIBRARIES})

add_library(controller
 src/controller/controller.cpp
 include/controller/controller.h
 src/controller/simple_pid.cpp
)
add_dependencies(controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(controller ${catkin_LIBRARIES})


add_library(vel_cntrl
 src/controller/vel_cntrl/vel_cntrl.cpp
 include/controller/vel_cntrl.h
)
target_link_libraries(vel_cntrl controller ${catkin_LIBRARIES})
add_dependencies(vel_cntrl ${catkin_EXPORTED_TARGETS})

add_library(vel_cntrl_ros src/controller/vel_cntrl/vel_cntrl_ros.cpp)
target_link_libraries(vel_cntrl_ros vel_cntrl ${catkin_LIBRARIES})
add_dependencies(vel_cntrl_ros ${catkin_EXPORTED_TARGETS})

add_library(ff_cntrl
 src/controller/feed_forward_cntrl/ff_cntrl.cpp
 src/controller/feed_forward_cntrl/pd_cond_i.cpp
 include/controller/ff_cntrl.h
 include/controller/pd_cond_i.h
)
target_link_libraries(ff_cntrl controller ${catkin_LIBRARIES})
add_dependencies(ff_cntrl ${catkin_EXPORTED_TARGETS})

add_library(ff_cntrl_ros src/controller/feed_forward_cntrl/ff_cntrl_ros.cpp)
target_link_libraries(ff_cntrl_ros ff_cntrl ${catkin_LIBRARIES})
add_dependencies(ff_cntrl_ros ${catkin_EXPORTED_TARGETS})

add_library(ekf_ros
 src/ekf/ros/ekf_ros.cpp
 src/ekf/ros/ekf_cb_ros.cpp
 )
target_link_libraries(ekf_ros ekf ${catkin_LIBRARIES})
add_dependencies(ekf_ros ${catkin_EXPORTED_TARGETS})

add_library(target_ekf_ros
 src/ekf/ros/target_ekf_ros.cpp
 src/ekf/ros/target_ekf_cb_ros.cpp
 )
target_link_libraries(target_ekf_ros ekf ${catkin_LIBRARIES})
add_dependencies(target_ekf_ros ${catkin_EXPORTED_TARGETS})

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
