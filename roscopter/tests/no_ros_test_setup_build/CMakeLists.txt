cmake_minimum_required(VERSION 2.8.3)
project(roscopter)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(GTest REQUIRED)

include_directories(include
  ${EIGEN3_INCLUDE_DIRS} 
  ${GTEST_INCLUDE_DIRS}
  )

add_executable(rtk_compassing_test 
  ../rtk_compassing/state_test.cpp
#   ../rtk_compassing/covariance_test.cpp
  ../rtk_compassing/compassing_utilities.cpp
  compassing_setup.cpp
  ../../include/tests/rtk_compassing_test.h
  )
target_link_libraries(rtk_compassing_test ekf ${catkin_LIBRARIES} ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} pthread)

add_library(ekf
  ../../src/ekf/state.cpp
  ../../src/ekf/dynamics.cpp
  ../../src/ekf/ekf.cpp
  ../../src/ekf/meas.cpp
  ../../src/ekf/log.cpp
  ../../src/ekf/sensors.cpp
  ../../include/ekf/state.h
  ../../include/ekf/meas.h
)
# target_include_directories(ekf PUBLIC include ${catkin_INCLUDE_DIRS})
# target_link_libraries(ekf ${YAML_CPP_LIBRARIES} stdc++fs)

