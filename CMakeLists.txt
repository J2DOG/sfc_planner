cmake_minimum_required(VERSION 3.0.2)
project(sfc_planner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin and system dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  mavros_msgs
  tracking_controller
)

find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(OMPL REQUIRED ompl)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

###################################
## catkin package declaration    ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  mavros_msgs
  tracking_controller
)

## Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

## Node executable
add_executable(sfc_navigation_node
  src/sfc_navigation.cpp
)

## Ensure messages/services are built before this target
add_dependencies(sfc_navigation_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## Link libraries
target_link_libraries(sfc_navigation_node
  ${catkin_LIBRARIES}
  ${OMPL_LDFLAGS}
  ${YAML_CPP_LIBRARIES}
  OsqpEigen::OsqpEigen
)