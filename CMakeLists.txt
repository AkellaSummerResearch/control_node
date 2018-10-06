cmake_minimum_required(VERSION 2.8.3)
project(control_node)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mavros_msgs
)

catkin_package(
  DEPENDS Eigen
  mavros_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(manualWaypointGen src/manualWaypointGen.cpp)
target_link_libraries(manualWaypointGen ${catkin_LIBRARIES})

add_executable(control src/control.cpp)
target_link_libraries(control ${catkin_LIBRARIES})

add_executable(waypoint src/waypoint.cpp)
target_link_libraries(waypoint ${catkin_LIBRARIES})
