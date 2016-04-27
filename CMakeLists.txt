cmake_minimum_required(VERSION 2.8.3)
project(drone_process)
add_definitions(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  droneMsgsROS
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES drone_process
  CATKIN_DEPENDS roscpp std_msgs droneMsgsROS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(
  include
  src
  test
)

## Declare a cpp library
add_library(drone_process src/drone_process.cpp include/drone_process.h)
add_dependencies(drone_process ${catkin_EXPORTED_TARGETS})
target_link_libraries(drone_process ${catkin_LIBRARIES})
TARGET_LINK_LIBRARIES(drone_process ${STATIC_LIBS} -lboost_system -lboost_chrono -lboost_thread)

add_executable(mainA test/claseA.cpp)
target_link_libraries(mainA drone_process)
target_link_libraries(mainA ${catkin_LIBRARIES})

add_executable(mainB test/mainB.cpp)
target_link_libraries(mainB drone_process)
target_link_libraries(mainB ${catkin_LIBRARIES})