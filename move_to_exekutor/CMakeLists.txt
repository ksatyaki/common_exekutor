cmake_minimum_required(VERSION 2.8.3)
project(move_to_exekutor)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  action_exekutor
  actionlib
  actionlib_msgs
  move_base_msgs
  roscpp
  simple_service
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES move_to_exekutor move_to_simple_exekutor
  CATKIN_DEPENDS action_exekutor actionlib actionlib_msgs roscpp simple_service std_msgs move_base_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

######################################

add_library(move_to_exekutor
  src/move_to_exekutor.cpp
)

target_link_libraries(move_to_exekutor
  ${catkin_LIBRARIES}
)

######################################

add_library(move_to_simple_exekutor
    src/move_to_simple_exekutor.cpp
)

target_link_libraries(move_to_simple_exekutor
  ${catkin_LIBRARIES}
)
  
######################################
