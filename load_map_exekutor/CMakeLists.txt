cmake_minimum_required(VERSION 2.8.3)
project(load_map_exekutor)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  action_exekutor
  cognidrive_ros
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
  LIBRARIES load_map_exekutor
  CATKIN_DEPENDS action_exekutor cognidrive_ros std_msgs
#  DEPENDS system_lib
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

## Declare a cpp library
add_library(load_map_exekutor
  src/load_map_exekutor.cpp
)
add_dependencies(load_map_exekutor cognidrive_ros_generated_msgs)

target_link_libraries(load_map_exekutor 
    ${catkin_LIBRARIES}
)

add_executable (test_load_map src/test_load_map.cc)
target_link_libraries (test_load_map 
    ${catkin_LIBRARIES}
    load_map_exekutor
    peiskernel_mt
)