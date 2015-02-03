cmake_minimum_required(VERSION 2.8.3)
project(miradock_exekutor)

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
   LIBRARIES miradock_exekutor
   CATKIN_DEPENDS action_exekutor cognidrive_ros std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library (miradock_exekutor
  src/miradock_exekutor.cpp
)

add_dependencies (miradock_exekutor
    cognidrive_ros_generated_messages_cpp
)

add_executable (test_miradock
    src/test_miradock.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(miradock_exekutor
  ${catkin_LIBRARIES}
)

target_link_libraries(test_miradock
  ${catkin_LIBRARIES}
  miradock_exekutor
  peiskernel_mt
)

install (FILES docking_stations.txt DESTINATION "~")
