cmake_minimum_required(VERSION 2.8.3)
project(move_to_exekutor)

find_package(catkin REQUIRED COMPONENTS
  action_exekutor
  actionlib
  actionlib_msgs
  move_base_msgs
  roscpp
  simple_service
  std_msgs
  cam_interface
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES move_to_exekutor move_to_simple_exekutor
  CATKIN_DEPENDS action_exekutor actionlib actionlib_msgs roscpp simple_service std_msgs move_base_msgs
  DEPENDS system_lib
)

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
