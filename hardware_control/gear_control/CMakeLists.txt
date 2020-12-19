cmake_minimum_required(VERSION 3.18)
project(gear_control)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure roscpp std_msgs)
generate_dynamic_reconfigure_options(cfg/gear_debug.cfg)

catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS
  dynamic_reconfigure
  roscpp
  std_msgs
  DEPENDS
  # system_lib
)

add_subdirectory(src)
