cmake_minimum_required(VERSION 3.18)
project(camera_record)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/record.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)