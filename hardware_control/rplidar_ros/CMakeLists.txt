cmake_minimum_required(VERSION 3.18)
project(rplidar_ros)

find_package(catkin REQUIRED COMPONENTS roscpp rosconsole sensor_msgs)

catkin_package()

add_subdirectory(src)

install(
  DIRECTORY launch rviz sdk
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
