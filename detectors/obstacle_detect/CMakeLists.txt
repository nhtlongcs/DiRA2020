cmake_minimum_required(VERSION 3.0.2)
project(obstacle_detect)

find_package(catkin REQUIRED COMPONENTS cds_msgs dynamic_reconfigure nodelet
                                        roscpp rviz geometry_msgs std_msgs)
find_package(OpenCV 4 REQUIRED COMPONENTS core)
find_package(Qt5 5 REQUIRED Core Widgets)

generate_dynamic_reconfigure_options(cfg/obstacle_detect.cfg)

catkin_package(
  # INCLUDE_DIRS include LIBRARIES obstacle_detect CATKIN_DEPENDS
  # dynamic_reconfigure nodelet roscpp  DEPENDS Armadillo
)

add_subdirectory(src)

#
# Install nodes
#
# install(TARGETS obstacle_detector_node RUNTIME DESTINATION
# ${CATKIN_PACKAGE_BIN_DESTINATION})

#
# Install header files
#
# install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION
# ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#
# Install nodelet and rviz plugins description
#
# install(FILES nodelet_plugins.xml rviz_plugins.xml DESTINATION
# ${CATKIN_PACKAGE_SHARE_DESTINATION})

#
# Install launch files
#
# install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
