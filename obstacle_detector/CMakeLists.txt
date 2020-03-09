cmake_minimum_required(VERSION 2.8.3)
project(obstacle_detector)

add_compile_options(-std=c++17)

find_package(
  catkin REQUIRED
  COMPONENTS cds_msgs
             dynamic_reconfigure
             nodelet
             pcl_conversions
             pcl_ros
             roscpp
             tf2_ros
             visualization_msgs)
find_package(Armadillo REQUIRED)

generate_dynamic_reconfigure_options(cfg/PclDetector.cfg cfg/LidarDetector.cfg)

catkin_package(INCLUDE_DIRS include)
include_directories(include ${catkin_INCLUDE_DIRS} ${ARMADILLO_INCLUDE_DIRS})

add_executable(PclDetectorNode src/node/PclDetectorNode.cpp src/PclDetector.cpp)
target_link_libraries(PclDetectorNode ${catkin_LIBRARIES})

add_executable(LidarDetectorNode src/node/LidarDetectorNode.cpp
                                 src/LidarDetector.cpp)
target_link_libraries(LidarDetectorNode ${catkin_LIBRARIES}
                      ${ARMADILLO_LIBRARIES})
