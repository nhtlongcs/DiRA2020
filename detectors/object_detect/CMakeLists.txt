cmake_minimum_required(VERSION 2.8.3)
project(object_detect)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cds_msgs
  common
  roscpp
  image_transport
  cv_bridge
  dynamic_reconfigure
)


generate_dynamic_reconfigure_options(
  cfg/ObjectDetect.cfg
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES object_detect
  CATKIN_DEPENDS cds_msgs common roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(api_${PROJECT_NAME}
  src/object_detect.cpp
)

add_dependencies(api_${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node src/object_detect_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  api_${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)