cmake_minimum_required(VERSION 3.18)
project(lane_detect)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  geometry_msgs
  roscpp
  common
  sensor_msgs
  cds_msgs
  message_generation
  dynamic_reconfigure
)
find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/Lane.cfg
)

include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp cds_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/laneline.cpp
  src/${PROJECT_NAME}/lane_detect.cpp
  src/${PROJECT_NAME}/lane_width_calculator.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node src/lane_detect_node.cpp)

target_link_libraries(
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  ${common_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
