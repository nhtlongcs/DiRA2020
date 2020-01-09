cmake_minimum_required(VERSION 2.8.3)
project(lane_detect)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  common
  sensor_msgs
  cds_msgs
  dynamic_reconfigure
)


generate_dynamic_reconfigure_options(
  cfg/Lane.cfg
)

set(OpenCV_STATIC OFF)
find_package(OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES api_${PROJECT_NAME}
  CATKIN_DEPENDS roscpp cds_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(api_${PROJECT_NAME}
  src/laneline.cpp
  src/lane_detect.cpp
)

add_dependencies(api_${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_executable(${PROJECT_NAME}_node src/lane_detect_node.cpp)

target_link_libraries(
  ${PROJECT_NAME}_node
  api_${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  ${api_common_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

install(TARGETS api_${PROJECT_NAME} ${ADDITIONAL_EXECUTABLES}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
