cmake_minimum_required(VERSION 2.8.3)
project(lane_detect)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  cds_msgs
  dynamic_reconfigure
)


generate_dynamic_reconfigure_options(
  cfg/lane.cfg
  cfg/carcontrol.cfg
  cfg/detectobject.cfg
  cfg/planning.cfg
)

set(OpenCV_STATIC OFF)
find_package(  OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES api_lane_detection
  CATKIN_DEPENDS roscpp cds_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(api_lane_detection
  src/laneline.cpp
  src/utils.cpp
  src/detectlane.cpp
  src/detectobject.cpp
  src/carcontrol.cpp
  src/planning.cpp
)

add_dependencies(api_lane_detection ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_executable(lane_detect src/main.cpp)

target_link_libraries(
  lane_detect
  api_lane_detection
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
)

add_dependencies(lane_detect ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

install(TARGETS api_lane_detection ${ADDITIONAL_EXECUTABLES}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
