cmake_minimum_required(VERSION 2.8.3)
project(sign_detect)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  cds_msgs
  common
  dynamic_reconfigure
)

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/Sign.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES signdetect
  CATKIN_DEPENDS cv_bridge image_transport roscpp common cds_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(api_${PROJECT_NAME}
  src/signdetect.cpp
)

add_dependencies(api_${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_executable(${PROJECT_NAME}_node src/signdetect_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

target_link_libraries(
  ${PROJECT_NAME}_node
  api_${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

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
