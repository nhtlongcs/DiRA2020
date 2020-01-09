cmake_minimum_required(VERSION 2.8.3)
project(planning)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  lane_detect
  object_detect
  roscpp
  sign_detect
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES planning
  CATKIN_DEPENDS lane_detect object_detect roscpp sign_detect
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/planning.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node src/planning_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)