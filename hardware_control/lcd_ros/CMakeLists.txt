cmake_minimum_required(VERSION 3.18)
project(lcd_ros)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
catkin_package(CATKIN_DEPENDS roscpp std_msg)

add_executable(${PROJECT_NAME}_node src/lcd_ros.cpp src/Hal.cpp src/LCDI2C.cpp)
target_include_directories(${PROJECT_NAME}_node PUBLIC include
                                                       ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

# add_dependencies(hal ${catkin_EXPORTED_TARGETS})

install(
  TARGETS ${PROJECT_NAME}_node
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
