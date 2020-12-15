#
# Build rviz plugins
#
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)
add_library(
  ${PROJECT_NAME}_gui
  circle_visual.cpp
  obstacle_detect_panel.cpp
  obstacles_display.cpp
  segment_visual.cpp
  ${CMAKE_SOURCE_DIR}/include/displays/circle_visual.hpp
  ${CMAKE_SOURCE_DIR}/include/displays/obstacle_detect_panel.hpp
  ${CMAKE_SOURCE_DIR}/include/displays/obstacles_display.hpp
  ${CMAKE_SOURCE_DIR}/include/displays/segment_visual.hpp)

target_include_directories(
  ${PROJECT_NAME}_gui
  PUBLIC ${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS}
         ${Qt5Core_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_gui PRIVATE ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_gui ${catkin_EXPORTED_TARGETS})
target_compile_features(${PROJECT_NAME}_gui PRIVATE cxx_std_17)

# Install libraries
#
install(TARGETS ${PROJECT_NAME}_gui
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
