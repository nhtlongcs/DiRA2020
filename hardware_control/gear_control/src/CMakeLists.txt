add_executable(${PROJECT_NAME}_node JHPWMPCA9685.cpp gear_control.cpp
                                    gear_control_node.cpp)
target_include_directories(
  ${PROJECT_NAME}_node PUBLIC ${PROJECT_SOURCE_DIR}/include
                              ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_node PRIVATE ${catkin_LIBRARIES} i2c)
target_compile_features(${PROJECT_NAME}_node PRIVATE cxx_std_17)

add_executable(gear_debug gear_debug.cpp)
target_include_directories(gear_debug PUBLIC ${PROJECT_SOURCE_DIR}/include
                                             ${catkin_INCLUDE_DIRS})
target_link_libraries(gear_debug PRIVATE ${catkin_LIBRARIES})

target_compile_features(gear_debug PRIVATE cxx_std_17)
