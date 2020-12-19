add_subdirectory(sdk)

add_library(${PROJECT_NAME} rplidar_ros.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE rplidar_sdk ${catkin_LIBRARIES})
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

add_executable(${PROJECT_NAME}_node rplidar_ros_node.cpp)
target_include_directories(${PROJECT_NAME}_node
                           PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(${PROJECT_NAME}_node PRIVATE ${PROJECT_NAME}
                                                   ${catkin_LIBRARIES})
target_compile_features(${PROJECT_NAME}_node PRIVATE cxx_std_17)
