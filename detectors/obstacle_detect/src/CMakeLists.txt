add_library(${PROJECT_NAME} ${PROJECT_NAME}.cpp)
target_include_directories(
  ${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS}
                         ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${catkin_LIBRARIES} opencv_core)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

add_executable(${PROJECT_NAME}_node ${PROJECT_NAME}_node.cpp)
target_include_directories(${PROJECT_NAME}_node
                           PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(${PROJECT_NAME}_node PRIVATE ${PROJECT_NAME}
                                                   ${catkin_LIBRARIES})
target_compile_features(${PROJECT_NAME}_node PRIVATE cxx_std_17)

add_subdirectory(displays)
