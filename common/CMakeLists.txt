cmake_minimum_required(VERSION 3.18)
project(common LANGUAGES CUDA CXX)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge)

find_package(OpenCV REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(TensorRT REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/common/libcommon.cpp
  src/common/lineparams.cpp
  src/trt/logger.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC include   ${catkin_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  CUDA::cudart
  opencv_imgproc
  TensorRT::nvinfer_plugin
  TensorRT::nvinfer
  TensorRT::nvonnxparser
  stdc++fs)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17 cuda_std_14)

install(TARGETS ${PROJECT_NAME} ${ADDITIONAL_EXECUTABLES}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)
