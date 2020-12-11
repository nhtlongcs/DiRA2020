cmake_minimum_required(VERSION 3.18)
project(lanenet LANGUAGES CUDA CXX)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(catkin REQUIRED COMPONENTS common cv_bridge image_transport roscpp
                                        sensor_msgs)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc highgui)

# TODO ???
catkin_package(
   INCLUDE_DIRS include LIBRARIES sign_detect CATKIN_DEPENDS cv_bridge
   image_transport roscpp sensor_msgs DEPENDS CUDAToolkit TensorRT OpenCV
)

include_directories(
  include
  ${common_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
