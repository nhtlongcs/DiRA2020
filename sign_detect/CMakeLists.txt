cmake_minimum_required(VERSION 3.18)
project(sign_detect LANGUAGES CUDA CXX)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport roscpp
                                        sensor_msgs)
find_package(CUDAToolkit REQUIRED)
find_package(TensorRT REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc highgui)

# TODO ???
catkin_package(
  # INCLUDE_DIRS include LIBRARIES sign_detect CATKIN_DEPENDS cv_bridge
  # image_transport roscpp sensor_msgs DEPENDS CUDAToolkit TensorRT OpenCV
)

add_subdirectory(src)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
