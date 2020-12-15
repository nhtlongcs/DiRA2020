cmake_minimum_required(VERSION 3.18)
project(sign_detect LANGUAGES CUDA CXX)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport roscpp common
                                        sensor_msgs dynamic_reconfigure)
find_package(CUDAToolkit REQUIRED)
find_package(TensorRT REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)


generate_dynamic_reconfigure_options(
  cfg/Sign.cfg
)

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
