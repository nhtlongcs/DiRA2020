add_library(yolov4tiny yolov4_tiny.cpp logger.cpp)

target_include_directories(yolov4tiny PUBLIC ../include ${OpenCV_INCLUDE_DIRS}
                                             ${CUDAToolkit_INCLUDE_DIRS})
target_link_libraries(yolov4tiny PRIVATE)
target_compile_features(yolov4tiny PRIVATE cxx_std_17 cuda_std_14)

add_executable(yolo-benchmark yolo_benchmark.cpp)
target_include_directories(yolo-benchmark PUBLIC ../include
                                                 ${catkin_INCLUDE_DIRS})
target_link_libraries(
  yolo-benchmark
  PRIVATE yolov4tiny
          ${catkin_LIBRARIES}
          CUDA::cudart
          opencv_imgproc
          TensorRT::nvinfer_plugin
          TensorRT::nvonnxparser
          stdc++fs)
target_compile_features(yolo-benchmark PRIVATE cxx_std_17 cuda_std_14)
