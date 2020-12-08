add_library(yolo_lib yolov4_tiny.cpp logger.cpp)

target_include_directories(yolo_lib PUBLIC ../include ${OpenCV_INCLUDE_DIRS}
                                           ${CUDAToolkit_INCLUDE_DIRS})
target_link_libraries(yolo_lib PRIVATE)
target_compile_features(yolo_lib PRIVATE cxx_std_17 cuda_std_17)

add_executable(yolo-onnx-trt yolo_onnx_trt.cpp)
target_include_directories(yolo-onnx-trt PUBLIC ../include
                                                 ${catkin_INCLUDE_DIRS})
target_link_libraries(
  yolo-onnx-trt
  PRIVATE yolo_lib
          ${catkin_LIBRARIES}
          CUDA::cudart
          opencv_imgproc
          TensorRT::nvinfer_plugin
          TensorRT::nvonnxparser
          stdc++fs)
target_compile_features(yolo-onnx-trt PRIVATE cxx_std_17 cuda_std_17)
