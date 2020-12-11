find_package(CUDAToolkit REQUIRED)
find_package(TensorRT REQUIRED)
find_package(common REQUIRED)

add_executable(lanenet_benchmark lanenet.cpp lanenet_demo.cpp)
#add_dependencies(lanenet_benchmark ${catkin_EXPORTED_TARGETS})
target_include_directories(lanenet_benchmark PUBLIC ../include
                                                    ${catkin_INCLUDE_DIRS})
target_link_libraries(
  lanenet_benchmark
  PRIVATE ${common_LIBRARIES}
          ${catkin_LIBRARIES}
          CUDA::cudart
          opencv_imgproc
          TensorRT::nvinfer_plugin
          TensorRT::nvinfer
          TensorRT::nvonnxparser
          stdc++fs)
target_compile_features(lanenet_benchmark PRIVATE cxx_std_17 cuda_std_17)

