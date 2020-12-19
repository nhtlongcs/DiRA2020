add_library(rplidar_sdk net_serial.cpp net_socket.cpp rplidar_driver.cpp
                        thread.cpp timer.cpp)
target_include_directories(rplidar_sdk PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_compile_features(rplidar_sdk PRIVATE cxx_std_17)
