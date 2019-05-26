cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories("/usr/include/eigen3")
add_executable(eigen Eigen.cpp)
#target_link_libraries(namePicture ${OpenCV_LIBS})//eigen不需要，因为他只有头文件



