cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_FLAGS "-std=c++11")
find_package(OpenCV REQUIRED)
add_executable(cross cross.cpp)
add_executable(classCross classCross.cpp)





