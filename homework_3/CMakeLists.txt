cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_FLAGS "-std=c++11")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
#include_directories("/usr/include/eigen3")
add_executable(newFeather cppNewFeather.cpp)
target_link_libraries(newFeather ${OpenCV_LIBS})



