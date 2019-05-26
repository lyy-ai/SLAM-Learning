cmake_minimum_required( VERSION 2.8 )
project( rotationTransforms )
set(CMAKE_CXX_FLAGS "-std=c++11")
# 添加Eigen头文件
include_directories( "/usr/include/eigen3" )

add_executable( rotationTransforms rotationTransforms.cpp )
