cmake_minimum_required( VERSION 2.8 )
project(virtual-billboard )

set( CMAKE_CXX_FLAGS "-std=c++11" )
#include_directories("/usr/include/eigen3")

find_package( OpenCV 3.4.1 REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( virtual-billboard virtual-billboard.cpp )

target_link_libraries( virtual-billboard ${OpenCV_LIBS} )


