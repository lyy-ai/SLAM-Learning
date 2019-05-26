cmake_minimum_required( VERSION 2.8 )
project( epipolar )

set( CMAKE_CXX_FLAGS "-std=c++11" )
include_directories("/usr/include/eigen3")

find_package( OpenCV 3.4.1 REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( epipolar epipolar.cpp )

target_link_libraries( epipolar ${OpenCV_LIBS} )


