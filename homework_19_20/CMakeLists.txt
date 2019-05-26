cmake_minimum_required( VERSION 2.8 )
project( ICP )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

#set (OpenCV_DIR "/home/cxl/Downloads/opencv-3.4.5/build")
find_package( OpenCV  REQUIRED )
# find_package( OpenCV REQUIRED ) # use this if in OpenCV2
FIND_PACKAGE(Pangolin REQUIRED)
FIND_PACKAGE(Sophus REQUIRED)



include_directories(
        ${Sophus_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        "/usr/include/eigen3/"
)

add_executable( ${PROJECT_NAME} icp.cpp )
target_link_libraries( ${PROJECT_NAME}
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES}
        ${OpenCV_LIBS}
       # /usr/local/lib/libSophus.so
)
