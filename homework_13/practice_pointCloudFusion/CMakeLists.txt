CMAKE_MINIMUM_REQUIRED( VERSION 2.8 ) #设定版本

PROJECT(PointCloud) #设定工程名
SET( CMAKE_CXX_FLAGS "-std=c++11" )#设定编译器
SET(CMAKE_BUILD_TYPE Debug)

include_directories("/usr/include/eigen3")

FIND_PACKAGE(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

#设定可执行二进制文件的目录
SET( EXECUTABLE_OUTPUT_PATH "/home/lyy/from_0_to_1_for_slam/homework_13/practice_pointCloudFusion/bin") 
	
#设定存放编译出来的库文件的目录
SET( LIBRARY_OUTPUT_PATH "/home/lyy/from_0_to_1_for_slam/homework_13/practice_pointCloudFusion/lib")

#设定头文件目录
INCLUDE_DIRECTORIES( "/home/lyy/from_0_to_1_for_slam/homework_13/practice_pointCloudFusion/include")

#并且把该目录设为连接目录
LINK_DIRECTORIES( "/home/lyy/from_0_to_1_for_slam/homework_13/practice_pointCloudFusion/lib")
	
#增加子文件夹，也就是进入源代码文件夹继续构建
ADD_SUBDIRECTORY( "/home/lyy/from_0_to_1_for_slam/homework_13/practice_pointCloudFusion/src")

