
# 增加PCL库的依赖
FIND_PACKAGE( PCL  REQUIRED )

FIND_PACKAGE( OpenCV 3.4.1 REQUIRED )

include_directories("/usr/include/eigen3")
FIND_PACKAGE(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

#INCLUDE_DIRECTORIES( ${PROJECT_SOURSE_DIR}/include )
INCLUDE_DIRECTORIES( "/home/lyy/from_0_to_1_for_slam/homework_13/practice_pointCloudFusion/include" )

ADD_LIBRARY(slambase_shared SHARED slamBase.cpp)
target_link_libraries(slambase_shared  ${Sophus_LIBRARIES})
TARGET_LINK_LIBRARIES(slambase_shared
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES} )


ADD_EXECUTABLE( pointCloudFusion pointCloudFusion.cpp )
TARGET_LINK_LIBRARIES(pointCloudFusion
        slambase_shared
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        )




