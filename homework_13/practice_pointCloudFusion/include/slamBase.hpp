# pragma once //保证头文件只被编译一次

#include <iostream>

// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// pcl
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.h>

using namespace std;
using namespace cv;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// camera instrinsic parameters
struct CAMERA_INTRINSIC_PARAMETERS
{
    double fx, fy, cx, cy, scale;
};

struct FRAME
{
    cv::Mat rgb, depth;
};

PointCloud::Ptr image2PointCloud(Mat rgb, Mat depth, CAMERA_INTRINSIC_PARAMETERS camera);
PointCloud::Ptr pointCloudFusion( PointCloud::Ptr &original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS camera );
//PointCloud::Ptr pointCloudFusion( PointCloud::Ptr &original, FRAME& newFrame, vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> T, CAMERA_INTRINSIC_PARAMETERS camera );
void readCameraTrajectory(string camTransFile, vector<Eigen::Isometry3d> &poses);
//void readCameraTrajectory(string camTransFile, vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses);
