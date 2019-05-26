/****************************
 * 题目：现有一个运动着的相机拍摄的连续两张图片，其中特征点匹配部分已经完成。
 * 请根据两帧图像对应的匹配点计算基础矩阵，并利用该矩阵绘制出前10个特征点对应的极线。
 *
* 本程序学习目标：
 * 理解掌握对极约束的原理
 * 熟悉OpenCV编程
 *
 * 公众号：计算机视觉life。发布于公众号旗下知识星球：从零开始学习SLAM
 * 时间：2018.11
****************************/
#include<iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace cv;

/*
//写的鸡巴，传入的k有啥用
  Point2d pixel2cam( vector< KeyPoint> &kp,Mat &k)
    {
        double fx=520.9,cx=325.1,fy=521.0,cy=249.7;
        Point2d pt;
        pt.x=(kp[0].pt-cx)/fx;
        pt.y=(kp[1].pt-cy)/fy;
        return pt;
    }
*/
      Point2d pixel2cam( const Point2d &p,Mat &K)
    {
       
        return Point2d(
                       (p.x-K.at<double>(0,2))/K.at<double>(0,0),
                       (p.y-K.at<double>(1,2))/K.at<double>(1,1)

                      );
    }

int main( int argc, char** argv )
{

    Mat rgb1 = imread( "./rgb1.ppm");
    Mat rgb2 = imread( "./rgb2.ppm");

    //Ptr<FeatureDetector> detector;
   // Ptr<DescriptorExtractor> descriptor;
    
   // detector = FeatureDetector::create("ORB");//opencv 2.4.9版本
   // descriptor = DescriptorExtractor::create("ORB");

    

   //初始化
    vector< KeyPoint > kp1, kp2;
    Mat descriptors_1,descriptors_2;
    Ptr<ORB> orb=ORB::create(500,1.2f,8,31,0,2,ORB::HARRIS_SCORE,31,20);

   //检测 fast角点位置
   // detector->detect( rgb1, kp1 );
   // detector->detect( rgb2, kp2 );
       orb->detect( rgb1, kp1 );
    orb->detect( rgb2, kp2 );

    // 计算描述子
    Mat desp1, desp2;
   // descriptor->compute( rgb1, kp1, desp1 );
    //descriptor->compute( rgb2, kp2, desp2 );
     orb->compute( rgb1, kp1, desp1 );
    orb->compute( rgb2, kp2, desp2 );

    // 匹配描述子
    vector< DMatch > matches;
    BFMatcher matcher;
    matcher.match( desp1, desp2, matches );
    cout<<"Find total "<<matches.size()<<" matches."<<endl;

    // 筛选匹配对
    vector< DMatch > goodMatches;
    double minDis = 9999;
    for ( size_t i=0; i<matches.size(); i++ )
    {
        if ( matches[i].distance < minDis )
            minDis = matches[i].distance;
    }

    for ( size_t i=0; i<matches.size(); i++ )
    {
        if (matches[i].distance < 10*minDis)
            goodMatches.push_back( matches[i] );
    }


    vector< Point2f > pts1, pts2;
    for (size_t i=0; i<goodMatches.size(); i++)
    {
        pts1.push_back(kp1[goodMatches[i].queryIdx].pt);
        pts2.push_back(kp2[goodMatches[i].trainIdx].pt);
    }

    // 请先计算基础矩阵并据此绘制出前10个匹配点对应的对极线，可以调用opencv函数
    // ----------- 开始你的代码 --------------//
    //计算基础矩阵
    Mat fundamental_matrix_8;
    fundamental_matrix_8=findFundamentalMat(pts1,pts2,CV_FM_8POINT);
    cout<<"fundamental_matrix_8= "<<fundamental_matrix_8<<endl;

    Mat fundamental_matrix_7;
    fundamental_matrix_7=findFundamentalMat(pts1,pts2,CV_FM_7POINT);
    cout<<"fundamental_matrix_7= "<<fundamental_matrix_7<<endl;

    vector<uchar> inliers(pts1.size(),0);//匹配状态（内点或者外点）
    Mat fundamental_ransac=findFundamentalMat(pts1,pts2,inliers,FM_RANSAC,1.0,0.99);
     cout<<"fundamental_ransac= "<<fundamental_ransac<<endl;

     

   //计算对应点的外极线epilines是一个三元组(a,b,c)，表示点在另一视图中对应的外极线ax+by+c=0;
    std::vector<cv::Vec<float, 3>> epilines1, epilines2;
    //std::vector<cv::Point2f> points1,points2;//这肯定不行啊，是空的啊
	cv::computeCorrespondEpilines(pts1, 1, fundamental_matrix_8, epilines1);
    cv::computeCorrespondEpilines(pts2, 2,  fundamental_matrix_8, epilines2);

	 cv::RNG& rng = theRNG();
	 for (int i = 0; i < 10; i++)
	{
		Scalar color = Scalar(rng(256), rng(256), rng(256));//随机产生颜色
		circle(rgb2, pts2[i], 5, color);//在视图2中把关键点用圆圈画出来，然后再绘制在对应点处的外极线
		line(rgb2, Point(0, -epilines1[i][2] / epilines1[i][1]), Point(rgb2.cols, -(epilines1[i][2] + epilines1[i][0] * rgb2.cols) / epilines1[i][1]), color);
        //绘制外极线的时候，选择两个点，一个是x=0处的点，一个是x为图片宽度处cols
		 circle(rgb1, pts1[i], 4, color);
		 line(rgb1, cv::Point(0, -epilines2[i][2] / epilines2[i][1]), cv::Point(rgb1.cols, -(epilines2[i][2] + epilines2[i][0] * rgb1.cols) / epilines2[i][1]), color);
		
 	}

     //计算本质矩阵E
     
     Point2d principal_point(325.1,249.7);//光心
     int focal_length=521;//焦距
     Mat essential_matrix;
     essential_matrix=findEssentialMat(pts1,pts2,focal_length,principal_point,RANSAC);//这个函数紧紧在opencv3.0以上可以用
     Mat E_mat=findEssentialMat(pts1,pts2,1,Point2d(0,0),RANSAC,0.999,1.f);
     cout<<"essential_matrix= "<<essential_matrix<<endl;
     cout<<"E_mat= "<<E_mat<<endl;

     //计算单应矩阵H
     Mat homography_matrix;
     homography_matrix=findHomography(pts1,pts2,RANSAC,3,noArray(),2000,0.99);
     cout<<"homography_matrix= "<<homography_matrix<<endl;

     //从本质矩阵中恢复旋转矩阵和平移矩阵
     //Eigen::Vector3d t;
     //Eigen::Matrix3d R;//类型不对不能带入recoverPose中
     Mat R,t;
     recoverPose(essential_matrix,pts1,pts2,R,t,focal_length,principal_point);//3.0
     cout<<"R= "<<R<<endl;
     cout<<"t= "<<t<<endl;

    Mat t_x=(Mat_<double>(3,3)<<0,-t.at<double>(2,0),t.at<double>(1,0),t.at<double>(2,0),0,-t.at<double>(0,0),-t.at<double>(1,0),t.at<double>(0,0),0);
   

    /////////从基础矩阵中恢复旋转矩阵和平移矩阵
     Mat R0,t0,R1,t1,R2,t2;
     recoverPose(fundamental_matrix_8,pts1,pts2,R0,t0,focal_length,principal_point);//3.0
     recoverPose(fundamental_matrix_7,pts1,pts2,R1,t1,focal_length,principal_point);//3.0
     recoverPose(fundamental_ransac,pts1,pts2,R2,t2,focal_length,principal_point);//3.0
     //cout<<"R0= "<<R0<<endl;
     //cout<<"t0= "<<t0<<endl;

     //验证E=t^R
    Mat t_x0=(Mat_<double>(3,3)<<0,-t0.at<double>(2,0),t0.at<double>(1,0),t0.at<double>(2,0),0,-t0.at<double>(0,0),-t0.at<double>(1,0),t0.at<double>(0,0),0);
    //cout<<"t^R= "<<t_x*R<<endl;
    Mat t_x1=(Mat_<double>(3,3)<<0,-t1.at<double>(2,0),t1.at<double>(1,0),t1.at<double>(2,0),0,-t1.at<double>(0,0),-t1.at<double>(1,0),t1.at<double>(0,0),0);
    Mat t_x2=(Mat_<double>(3,3)<<0,-t2.at<double>(2,0),t2.at<double>(1,0),t2.at<double>(2,0),0,-t2.at<double>(0,0),-t2.at<double>(1,0),t2.at<double>(0,0),0);



     //验证对极约束
     Mat K=(Mat_<double>(3,3)<<520.9,0,325.1,0,521.0,249.7,0,0,1);
     for(DMatch m:goodMatches)
     {

        // 将像素坐标转换至相机坐标
        Point2d pt1=pixel2cam(kp1[m.queryIdx].pt,K);
        Mat x1=(Mat_<double>(3,1)<<pt1.x,pt1.y,1);
        Point2d pt2=pixel2cam(kp2[m.trainIdx].pt,K);
        Mat x2=(Mat_<double>(3,1)<<pt2.x,pt2.y,1);
        Mat d=x2.t()*t_x0*R0*x1;
        cout<<"epipolar constraint0= "<<d<<endl;
     }
    
    cout<<endl;

     for(DMatch m:goodMatches)
     {

        // 将像素坐标转换至相机坐标
        Point2d pt1=pixel2cam(kp1[m.queryIdx].pt,K);
        Mat x1=(Mat_<double>(3,1)<<pt1.x,pt1.y,1);
        Point2d pt2=pixel2cam(kp2[m.trainIdx].pt,K);
        Mat x2=(Mat_<double>(3,1)<<pt2.x,pt2.y,1);
        Mat d=x2.t()*t_x1*R1*x1;
        cout<<"epipolar constraint1= "<<d<<endl;
     }
    cout<<endl;

     for(DMatch m:goodMatches)
     {

        // 将像素坐标转换至相机坐标
        Point2d pt1=pixel2cam(kp1[m.queryIdx].pt,K);
        Mat x1=(Mat_<double>(3,1)<<pt1.x,pt1.y,1);
        Point2d pt2=pixel2cam(kp2[m.trainIdx].pt,K);
        Mat x2=(Mat_<double>(3,1)<<pt2.x,pt2.y,1);
        Mat d=x2.t()*t_x2*R2*x1;
        cout<<"epipolar constraint2= "<<d<<endl;
     }
      cout<<endl;
       for(DMatch m:goodMatches)
     {

        // 将像素坐标转换至相机坐标
        Point2d pt1=pixel2cam(kp1[m.queryIdx].pt,K);
        Mat x1=(Mat_<double>(3,1)<<pt1.x,pt1.y,1);
        Point2d pt2=pixel2cam(kp2[m.trainIdx].pt,K);
        Mat x2=(Mat_<double>(3,1)<<pt2.x,pt2.y,1);
        Mat d=x2.t()*t_x*R*x1;
        cout<<"epipolar constraint= "<<d<<endl;
     }
   

    // ----------- 结束你的代码 --------------//
    imshow("epiline1", rgb2);
    imshow("epiline2", rgb1);
    waitKey(0);
    return 0;
}
