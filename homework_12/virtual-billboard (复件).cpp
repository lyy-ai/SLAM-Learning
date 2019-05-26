/****************************
 * 实现虚拟广告牌的效果。
 * 提供两张图，一张是“计算机视觉life”公众号的logo，另外一张是带广告牌的原图，请用单应矩阵实现将原图中广告牌替换为提供的logo的效果。
 * 利用OpenCV函数，通过鼠标点击来选择要替换的广告牌的四个顶点。
 *
* 本程序学习目标：
 * 理解掌握单应矩阵的使用
 *
 * 公众号：计算机视觉life。发布于公众号旗下知识星球：从零开始学习SLAM
 * 时间：2018.11
****************************/

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp> 
#include "opencv2/imgproc/imgproc.hpp" 
#include "opencv2/calib3d/calib3d.hpp"
#include <iostream>
#include <limits>
#include <numeric>

using namespace cv;
using namespace std;

struct userdata{
    Mat im;
    vector<Point2f> points;
};


void mouseHandler(int event, int x, int y, int flags, void* data_ptr)
{
    if  ( event == EVENT_LBUTTONDOWN )
    {
        userdata *data = ((userdata *) data_ptr);
        circle(data->im, Point(x,y),3,Scalar(0,255,255), 5, CV_AA);
        imshow("Image", data->im);
        if (data->points.size() < 4)
        {
            data->points.push_back(Point2f(x,y));
        }
    }
    
}

void showFinal(Mat src1, Mat src2)
 { 
     Mat gray, gray_inv, src1final, src2final;
    cvtColor(src2, gray, CV_BGR2GRAY);
    threshold(gray, gray, 0, 255, CV_THRESH_BINARY); 
      
  //adaptiveThreshold(gray,gray,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,5,4); 
    bitwise_not(gray, gray_inv); 
    src1.copyTo(src1final, gray_inv);
    src2.copyTo(src2final, gray); 
    Mat finalImage = src1final + src2final; 
    namedWindow("output", WINDOW_AUTOSIZE); 
   imshow("output", finalImage);
    cvWaitKey(0); 
}


int main( int argc, char** argv)
{

    // Read in the image.
    //Mat im_src = imread("first-image.jpg");
    Mat im_src = imread("cvlife.jpg");
    Size size = im_src.size();
   
    // Create a vector of points.
    vector<Point2f> pts_src;
    pts_src.push_back(Point2f(0,0));
    pts_src.push_back(Point2f(size.width - 1, 0));
    pts_src.push_back(Point2f(size.width - 1, size.height -1));
    pts_src.push_back(Point2f(0, size.height - 1 ));
    
    

    // Destination image
    //Mat im_dst = imread("times-square.jpg");
    Mat im_dst = imread("ad.jpg");

    
    // Set data for mouse handler
    Mat im_temp = im_dst.clone();
    userdata data;
    data.im = im_temp;


    //show the image
    imshow("Image", im_temp);
    
    cout << "Click on four corners of a billboard and then press ENTER" << endl;
    //set the callback function for any mouse event
    setMouseCallback("Image", mouseHandler, &data);
    waitKey(0);
    
    // ----------  开始你的代码  --------------
      //vector<Point2f> pts_dst;

   Mat m1=Mat(pts_src);
   Mat m2=Mat(data.points);//data->points报错
   Mat status;
   Mat h=findHomography(m2,m1,status,0,3);
   cv::Mat M=cv::getPerspectiveTransform(pts_src,data.points);
   warpPerspective(im_src,im_dst,M,im_temp.size());
  // Mat im_dst0=im_src+im_dst;
    Mat im_dst0 = imread("ad.jpg");
    showFinal(im_dst0,im_dst);
     /*
   imshow("Image", im_dst);
     waitKey(0);
   //填充
   Point PointArray[4];
   PointArray[0] = data.points[0];
     PointArray[1] = data.points[1];
    PointArray[2] = data.points[2];
    PointArray[3] =data.points[3];
   fillConvexPoly(im_dst,PointArray, 4, Scalar(0), CV_AA);*/


    // ----------  结束你的代码  --------------
    // Display image.
  //  imshow("Image", im_dst);
  //  waitKey(0);

    return 0;
}
