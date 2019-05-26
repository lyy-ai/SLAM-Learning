/****************************
 * 题目：相机视场角比较小（比如手机摄像头）时，一般可以近似为针孔相机成像，三维世界中的直线成像也是直线。
 * 但是很多时候需要用到广角甚至鱼眼相机，此时会产生畸变，三维世界中的直线在图像里会弯曲。因此，需要做去畸变。
 * 给定一张广角畸变图像，以及相机的内参，请完成图像去畸变过程
 *
* 本程序学习目标：
 * 掌握图像去畸变原理
 *
 * 时间：2018.10
****************************/
#include <opencv2/opencv.hpp>

using namespace std;
string image_file = "./test.png";   // 请确保路径正确

int main(int argc, char **argv) {
 
    double k1 = -0.28340811, k2 = 0.07395907;  // 畸变参数
    double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;

    cv::Mat image = cv::imread(image_file, CV_8UC1);   // 图像是灰度图
    int rows = image.rows, cols = image.cols;
    cv::Mat image_undistort = cv::Mat(rows, cols, CV_8UC1);   // 去畸变以后的图
    cv::imshow("image distorted", image);
    // 计算去畸变后图像的内容
    for (int v = 0; v < rows; v++)
        for (int u = 0; u < cols; u++) {

            double u_distorted = 0, v_distorted = 0;
            // 开始代码，注意(u,v)要先转化为归一化坐标
             // u=u/v;
             // v=1;
             //将图像像素坐标转换成相机坐标
             double x=(u-cx)/fx;////x,u自己写成v
             double  y=(v-cy)/fy;
              double r_2=x*x+y*y;
              double x_distorted=x*(1+k1*r_2+k2*r_2*r_2);
              double y_distorted=y*(1+k1*r_2+k2*r_2*r_2);
              //将相机坐标变为像素坐标
              u_distorted=x_distorted*fx+cx;
              v_distorted=y_distorted*fy+cy;
            // 结束代码
          //cout<<"11"<<endl;
            if (u_distorted >= 0 && v_distorted >= 0 && u_distorted < cols && v_distorted < rows) {
                image_undistort.at<uchar>(v, u) = image.at<uchar>((int) v_distorted, (int) u_distorted);
            } else {
                image_undistort.at<uchar>(v, u) = 0;
            }
        }

    cv::imshow("image undistorted", image_undistort);
    cv::waitKey();

    return 0;
}

