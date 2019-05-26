
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

int main()
{
    cv::String path="/home/lyy/from_0_to_1_for_slam/homework_1/data/";//待处理图片路径
    cv::String dest="/home/lyy/from_0_to_1_for_slam/homework_1/dst/";//保存处理后的图片路径
    cv::String savefilename;
    vector<cv::String> filenames;
    Mat srcImg,dstImg;

    cv::glob(path,filenames);//glob 寻找与模式匹配的文件路径
   for(int i=0;i<filenames.size();++i)
    {
        srcImg=cv::imread(filenames[i]);
        srcImg.copyTo(dstImg);
        if(i<10)
        {
        savefilename=dest+"000"+to_string(i)+".png";
        cv::imwrite(savefilename,dstImg);
        }
        if(i>=10&&i<99)
        {
        savefilename=dest+"00"+to_string(i)+".png";
        cv::imwrite(savefilename,dstImg);
        }
        if(i>=100&&i<999)
        {
        savefilename=dest+"0"+to_string(i)+".png";
        cv::imwrite(savefilename,dstImg);
        }
        if(i>=1000&&i<9999)
        {
        savefilename=dest+to_string(i)+".png";
        cv::imwrite(savefilename,dstImg);
        }
     
    }
  
return 0;
  }

