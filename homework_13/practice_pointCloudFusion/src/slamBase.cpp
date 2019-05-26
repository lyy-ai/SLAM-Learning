#include "slamBase.hpp"
#include <string>


PointCloud::Ptr image2PointCloud(Mat rgb, Mat depth, CAMERA_INTRINSIC_PARAMETERS camera)
{
    PointCloud::Ptr cloud ( new PointCloud );
    for (int m = 0; m < depth.rows; m++)
        for (int n = 0; n < depth.cols; n++)
        {
            // 获取深度图中(m,n)处的值
            ushort d = depth.ptr<ushort>(m)[n];
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            PointT p;
            // 计算这个点的空间坐标
            p.z = double(d) / camera.scale;
            p.x = (n - camera.cx) * p.z / camera.fx;
            p.y = (m - camera.cy) * p.z / camera.fy;

            // 从rgb图像中获取它的颜色
            p.b = rgb.ptr<uchar>(m)[n * 3];
            p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
            p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

            // 把p加入到点云中
            cloud->points.push_back(p);
        }
    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;
    return cloud;
}


PointCloud::Ptr pointCloudFusion( PointCloud::Ptr &original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS camera )
//PointCloud::Ptr pointCloudFusion( PointCloud::Ptr &original, FRAME& newFrame, vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> T, CAMERA_INTRINSIC_PARAMETERS camera )
{
	// ---------- 开始你的代码  ------------- -//
	// 简单的点云叠加融合
    /*
    //自己写的可以运行但是结果不对，缺少
    original->is_dense=false;

     PointCloud::Ptr output (new PointCloud());
    pcl::transformPointCloud( *original, *output, T.matrix() );
   *output += *original;
   return output;*/

     //答案
    PointCloud::Ptr newCloud(new PointCloud()), transCloud(new PointCloud());
    newCloud = image2PointCloud(newFrame.rgb, newFrame.depth, camera);

    pcl::transformPointCloud(*newCloud, *transCloud, T.matrix());
    *original += *transCloud;

    return original;
  
	// ---------- 结束你的代码  ------------- -//
}


void readCameraTrajectory(string camTransFile, vector<Eigen::Isometry3d> &poses)
//void readCameraTrajectory(string camTransFile,vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>  poses)
{
    ifstream fcamTrans(camTransFile);
    if(!fcamTrans.is_open())
    {
        cerr << "trajectory is empty!" << endl;
        return;
    }

   	// ---------- 开始你的代码  ------------- -//
	// 参考作业8 绘制轨迹

    string line;
    double t, tx, ty, tz, qx, qy, qz, qw;
    while( getline(fcamTrans,line) )
    {
        stringstream lineStream(line);
        lineStream>>t>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
        string  a ="-1.#INF";
         if (t == atof(a.c_str()))
         {
           cout << "WARN: INF ERROR" << endl;
           continue;
         }
        //Eigen::Vector3d t(tx,ty,tz);
        Eigen::Quaterniond q = Eigen::Quaterniond(qw,qx,qy,qz).normalized();
       // Sophus::SE3 SE3_qt(q,t);//自己写的编译不过
       // poses.push_back(SE3_qt);

        //Eigen::Isometry3d t(q);//不容易理解
       // t(0,3)=tx;t(1,3)=ty;t(2,3)=tz;
        //poses.push_back(t);

        Eigen::Isometry3d T(q); //变换矩阵初始化旋转部分，
        T.pretranslate( Eigen::Vector3d(tx,ty,tz));//变换矩阵初始化平移向量部分
        poses.push_back( T );   //存储变换矩阵到位姿数组

       
    }
	// ---------- 结束你的代码  ------------- -//
}