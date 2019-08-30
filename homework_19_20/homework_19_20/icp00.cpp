/****************************
 * 题目：给定一个轨迹１，数据格式：timestamp tx ty tz qx qy qz qw
 * 自定义一个任意的旋转矩阵和平移向量（可以尝试不同的值看看结果有什么变化），对轨迹１进行变换，得到一个新的轨迹２
 * 使用ICP算法（提示：取平移作为三维空间点）估计轨迹１，２之间的位姿，然后将该位姿作用在轨迹２
 * 验证：ICP算法估计的旋转矩阵和平移向量是否准确；轨迹１，２是否重合
 * 
* 本程序学习目标：
 * 熟悉ICP算法的原理及应用。
 *
 * 公众号：计算机视觉life。发布于公众号旗下知识星球：从零开始学习SLAM，参考答案请到星球内查看。
 * 时间：2019.05
 * 作者：小六
****************************/
#include <iostream>
#include "sophus/se3.h"
#include <fstream>
#include <pangolin/pangolin.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

using namespace std;
using namespace cv;

void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> pose1,
                    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> pose2);



int main()
{
    // path to trajectory file
      string trajectory_file_orb = "./viorb.txt";
      string trajectory_file_gt = "./new.txt";

    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> pose_orb;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> pose_groundtruth;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> pose_estimate;
    
    vector<Point3f>  pts_groundtruth,pts_orb;

    
    //　读取轨迹数据orb
    ifstream textFile_orb;
    textFile_orb.open(trajectory_file_orb.c_str());
    if (!textFile_orb.is_open()){
        cout << "file is empty!" <<endl;
        return -1;
     }
    string line;
    double  timestamp,tx, ty, tz, qx, qy, qz, qw;
    while( getline(textFile_orb,line) )
    {
        stringstream lineStream(line);
        lineStream>>timestamp>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
         Eigen::Vector3d t(tx,ty,tz);
         Eigen::Quaterniond q = Eigen::Quaterniond(qw,qx,qy,qz).normalized();//一定要归一化
         Sophus::SE3 T(q,t);
       
        pose_orb.push_back(T);
        pts_orb.push_back(cv::Point3f(tx,ty,tz));

    }
    textFile_orb.close();


    //　读取轨迹数据gt
     ifstream textFile_gt;
    textFile_gt.open(trajectory_file_gt.c_str());
    if (!textFile_gt.is_open()){
        cout << "file is empty!" <<endl;
        return -1;
     }
    string line0;
    double  timestamp0,tx0, ty0, tz0, qx0, qy0, qz0, qw0;
    int i=0;
    while( getline(textFile_gt,line0)&&i<36382 )
    {
        stringstream lineStream0(line0);
        lineStream0>>timestamp0>>tx0>>ty0>>tz0>>qx0>>qy0>>qz0>>qw0;
         Eigen::Vector3d t0(tx0,ty0,tz0);
         Eigen::Quaterniond q0 = Eigen::Quaterniond(qw0,qx0,qy0,qz0).normalized();//一定要归一化
         Sophus::SE3 T0(q0,t0);
       
       
        pose_groundtruth.push_back(T0);
        pts_groundtruth.push_back(cv::Point3f(tx0,ty0,tz0)); 
        i=i+12; 
        //int k=i/12;
    }
    textFile_gt.close();
	
    // 使用ICP算法估计轨迹１，２之间的位姿，然后将该位姿作用在轨迹２
    Point3f p1,p2;//质心
    int N=pts_orb.size();
    for(int i=0;i<N;i++)
    {
        p1+=pts_orb[i];
        p2+= pts_groundtruth[i];
    }
    p1/=N;
    p2/=N;

    vector<Point3f> q1(N),q2(N);//去质心
    for(int i=0;i<N;i++)
    {
        q1[i]=pts_orb[i]-p1;
        q2[i]=pts_groundtruth[i]-p2;
    }
   
    //计算 q1*q2^T
    Eigen::Matrix3d W=Eigen::Matrix3d::Zero();
    for(int i=0;i<N;i++)
    {
        W+=Eigen::Vector3d(q1[i].x,q1[i].y,q1[i].z)*Eigen::Vector3d(q2[i].x,q2[i].y,q2[i].z).transpose();

    }
    cout<<"W= "<<W<<endl;

    //SVD
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W,Eigen::ComputeFullU|Eigen::ComputeFullV);
    Eigen::Matrix3d U=svd.matrixU();
    Eigen::Matrix3d V=svd.matrixV();
    cout<<"U= "<<U<<endl;
    cout<<"V= "<<V<<endl;

    Eigen::Matrix3d R_=U*(V.transpose());
    Eigen::Vector3d t_=Eigen::Vector3d(p1.x,p1.y,p1.z)-R_*Eigen::Vector3d(p2.x,p2.y,p2.z);

    /////////////////
    //取逆
    Eigen::Matrix3d Rot = R_.inverse();
   Eigen::Vector3d tran = -Rot*t_;
    Sophus::SE3 T_estimate(Rot, tran);  // pose_groundtruth =  T_estimate* pose_new
    
    // Sophus::SE3 T_estimate(R_, t_);
    for(int i = 0; i< pose_orb.size();i++) {
        pose_estimate.push_back(T_estimate * pose_orb[i]);
    }


//    DrawTrajectory(pose_groundtruth, pose_new);  // 变换前的两个轨迹
    DrawTrajectory(pose_groundtruth, pose_estimate);  // 轨迹应该是重合的
    return 0;
}



void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> pose1,
                    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> pose2) {
    if (pose1.empty()||pose2.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1080, 720);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < pose1.size() - 1; i++) {
            //glColor3f(1 - (float) i / pose1.size(), 1.0f, (float) i / pose1.size());
            glColor3f(1.0, 0.0, 0.0);
            glBegin(GL_LINES);
            auto p1 = pose1[i], p2 = pose1[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        for (size_t i = 0; i < pose2.size() - 1; i++) {
            //glColor3f(1 - (float) i / pose2.size(), 1.0f, (float) i / pose2.size());
             glColor3f(0,0,1);
            glBegin(GL_LINES);
            auto p1 = pose2[i], p2 = pose2[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}
