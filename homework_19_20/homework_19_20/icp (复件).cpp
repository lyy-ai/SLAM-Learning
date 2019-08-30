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
    string trajectory_file = "../trajectory.txt";


    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> pose_groundtruth;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> pose_new;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> pose_estimate;
    vector<Point3f> pts_new, pts_groundtruth;

    Eigen::Quaterniond q;
    Eigen::Vector3d t;
    Sophus::SE3 T;
    string timestamp;
    ifstream textFile;

    // 自定义一个变换矩阵
    /**********************　开始你的代码，参考星球里作业５代码　****************************/
    // 旋转向量（轴角）：沿Z轴旋转45°
 
    // 平移向量，可以自己自定义，我这里是 x=3, y=-1, z=0.8，可以多尝试其他值
  
    /**********************　结束你的代码　****************************/
	
    Sophus::SE3 myTransform(rotate,tranlation);
    cout<<"rotation matrix =\n"<<rotation_vector.matrix() <<endl;
    cout<<"translation vector =\n"<<tranlation <<endl;
    cout<<"myTransform =\n"<<myTransform.matrix() <<endl;

    textFile.open(trajectory_file.c_str());

    //　读取轨迹数据
    /**********************　开始你的代码，参考星球里作业８代码　****************************/
	// 提示：取平移作为三维空间点
 
    /**********************　结束你的代码　****************************/
    textFile.close();

	
    // 使用ICP算法估计轨迹１，２之间的位姿，然后将该位姿作用在轨迹２
    /**********************　开始你的代码，参考十四讲中第７章ICP代码　****************************/

    /**********************　结束你的代码　****************************/

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
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
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
            glColor3f(1 - (float) i / pose1.size(), 0.0f, (float) i / pose1.size());
            glBegin(GL_LINES);
            auto p1 = pose1[i], p2 = pose1[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        for (size_t i = 0; i < pose2.size() - 1; i++) {
            glColor3f(1 - (float) i / pose2.size(), 0.0f, (float) i / pose2.size());
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
