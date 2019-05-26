/****************************
 * 题目：SLAM问题的目标之一就是精确的估计相机运动的轨迹（姿态），如果我们将相机运动的轨迹绘制出来，就可以直观的观察它的运动是否符合预期。
 * 给定一个轨迹文件trajectory.txt，该文件的每一行由若干个数据组成，格式为 [time, tx, ty, tz, qx, qy, qz, qw],
 * 其中 time 为时间，tx,ty,tz 为平移部分，qx,qy,qz,qw 是四元数表示的旋转部分，请完成数据读取部分的代码，绘制部分代码已经给出。
 *
* 本程序学习目标：
 * 熟悉李代数库Sophus安装及基本操作
 * 熟悉数据流读取基本操作
 * 需要安装pangolin，注意异常数据处理
 *
 * 公众号：计算机视觉life。发布于公众号旗下知识星球：从零开始学习SLAM
 * 时间：2018.10
****************************/

#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream>
#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

// path to trajectory file
string trajectory_file = "./trajectory.txt";

 
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);

int main(int argc, char **argv) {

    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses;

    /// implement pose reading code
    // 开始你的代码
  /*  //自己写的
    ifstream infile("./trajectory.txt",ios::in);;
	std::string feature; //存储读取的每行数据
	float feat_onePoint;  //存储每行按空格分开的每一个float数据
	std::vector<float> lines; //存储每行数据
	//std::vector<vector<float>> lines_feat; //存储所有数据
	//lines_feat.clear();
    poses.clear();
    Eigen::Vector3d t;
    Eigen::Quaterniond q;
	while(!infile.eof()) 
	{	
		getline(infile, feature); //一次读取一行数据
		stringstream stringin(feature); //使用串流实现对string的输入输出操作
		lines.clear();
		while (stringin >> feat_onePoint) {      //按空格一次读取一个数据存入feat_onePoint 
			lines.push_back(feat_onePoint); //存储每行按空格分开的数据 
		}
        for(auto i=0;i<lines.size();++i)
        {
            t<<lines[1],lines[2],lines[3];
            q=Eigen::Quaterniond(lines[7],lines[4],lines[5],lines[6]);
            
        }
		//lines_feat.push_back(lines); //存储所有数据
        Sophus::SE3 SE3_qt(q,t);
        poses.push_back(SE3_qt);
	}
	infile.close();*/


///网上的答案
    fstream finfile;
    finfile.open(trajectory_file.c_str());
     if (!finfile.is_open()){
        cout << "file is empty!" <<endl;
        return -1;
     }
    string line;
    double timestamp, tx, ty, tz, qx, qy, qz, qw;
    while( getline(finfile,line) ){
        stringstream lineStream(line);
        lineStream>>timestamp>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
          //if (timestamp == "#"){
          //  cout << "WARN: INF ERROR" << endl;
         //   continue;
        //}
        Eigen::Vector3d t(tx,ty,tz);
        Eigen::Quaterniond q = Eigen::Quaterniond(qw,qx,qy,qz).normalized();
        Sophus::SE3 SE3_qt(q,t);
        poses.push_back(SE3_qt);
    }


    // 结束你的代码

    // draw trajectory in pangolin
    DrawTrajectory(poses);
    return 0;
}

// 无需改动以下绘图程序
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses) {
    if (poses.empty()) {
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
        for (size_t i = 0; i < poses.size() - 1; i++) {
            glColor3f(1 - (float) i / poses.size(), 0.0f, (float) i / poses.size());
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   
    }

}
