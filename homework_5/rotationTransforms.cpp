/****************************
 * 题目：已知旋转矩阵定义是沿着Z轴旋转45°。
 * 按照该定义初始化旋转向量、旋转矩阵、四元数、欧拉角。请编程实现：
 * 1、以上四种表达方式的相互转换关系并输出，看看是否正确
 * 2、假设平移向量为（1,2,3）,请输出旋转矩阵和该平移矩阵构成的欧式变换矩阵，并根据欧式变换矩阵提取旋转向量及平移向量
 *
* 本程序学习目标：
 * 学习eigen中刚体旋转的四种表达方式，熟悉他们之间的相互转换关系
 * 熟悉旋转平移和欧式变换矩阵的相互转换关系
 *
 * 作者：公众号：计算机视觉life。发布于公众号旗下知识星球：从零开始学习SLAM
 * 时间：2018.09
****************************/
#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
using namespace std;

int main ( int argc, char** argv )
{
    // ----------  初始化 -----------//

    // 旋转向量（轴角）：沿Z轴旋转45°
    Eigen::AngleAxisd rotation_vector ( M_PI/4, Eigen::Vector3d ( 0,0,1 ) );
    cout<<"rotation_vector axis = \n" << rotation_vector.axis() <<"\n rotation_vector angle = "<< rotation_vector.angle()<<endl;

    //旋转矩阵：沿Z轴旋转45°
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
    rotation_matrix <<  0.707, -0.707,      0,
            0.707,  0.707,      0,
            0,      0,          1;
    cout<<"rotation matrix =\n"<<rotation_matrix <<endl;

    // 四元数：沿Z轴旋转45°
    Eigen::Quaterniond quat = Eigen::Quaterniond(0, 0, 0.383, 0.924);
    cout<<"四元数输出方法1：quaternion = \n"<<quat.coeffs() <<endl;   // 请注意coeffs的顺序是(x,y,z,w),w为实部，前三者为虚部
    cout<<"四元数输出方法2：\n x = " << quat.x() << "\n y = " << quat.y() << "\n z = " << quat.z() << "\n w = " << quat.w() << endl;

    // 欧拉角: ：沿Z轴旋转45°
    Eigen::Vector3d euler_angles = Eigen::Vector3d(M_PI/4, 0, 0);// ZYX顺序，即roll pitch yaw顺序
    cout<<"Euler: yaw pitch roll = "<<euler_angles.transpose()<<endl;



    //  相互转化关系 
    // ----------  请在下面补充对应的代码 ----------
    rotation_matrix=rotation_vector.toRotationMatrix();//旋转向量转化为旋转矩阵
    rotation_matrix=rotation_vector.matrix();
    // 旋转向量转化为其他形式
    cout<<"旋转向量转化为旋转矩阵方法1：rotation matrix =\n"<< rotation_matrix<<endl;
    cout<<"旋转向量转化为旋转矩阵方法2：rotation matrix =\n"<<rotation_matrix <<endl;

    quat = Eigen::Quaterniond(rotation_vector);
    cout<<"旋转向量转化为四元数：quaternion =\n"<<quat.coeffs()<<endl;  //coeffs的顺序是(x,y,z,w),w为实部，前三者为虚部


    // 旋转矩阵转化为其他形式
     rotation_vector.fromRotationMatrix(rotation_matrix);//旋转矩阵转化为旋转向量
    cout<<"旋转矩阵转化为旋转向量：rotation_vector axis = \n" << rotation_vector.axis() <<"\n rotation_vector angle = "<<rotation_vector.angle() <<endl;
    //注意：fromRotationMatrix 参数只适用于旋转向量，不适用于四元数

    rotation_vector =rotation_matrix.matrix();//之前出错
    cout<<"旋转矩阵直接给旋转向量赋值初始化：rotation_vector axis = \n" <<rotation_vector.axis()  <<"\n rotation_vector angle = "<<rotation_vector.angle() <<endl;

    euler_angles = rotation_matrix.eulerAngles(2,1,0);
    cout<<"旋转矩阵转化为欧拉角：yaw pitch roll = "<< euler_angles.transpose()<<endl;

    quat = Eigen::Quaterniond (rotation_matrix);
    cout<<"旋转矩阵转化为四元数：quaternion =\n"<< quat.coeffs() <<endl;

    // 四元数转化为其他形式
    rotation_vector =quat.matrix();/////之前出错
    cout<<"四元数转化为旋转向量：rotation_vector axis = \n" <<rotation_vector.axis()  <<"\n rotation_vector angle = "<<rotation_vector.angle()<<endl;

    rotation_matrix = quat.matrix();
    cout<<"四元数转化为旋转矩阵方法1：rotation matrix =\n"<<rotation_matrix<<endl;

    rotation_matrix = quat.toRotationMatrix();
    cout<<"四元数转化为旋转矩阵方法2：rotation matrix =\n"<<rotation_matrix<<endl;


    //  欧氏变换矩阵

     // 欧氏变换矩阵使用 Eigen::Isometry，仿射变换用 Eigen::Affine3d，射影变换用 Eigen::Projective3d
     Eigen::Isometry3d T = Eigen::Isometry3d::Identity();              // 名称为3d，实质上是4 x 4的矩阵
     T.rotate(rotation_vector);
     T.pretranslate(Eigen::Vector3d(1,3,4));
     cout << "Transform matrix = \n" << T.matrix() <<endl;             // 变换矩阵需要用成员函数转一下输出
   
    Eigen::Quaterniond q=Eigen::Quaterniond(rotation_vector);
   //////////////////写的有问题
    //rotation_matrix=T.matrix();
   // Eigen::Vector3d translation=T.translation();
    //////////////////////////
  // rotation_matrix=T.block<3,3>(1,1);
  // Eigen::Vector3d translation=T.block<3,1>(1,1);
   //Eigen::Vector3d translation=(T*v2d1.transpose()).transpose()*v2d.transpose();
   ///////////////////////////
  
  /*
  //我他妈是智障吗。明明一个函数写出来了，写了一大堆
  
  Eigen::Matrix<double,4,4> matrix_44;//里面只能是double
  matrix_44<<1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0;
  Eigen::Matrix<double,4,3> matrix_43;
  matrix_43<<1,0,0,0,1,0,0,0,1,0,0,0;
  rotation_matrix=(T*matrix_44*matrix_43).transpose()*matrix_43;

   Eigen::Matrix<double,4,4> matrix_44_1;
  matrix_44_1<<0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0;
  Eigen::Matrix<double,4,3> matrix_43_1;
  matrix_43<<1,0,0,0,0,0,0,0,0,0,0,0;
  Eigen::Vector3d vector_31(1,0,0);
  Eigen::VectorXd translation=T*matrix_44_1*matrix_43*vector_31;//此处4×1 哆啦一个1

 
     cout << "欧氏变化矩阵提取旋转矩阵：rotation_matrix = \n" <<rotation_matrix  << endl;
    cout << "欧氏变化矩阵提取平移向量：translation = \n" << translation.topLeftCorner(3,1)<< endl;*/

    cout << "欧氏变化矩阵提取旋转矩阵：rotation_matrix = \n" <<T.rotation()<< endl;
    cout << "欧氏变化矩阵提取平移向量：translation = \n" <<T.translation()<< endl;

    Eigen::Vector3d P(1,1,1);
    Eigen::Vector3d p=T*P;
    Eigen::Vector3d p1=q*P;
    cout<<"旋转矩阵变换后的P"<<p.transpose()<<endl;
    cout<<"四元数变换后的P"<<p1.transpose()<<endl;


    // ----------  结束 ----------*/
    return 0;
}
