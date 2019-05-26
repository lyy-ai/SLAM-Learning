#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace std;

int main()
{
//要注意Eigen中四元数赋值的顺序，实数w在首；
//但是实际上它的内部存储顺序是[x y z w]。实际上后面输出系数的时候也是按内部存储顺序输出
  Eigen::Quaterniond q(0.1,0.35,0.2,0.3);//初始化把w放前面
  cout<<q.coeffs()<<endl;
  cout<<endl;
  q=q.normalized();//只有单位四元数才能表示旋转矩阵
  cout<<q.coeffs()<<endl;//打印四元数
  Eigen::Matrix3d R=q.toRotationMatrix();
  Eigen::Vector3d t(1,1,1);
  //欧式变换矩阵为T
  Eigen::Isometry3d T=Eigen::Isometry3d::Identity();
  T.rotate(R);
  T.pretranslate(t);
  cout<<T.matrix()<<endl;
  //旋转矩阵变为四元数
  Eigen::Quaterniond q0=Eigen::Quaterniond(R);

  cout<<endl;
  cout<<R<<endl;
  cout<<endl;
  cout<<R.transpose()<<endl;
    cout<<endl;
  cout<<R.inverse()<<endl;
    cout<<endl;
  cout<<R*R.transpose()<<endl;
    cout<<endl;
  cout<<R.determinant()<<endl;//旋转矩阵的行列式
 
   
  return 0;
}
