/****************************
 * 题目：四元数球面线性插值
 * 我们用智能手机采集了图像序列和IMU数据，由于IMU帧率远大于图像帧率，需要你用Slerp方法进行四元数插值，使得插值后的IMU和图像帧对齐
 * 已知某帧图像的时间戳为：t =700901880170406，离该图像帧最近的前后两个时刻IMU时间戳为：
 * t1 = 700901879318945，t2 = 700901884127851
 * IMU在t1, t2时刻测量得的旋转四元数为：
 * q1x=0.509339, q1y=0.019188, q1z=0.049596, q1w=0.858921；
 * q2x=0.509443, q2y=0.018806, q2z=0.048944,q2w=0.858905
 * 根据上述信息求IMU对齐到图像帧的插值后的四元数
* 本程序学习目标：
 * 学习四元数球面线性插值方法，注意实际使用中需要考虑的问题。
 *
 * 作者：公众号：计算机视觉life。发布于公众号旗下知识星球：从零开始学习SLAM
 * 时间：2019.08
****************************/
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>

#include <cmath>

using namespace std;
using namespace cv;
using namespace Eigen;

// 四元数球面线性插值简化方法：v'=v1*cosθ' + v⊥*sinθ'，原理见公众号推送文章
 Quaterniond slerp(double t, Quaterniond &q1, Quaterniond &q2)
 {
	 // ---- 开始你的代码 ----- //
	// 参考公众号推送文章
	Quaterniond q_slerp;
	float cos_q1q2=q1.w()*q2.w()+q1.x()*q2.x()+q1.y()*q2.y()+q1.z()*q2.z();
    

	//判断夹角
	if(cos_q1q2<0.0f)
	{
		//q1=-1*q1;//这样定义是错误的
		q1.w()=-1*q1.w();
		q1.x()=-1*q1.x();
		q1.y()=-1*q1.y();
		q1.z()=-1*q1.z();
		//cos_q1q2=-1*cos_q1q2;
		cos_q1q2=q1.w()*q2.w()+q1.x()*q2.x()+q1.y()*q2.y()+q1.z()*q2.z();
    
	}
	
	float k1,k2;

	if(cos_q1q2>0.9995f)
	{
		k1=1.0f-t;
		k2=t;
	}

	else
	{
		float sin_q1q2=sqrt(1.0f-cos_q1q2*cos_q1q2);
		float q1q2=atan2(sin_q1q2,cos_q1q2);
		k1=sin((1.0f-t)*q1q2)/sin_q1q2;
		k2=sin(t*q1q2)/sin_q1q2;
	}
	
	//q_slerp=k1*q1+k2*q2;//这样定义是错误的
	q_slerp.w()=k1*q1.w()+k2*q2.w();
	q_slerp.x()=k1*q1.x()+k2*q2.x();
	q_slerp.y()=k1*q1.y()+k2*q2.y();
	q_slerp.z()=k1*q1.z()+k2*q2.z();


	return q_slerp;

	
	// ---- 结束你的代码 ----- //
 }
int main ( int argc, char** argv )
{
	double t_img(700901880170406), t1_imu(700901879318945), t2_imu(700901884127851);
	Quaterniond q1 = Quaterniond(0.858921,0.509339, 0.019188, 0.049596);
	Quaterniond q2 = Quaterniond(0.858905,0.509443, 0.018806, 0.048944);
	double t = (t_img-t1_imu)*1e-9;
	cout<<t<<endl;
    Quaterniond q_slerp = slerp(t, q1, q2);
	cout<<"插值后的四元数：q_slerp =\n"<< q_slerp.coeffs() <<endl;  //coeffs的顺序是(x,y,z,w)
	
    return 0;
}
