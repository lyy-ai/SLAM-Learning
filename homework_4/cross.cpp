
#include <iostream>
#include <vector>
#include <algorithm>//sort
#include <cassert>
#include <cmath>
#include <iterator>

using namespace std;

  typedef vector<double> Vec0;//向量的定义重命名不能与库函数重名，否则报错模糊

   Vec0 operator-(const Vec0& x, const Vec0& y)
    {
       assert(x.size() == y.size());
                        
       Vec0 tmp;
      for(size_t i = 0; i < x.size(); ++i)
	  {
           tmp.push_back(x[i] - y[i]);
	  }
        
      return tmp;         // 返回局部变量的拷贝
    }

    double operator*(const Vec0& x, const Vec0& y)
   {
    assert(x.size() == y.size());       
    double sum = 0.;
    for (size_t i = 0; i < x.size(); ++i)
	{
      sum += x[i]*y[i];
	}
        
    return sum;
   }

   // 三维的情况
    Vec0 operator^(const Vec0& x, const Vec0& y)
    {
    assert(x.size() == y.size() && x.size() == 3);
    return Vec0{x[1]*y[2]-x[2]*y[1], 
                x[2]*y[0]-x[0]*y[2],
                x[0]*y[1]-x[1]*y[0]};
           
    }

	//模长或范数计算
	double norm(const Vec0& x)
   {
    double val = 0.;
    for(auto elem: x)
	{
       val += elem*elem;
	}
        
    return sqrt(val);   
                   
    }

  // 二维就姑且返回其模长吧
    double twoDCrossProd(const Vec0& x, const Vec0& y)
   {
    return x[0]*y[1]-x[1]*y[0];
    }

//向量夹角计算
 #define PI 3.14159265358979323846
// 弧长向弧度的转换
 double toDegree(double val)
 {
    return val*180/PI;
 }

 double angle(const Vec0& x, const Vec0& y)
 {
    return toDegree(acos(x*y/norm(x)/norm(y)));
            // x*y, 计算二者的内积
 }

//点到直线距离
// x0, x1, x2 分别表示三角形的三个顶点的坐标
// 这里计算的是点x0到x1和x2构成的直线的距离
double distance(const Vec0& x0, const Vec0& x1, const Vec0& x2)
{
    return twoDCrossProd(x1-x0, x2-x0)/norm(x1-x2);
}

int main(int,char**)
{  
   
   Vec0 x{1, 0, 0}, y{0, 1, 0};
   Vec0 z = x^y; // 计算叉乘
   copy(z.begin(), z.end(), ostream_iterator<double>(cout, " "));
    cout << endl;// 0 0 1
	Vec0 w=x-y;//相减法
	copy(w.begin(), w.end(), ostream_iterator<double>(cout, " "));
    cout << endl;
    double k=x*y;//点乘法
	cout<<k<<endl;
	double a=norm(x);//范数
	cout<<a<<endl;
	
        
	Vec0 alpha{1, 0}, beta{1, 1};
	double b= twoDCrossProd(alpha,beta);//模长计算
	cout<<b<<endl;
    cout << angle(alpha,beta) << endl;
            // 45

    Vec0 x0{0, 0}, x1{1, 0}, x2{0, 1};   
    cout << distance(x0, x1, x2) << endl;
            // 1/sqrt(2)
    return 0;
}


