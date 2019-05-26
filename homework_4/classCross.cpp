#include <iostream>
#include <stdio.h>
#include <string>
#include <iostream>
#include <vector>
#include <algorithm>//sort
#include <cassert>
#include <cmath>
#include <iterator>

using namespace std;
/*
typedef int P(); // 简单的
typedef void Q(int *p, const std::string& s1, const std::string& s2, size_t size, bool is_true); // 复杂的
class X {
public:
    P(eat_shit); // 等价于声明`int eat_shit();`
    Q(bullshit); // 等价于声明`void bullshit(int *p, const string& s1, const string& s2, size_t size, bool is_true);`
};
 


int main() {
    X *xx;
    printf("shit ret: %d\n", xx->eat_shit());
    int a[] = {1, 3, 4, 5, 7};
    xx->bullshit(a, "foo", "bar", sizeof(a)/sizeof(int), true);
}
 
int X::eat_shit() {
    return 888;
}
 
void X::bullshit(int *p, const std::string& s1, const std::string& s2, size_t size, bool is_true) {
    std::cout << "s1: " << s1 << ", s2: " << s2 << ", size: " << size << std::endl;
    printf("elems:\n");
    for(int i = 0; i < size; i++) {
        printf("%d %s",  *p++, (i == size-1) ? "" : ",");
    }
    printf("\n");
}*/


class Vector
{

 public:
  Vector(double cx, double cy, double cz) : x(cz), y(cy), z(cz){}
  Vector OuterProduct(const Vector & v)
  {
    double nx = y * v.z - z * v.y;
    double ny = z * v.x - x * v.z;
    double nz = x * v.y - y * v.x;
    return Vector(nx, ny, nz); 
   
   }
  double InnerProduct(const Vector & v)
  {
     return x * v.x + y * v.y + z * v.z;
    
   }
  void print()
  {
     cout<<x<<" "<<y<<" "<<z<<endl;
  }
  
  

 private:
  double x;
  double y;
  double z;

};


int main()
{
  Vector vector{1,2,3};
  vector.print();
  double a=vector.InnerProduct(vector);
  Vector b=vector.OuterProduct(vector);
  cout<<a<<endl;
 // cout<<b<<endl;
            
  return 0;
}
/*
  Vector Vector::OuterProduct(const Vector & v)
  {
    double nx = y * v.z - z * v.y;
    double ny = z * v.x - x * v.z;
    double nz = x * v.y - y * v.x;
    //return Vector(nx, ny, nz); 
    void print();
   }
  double Vector::InnerProduct(const Vector & v)
  {
     //return x * v.x + y * v.y + z * v.z;
     double sum=x * v.x + y * v.y + z * v.z;
     void print();
   }
  
  void Vector::print(double x,double y,double z)
  {
         cout<<x<<" "<<y<<" "<<z<<endl;
  }
  
*/
