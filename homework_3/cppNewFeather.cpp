
#include <iostream>
#include <vector>
#include <algorithm>//sort
#include "opencv2/opencv.hpp"
using namespace cv;
using namespace std;

bool cmp(Point2i pt1, Point2i pt2){

	// ---  ----//
   if(pt1.x<pt2.x)
   {
	   return true;
   }
   else if(pt1.x==pt2.x)
   {
	   return pt1.y<pt2.y;
   }
   return false;
	// ---  ----//
}
int main()
{
	vector<Point2i> vec;
	vec.push_back(Point2i(2, 1));
	vec.push_back(Point2i(3, 3));
	vec.push_back(Point2i(2, 3));
	vec.push_back(Point2i(3, 2));
	vec.push_back(Point2i(3, 1));
	vec.push_back(Point2i(1, 3));
	vec.push_back(Point2i(1, 1));
	vec.push_back(Point2i(2, 2));
	vec.push_back(Point2i(1, 2));
	
	cout << "Before sort: " << endl;
	for (int i = 0; i < vec.size(); i++){
		cout << vec[i] << endl;
	}

	sort(vec.begin(), vec.end(), cmp);

	cout << "After sort: " << endl;
	for (int i = 0; i < vec.size(); i++){
		cout << vec[i] << endl;
	}

	return 0;
}




