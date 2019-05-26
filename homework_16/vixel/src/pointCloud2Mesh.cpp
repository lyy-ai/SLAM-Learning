/****************************
 * 题目：给定一个稠密的点云，结合前面的练习，对其进行如下操作：
 * 下采样和滤波、重采样平滑、法线计算，贪心投影网格化（请提供结果的截图）。
 *
* 本程序学习目标：
 * 熟悉PCL网格化流程。
 *
 * 公众号：计算机视觉life。发布于公众号旗下知识星球：从零开始学习SLAM
 * 时间：2019.01
****************************/
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
typedef pcl::PointXYZ PointT;

int main(int argc, char** argv)
{

	// Load input file
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_downSampled(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_smoothed(new pcl::PointCloud<PointT>);
	if (pcl::io::loadPCDFile("../../data/fusedCloud.pcd", *cloud) == -1)
    {
        cout << "点云数据读取失败！" << endl;
    }

    std::cout << "Orginal points number: " << cloud->points.size() << std::endl;

   	// ----------------------开始你的代码--------------------------//
	// 请参考之前文章中点云下采样，滤波、平滑等内容，以及PCL官网实现以下功能。代码不难。
	
	// 下采样
	pcl::VoxelGrid<PointT> downSampled;//滤波对象
	downSampled.setInputCloud(cloud);
	downSampled.setLeafSize(0.01f,0.01f,0.01f);//体素大小
	downSampled.filter(*cloud_downSampled);

	pcl::io::savePCDFile("./downsampledPC.pcd",*cloud_downSampled);
	// 统计滤波
	
	pcl::StatisticalOutlierRemoval<PointT> OutlierRemoval;
	OutlierRemoval.setInputCloud(cloud_downSampled);
	OutlierRemoval.setMeanK(50);
	OutlierRemoval.setStddevMulThresh(3.0);
	OutlierRemoval.filter(*cloud_filtered);

	pcl::io::savePCDFile("./filteredPC.pcd",*cloud_filtered);

	// 对点云重采样  

	pcl::search::KdTree<PointT>::Ptr treeSampling(new pcl::search::KdTree<PointT>);
	pcl::MovingLeastSquares<PointT,PointT> mls;
	mls.setComputeNormals(false);
	mls.setInputCloud(cloud_filtered);
	mls.setPolynomialOrder(2);
	mls.setPolynomialFit(false);
	mls.setSearchMethod(treeSampling);
	mls.setSearchRadius(0.05);
	mls.process(*cloud_smoothed);

	pcl::io::savePCDFile("./cloud_smooth.pcd",*cloud_smoothed);
 
	// 法线估计

	pcl::NormalEstimation<PointT,pcl::Normal> normalEstimation;//对象
	normalEstimation.setInputCloud(cloud_smoothed);
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);// 创建用于最近邻搜索的KD-Tree
	normalEstimation.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);// 定义输出的点云法线

	normalEstimation.setKSearch(10);// 使用当前点周围最近的10个点
	normalEstimation.compute(*normals);//计算法线
	

/*
	// 贪心投影三角化
	// 将点云位姿、颜色、法线信息连接到一起

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud_smoothed, *normals, *cloud_with_normals);

	//定义搜索树对象
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// 三角化
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;   // 定义三角化对象
	pcl::PolygonMesh triangles; //存储最终三角化的网络模型

	// 设置三角化参数
	gp3.setSearchRadius(0.1);  //设置搜索时的半径，也就是KNN的球半径
	gp3.setMu (2.5);  //设置样本点搜索其近邻点的最远距离为2.5倍（典型值2.5-3），这样使得算法自适应点云密度的变化
	gp3.setMaximumNearestNeighbors (100);    //设置样本点最多可搜索的邻域个数，典型值是50-100

	gp3.setMinimumAngle(M_PI/18); // 设置三角化后得到的三角形内角的最小的角度为10°
	gp3.setMaximumAngle(2*M_PI/3); // 设置三角化后得到的三角形内角的最大角度为120°

	gp3.setMaximumSurfaceAngle(M_PI/4); // 设置某点法线方向偏离样本点法线的最大角度45°，如果超过，连接时不考虑该点
	gp3.setNormalConsistency(false);  //设置该参数为true保证法线朝向一致，设置为false的话不会进行法线一致性检查

	gp3.setInputCloud (cloud_with_normals);     //设置输入点云为有向点云
	gp3.setSearchMethod (tree2);   //设置搜索方式
	gp3.reconstruct (triangles);  //重建提取三角化*/


	//伯松重建
	//将点云和法线放到一起
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::concatenateFields(*cloud_smoothed, *normals, *cloud_with_normals);
 
	//创建搜索树
	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
	tree2->setInputCloud(cloud_with_normals);
	//创建Poisson对象，并设置参数
	pcl::Poisson<pcl::PointXYZRGBNormal> pn;
		//创建多变形网格，用于存储结果
	pcl::PolygonMesh mesh;

	pn.setConfidence(true); //是否使用法向量的大小作为置信信息。如果false，所有法向量均归一化。
	pn.setDegree(2); //设置参数degree[1,5],值越大越精细，耗时越久。
	pn.setDepth(8); //树的最大深度，求解2^d x 2^d x 2^d立方体元。由于八叉树自适应采样密度，指定值仅为最大深度。
	pn.setIsoDivide(8); //用于提取ISO等值面的算法的深度
	pn.setManifold(true); //是否添加多边形的重心，当多边形三角化时。 设置流行标志，如果设置为true，则对多边形进行细分三角话时添加重心，设置false则不添加
	pn.setOutputPolygons(false); //是否输出多边形网格（而不是三角化移动立方体的结果）
	pn.setSamplesPerNode(9.0); //设置落入一个八叉树结点中的样本点的最小数量。无噪声，[1.0-5.0],有噪声[15.-20.]平滑
	pn.setScale(1.25); //设置用于重构的立方体直径和样本边界立方体直径的比率。
	pn.setSolverDivide(8); //设置求解线性方程组的Gauss-Seidel迭代方法的深度
	//pn.setIndices();
 
   	pn.setInputCloud(cloud_with_normals);
	//设置搜索方法和输入点云
	pn.setSearchMethod(tree2);


	//执行重构
	pn.performReconstruction(mesh);
	//pn.reconstruct (mesh);  
 
	//保存网格图
	//pcl::io::savePCDFile("mesh.pcd", mesh);//不能用
 

	// ----------------------结束你的代码--------------------------//

    // 显示网格化结果
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);  //
   // viewer->addPolygonMesh(triangles, "mesh");  //
	viewer->addPolygonMesh(mesh, "mesh");
	//设置网格模型显示模式
	//viewer->setRepresentationToSurfaceForAllActors();   //网格模型以面片形式显示
	//viewer->setRepresentationToPointsForAllActors();    //网格模型以点形式显示
	viewer->setRepresentationToWireframeForAllActors();   //网格模型以线框图模式显示
	viewer->addCoordinateSystem(1.0);                     //设置坐标系  
	viewer->initCameraParameters();

    while (!viewer->wasStopped())
    {
    	viewer->spinOnce(100);
    	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    return 1;
}

