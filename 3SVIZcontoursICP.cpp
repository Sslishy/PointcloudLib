#include <iostream>
#include <thread>
#include <pcl/features/boundary.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>       // ndt配准文件头
#include <pcl/filters/approximate_voxel_grid.h>// 滤波文件头
#include <pcl/visualization/pcl_visualizer.h>
#include<pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include"Planefitting.h"
#include"Computepointspose.h"
#include"PointProcess.h"
#include"computeangle.h"
#include"PointCloudAligment.h"
using namespace std::chrono_literals;
int
main(int argc, char **argv) {
	string f = "false";
	string path1= argv[1];
	string path = "/home/slishy/Code/PCD/hanjie/" + path1;
	string path2 = argv[2];
	string path3 = "/home/slishy/Code/PCD/hanjie/" + path2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path3, *target_cloud) == -1) {
        PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
        return (-1);
    }
    // Loading second scan of room from new perspective.
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	 pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *input_cloud) == -1) {
        PCL_ERROR ("Couldn't read file room_scan2.pcd \n");
        return (-1);
    }
	
	PointProcess pp;
     	// 点云过滤
	pp.SetK(80);
	pp.Removepoint(input_cloud);
    pp.Removepoint(target_cloud);
	//提取平面点云
	Planefitting plane;
	plane.SetDistanceThreshold(0.002);
	plane.SetMaxIterations(100);
	plane.extract(target_cloud,target_cloud_,f);
	plane.extract(input_cloud,input_cloud_,f);
    pcl::PointCloud<pcl::Boundary> boundaries; //保存边界估计结果
    pcl::PointCloud<pcl::Boundary> boundaries1; //保存边界估计结果
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst; //定义一个进行边界特征估计的对象
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst; //定义一个法线估计的对象
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); //保存法线估计的结果
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary (new pcl::PointCloud<pcl::PointXYZ>); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary1 (new pcl::PointCloud<pcl::PointXYZ>); 
	normEst.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(target_cloud_)); 
	normEst.setRadiusSearch(0.003); //设置法线估计的半径
	normEst.compute(*normals); //将法线估计结果保存至normals
	boundEst.setInputCloud(target_cloud_); //设置输入的点云
	boundEst.setInputNormals(normals); //设置边界估计的法线，因为边界估计依赖于法线
	boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>)); //设置搜索方式KdTree
	boundEst.setKSearch(1000);
	boundEst.compute(boundaries); //将边界估计结果保存在boundaries
	std::cerr << "boundaries: " <<boundaries.points.size() << std::endl;
	//存储估计为边界的点云数据，将边界结果保存为pcl::PointXYZ类型
	for(int i = 0; i < target_cloud_->points.size(); i++) 
	{ 
		if(boundaries[i].boundary_point > 0) 
		{ 
			cloud_boundary->push_back(target_cloud_->points[i]); 
		} 
	} 
    normEst.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(input_cloud)); 
	normEst.setRadiusSearch(0.003); //设置法线估计的半径
	normEst.compute(*normals); //将法线估计结果保存至normals
	boundEst.setInputCloud(input_cloud); //设置输入的点云
	boundEst.setInputNormals(normals); //设置边界估计的法线，因为边界估计依赖于法线
	boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>)); //设置搜索方式KdTree
	boundEst.setKSearch(1000);
	boundEst.compute(boundaries1); //将边界估计结果保存在boundaries
	//存储估计为边界的点云数据，将边界结果保存为pcl::PointXYZ类型
	for(int i = 0; i < input_cloud->points.size(); i++) 
	{ 
		if(boundaries1[i].boundary_point > 0) 
		{ 
			cloud_boundary1->push_back(input_cloud->points[i]); 
		} 
	} 
	pcl::io::savePCDFile("cloud1",*cloud_boundary);
	pcl::io::savePCDFile("cloud2",*cloud_boundary1);
	pcl::PointCloud<pcl::PointXYZ>::Ptr fina11(new pcl::PointCloud<pcl::PointXYZ>);
	PointCloudAligment pa;
	pa.Aligment(cloud_boundary,cloud_boundary1,fina11);
	 Eigen::Matrix4f transformation2;
	pa.Gettransformation(transformation2);
	pcl::transformPointCloud(*cloud_boundary,*cloud_boundary,transformation2);	
	pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(cloud_boundary);
	icp.setInputTarget(cloud_boundary1);
	icp.setMaximumIterations(50);
	/* icp.setMaxCorrespondenceDistance(0.001);
	icp.setRANSACOutlierRejectionThreshold(0.05);
	icp.setTransformationEpsilon(1e-10);*/
	icp.align(*final);
	Eigen::Matrix4f transformation1 = icp.getFinalTransformation();

	pcl::PointCloud<pcl::PointXYZ>::Ptr pickpoint(new pcl::PointCloud<pcl::PointXYZ>);
	pickpoint->points.resize(1);
	//** input XYZ**//
	pickpoint->points[0].x = 0;
	pickpoint->points[0].y = 0;
	pickpoint->points[0].z = 0;
	//** input RPY **//
	cv::Vec3f RPY(0,0,0);
	cv::Mat RPYmaritix;
		cv::Mat_<double> trans1 = (cv::Mat_<double>(3, 3) <<
	transformation1(0, 0), transformation1(0, 1), transformation1(0, 2), 
	transformation1(1, 0), transformation1(1, 1), transformation1(1, 2), 
	transformation1(2, 0), transformation1(2, 1), transformation1(2, 2)
	);;
	computeangle ca;
	RPYmaritix = ca.eulerAnglesToRotationMatrix(RPY);
	//** Transformation xyzrpy **//
	pcl::transformPointCloud(*pickpoint,*pickpoint,transformation1);
	pcl::transformPointCloud(*cloud_boundary,*cloud_boundary,transformation1);
	RPYmaritix = trans1 * RPYmaritix;
	RPY = ca.rotationMatrixToEulerAngles(RPYmaritix);
	cout << pickpoint->points[0].x << ";" << pickpoint->points[0].y <<";" <<pickpoint->points[0].z <<";" << RPY[0] <<"; "<< RPY[1] <<";" << RPY[2]<<endl;
	pcl::visualization::PCLVisualizer::Ptr
	viewer_final(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer_final->setBackgroundColor(0, 0, 0);
	viewer_final->addPointCloud(cloud_boundary,pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_boundary,0.0,0.0,255.0),"cloud");
	viewer_final->addPointCloud(cloud_boundary1,pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_boundary1,0.0,255.0,0.0),"cloud1");
	// Starting visualizer
	viewer_final->addCoordinateSystem(1.0, "global");
	viewer_final->initCameraParameters();
while (!viewer_final->wasStopped()) {
	viewer_final->spinOnce(100);
	std::this_thread::sleep_for(100ms);
}

    return (0);
}