/*
 * @Author: Slishy 
 * @Date: 2021-07-19 13:37:34 
 * @Last Modified by: slishy
 * @Last Modified time: 2021-07-19 17:01:09
 */
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <ctime>
#include <time.h>
#include"Planefitting.h"
#include"Computepointspose.h"
#include"computeangle.h" 
#include"CollisionDetection.h"
#include"Pointviewer.h"
#include"cylinderfitting.h"
#include"PointProcess.h"
using namespace std;
using namespace cv;
void Getcentertoallpointdistance(vector<pcl::PointXYZ> center,vector<pcl::PointCloud<pcl::PointXYZ>> &cloudlist)
{
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloudlist[0].makeShared());
	int k = cloudlist[0].points.size();
	std::vector<int> pointIdxNKNSearch(k);
    std::vector<float> pointNKNSquaredDistance(k);
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	inliers->indices.resize(k);
	kdtree.nearestKSearch(center[0],k,pointIdxNKNSearch,pointNKNSquaredDistance);
	for (size_t i = 0; i < pointIdxNKNSearch.size(); i++)
	{
		if (sqrt(pointNKNSquaredDistance[i]) > 10 * 0.001)
		{
			
     		inliers->indices[i] = pointIdxNKNSearch[i];
		}
	}
		pcl::copyPointCloud(*cloudlist[0].makeShared(),*cloud);
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*cloud);		
		pcl::io::savePCDFileASCII("cloud1",*cloud);
		pcl::copyPointCloud(*cloud,*cloudlist[0].makeShared());
		pcl::io::savePCDFileASCII("cloud2",*cloudlist[0].makeShared());
}
void consleXYZRPY(vector<pcl::PointXYZ> center, vector<cv::Vec3f> RPYList, vector<pcl::PointCloud<pcl::PointXYZ>> cloudlist, vector<bool> Collision)
{
	for (size_t i = 0; i < center.size(); i++)
	{
		cout << center[i].x << ";" << center[i].y << ";" << center[i].z << ";" << RPYList[i][0] << ";" << RPYList[i][1] << ";" << RPYList[i][2] << ";" << cloudlist[i].size() << ";" << boolalpha << Collision[i] << ";" << endl;
	}

}
void consleXYZRPY(vector<pcl::PointXYZ> center, vector<cv::Vec3f> RPYList, vector<pcl::PointCloud<pcl::PointXYZ>> cloudlist, vector<bool> Collision,vector<float> W,vector<float> H)
{
	for (size_t i = 0; i < center.size(); i++)
	{
		cout << center[i].x << ";" << center[i].y << ";" << center[i].z << ";" << RPYList[i][0] << ";" << RPYList[i][1] << ";" << RPYList[i][2] << ";" << cloudlist[i].size() << ";" << boolalpha << Collision[i] << ";"<< W[0] <<";"<<H[0]<<";" << endl;
	}

}
int main(int argc, char** argv) {
	const clock_t begin_time = clock( );
	vector<pcl::PointCloud<pcl::PointXYZ>> cloudlist;
	vector<pcl::Normal> N;
	vector<pcl::Normal> Normal_X;
	vector<pcl::Normal> Normal_Y;
	vector<cv::Vec3f> RPYList;
	vector<float> Width;
	vector<float> Height;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	vector<pcl::PointXYZ> center;
	vector<pcl::PointXYZ> center1;
	pcl::PointXYZ center_mean;
	pcl::PointXYZ maxpoint;
	pcl::PointXYZ minpoint;
	vector<bool> Collision;
	string path1 = argv[1];
	//string path = "/home/td/Code/PCD/" + path1;
	//string pathwhite = "/home/td/Code/PCD/"+path1;
	string path = "/home/slishy/Code/PCD/box/" + path1;
	string pathwhite = "/home/slishy/Code/PCD/white/"+path1;
	string pathbluebox = "/home/slishy/Code/PCD/bluebox/" + path1;
	string pathwithout_blue = "/home/slishy/Code/PCD/without_blue/" + path1;
	string pathbag = "/home/slishy/Code/PCD/bag/" + path1;
	string box = "0";
	string whitebox = "1";
	string bluebox = "2";
	string without_blue = "3";
	string bag = "4";
	string angle = argv[3];
	float angle_ = atof(angle.c_str());
    string offx_ = argv[4];
	string offy_ = argv[5];
	float offx = atof(offx_.c_str());
	float offy = atof(offy_.c_str());
if (argv[2] == box)
{	
	if (pcl::io::loadPCDFile(path, *cloud) < 0)
		{

			
			cout << "load fail . . . ." << endl;
			return 0;
		}
	    *cloud1 = *cloud;
		PointProcess pp;
		// 点云过滤
		pp.Removepoint(cloud);
		//提取平面点云
		Planefitting plane;
		plane.SetDistanceThreshold(0.01);
		plane.SetMaxIterations(100);
		
		plane.extractbynormal(cloud,cloudlist);
		Computepointspose Cp;
		//设置通过多少个点算法线
		Cp.SetFindNum(999);
		//计算点云质心
		Cp.computePickPoints(cloudlist,center);
		Cp.computePointNormal(cloudlist,center,N);
		//法线超过40°被滤掉
		Cp.Setfilternormalangle(cloudlist,40.0f);
		computeangle Ca;
		//设置旋转角度
		Ca.SetRotate(angle_);
		Ca.ComputeDirection(N,Normal_X,Normal_Y);
		//计算RX,RY,RZ
		Ca.computeRPY(N,Normal_X,Normal_Y,RPYList);
		Collision.resize(2);
		Cp.computePickpointsbyWH(cloudlist,Width,Height,center1);
		Ca.Setoffset_x_value(offx);
		Ca.Setoffset_y_value(offy);
		Ca.offsetX(center,Normal_X);
		Ca.offsetY(center,Normal_Y);
		consleXYZRPY(center,RPYList,cloudlist,Collision,Width,Height);
		Pointviewer Pv;
		Pv.simpleVisN(cloud1,center,N,cloudlist,Normal_X,Normal_Y,RPYList);  
}
	if (argv[2] == whitebox)
	{
		if (pcl::io::loadPCDFile(pathwhite, *cloud) < 0)
		{
			cout << "load fail . . . ." << endl;
			return 0;
		}
		
		PointProcess pp;
		// 点云过滤
		pp.Removepoint(cloud);
		*cloud1 = *cloud;
		pcl::io::savePCDFile("cloud.pcd",*cloud);
		Eigen::Vector4f center_;
		pcl::compute3DCentroid(*cloud, center_);
		center_mean = pcl::PointXYZ(center_[0], center_[1], center_[2]);
		pcl::getMinMax3D(*cloud,minpoint,maxpoint);
		center_mean.z = maxpoint.z;
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		int k = cloud->points.size();
		kdtree.setInputCloud(cloud);
		std::vector<int> pointIdxNKNSearch(k);
        std::vector<float> pointNKNSquaredDistance(k);
		kdtree.nearestKSearch(center_mean,k,pointIdxNKNSearch,pointNKNSquaredDistance);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
		inliers->indices.resize(k);
		for (size_t i = 0; i < pointIdxNKNSearch.size(); i++)
		{
			
			if ((center_mean.z - cloud->points[pointIdxNKNSearch[i]].z) > 0.05 )
			{
				inliers->indices[i] = pointIdxNKNSearch[i]; 
			}
		}
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*cloud);
		pcl::io::savePCDFile("cloud1.pcd",*cloud);
		//提取平面点云
		Planefitting plane;
		plane.SetRatio(0.8);
		plane.SetDistanceThreshold(0.003);
		plane.SetMaxIterations(50);
		plane.extractbynormal(cloud,cloudlist);
		Computepointspose Cp;
		//设置通过多少个点算法线
		Cp.SetFindNum(999);
		//计算点云质心
		Cp.computePickPoints(cloudlist,center);
		Cp.computePointNormal(cloudlist,center,N);
		
		//法线超过20°被滤掉
		Cp.Setfilternormalangle(cloudlist,20.0f);
		
		computeangle Ca;
		//设置旋转角度
		Ca.SetRotate(angle_);
		Ca.ComputeDirection(N,Normal_X,Normal_Y);
		//计算RX,RY,RZ
		Ca.computeRPY(N,Normal_X,Normal_Y,RPYList);
		vector<float> z(center.size());
		for (size_t i = 0; i < center.size(); i++)
		{
			z[i] = center[i].z;
		}
		
		Collision.resize(2);
		Cp.computePickpointsbyWH(cloudlist,Width,Height,center);
		for (size_t i = 0; i < z.size(); i++)
		{
			center[i].z = z[i];
		}
		Ca.Setoffset_x_value(offx);
		Ca.Setoffset_y_value(offy);
		Ca.offsetX(center,Normal_X);
		Ca.offsetY(center,Normal_Y);
		//cout << Width[0] * 1000 <<endl;
		//cout << Height[0] * 1000 <<endl;
		consleXYZRPY(center,RPYList,cloudlist,Collision,Width,Height);
		Pointviewer Pv;
		Pv.simpleVisN(cloud1,center,N,cloudlist,Normal_X,Normal_Y,RPYList);  
	}
	if (argv[2] == bluebox)
	{
		if (pcl::io::loadPCDFile(pathbluebox, *cloud) < 0)
		{
			cout << "load fail . . . ." << endl;
			return 0;
		}
		PointProcess pp;
		// 点云过滤
		pp.Removepoint(cloud);
		//提取平面点云
		Planefitting plane;
		plane.SetDistanceThreshold(0.01);
		plane.SetMaxIterations(100);
		plane.extractbynormal(cloud,cloudlist);
		Computepointspose Cp;
		//设置通过多少个点算法线
		Cp.SetFindNum(100);
		//计算点云质心
		Cp.computePickPoints(cloudlist,center);
		Cp.computePointNormal(cloudlist,center,N);
		//法线超过40°被滤掉
		//Cp.Setfilternormalangle(cloudlist,40.0f);
		computeangle Ca;
		//设置旋转角度
		Ca.SetRotate(angle_);
		Ca.ComputeDirection(N,Normal_X,Normal_Y);
		//计算RX,RY,RZ
		Ca.computeRPY(N,Normal_X,Normal_Y,RPYList);
		Collision.resize(2);
		Cp.computePickpointsbyWH(cloudlist,Width,Height,center1);
		Ca.Setoffset_x_value(offx);
		Ca.Setoffset_y_value(offy);
		Ca.offsetX(center,Normal_X);
		Ca.offsetY(center,Normal_Y);
		consleXYZRPY(center,RPYList,cloudlist,Collision,Width,Height);
		Pointviewer Pv;
		Pv.simpleVisN(cloud1,center,N,cloudlist,Normal_X,Normal_Y,RPYList); 
		 
	}
	if (argv[2] == without_blue)
	{
		if (pcl::io::loadPCDFile(pathwithout_blue, *cloud) < 0)
		{
			cout << "load fail . . . ." << endl;
			return 0;
		}
		PointProcess pp;
		// 点云过滤
		cout <<cloud->points.size()<<endl;
		pp.Removepoint(cloud);
		cout <<cloud->points.size()<<endl;
		PointProcess pp1;
		pp1.Removepoint(cloud);
		cout <<cloud->points.size()<<endl;
		pcl::io::savePCDFile("1.pcd",*cloud);
		pcl::getMinMax3D(*cloud,minpoint,maxpoint);
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(cloud); 
		pass.setFilterFieldName("z");     
		pass.setFilterLimits(maxpoint.z- 0.12, maxpoint.z);
		pass.filter(*cloud); 
		//提取平面点云
		Planefitting plane;
		plane.SetRatio(0.8);
		plane.SetDistanceThreshold(0.003);
		plane.SetMaxIterations(50);
		plane.extractbynormal(cloud,cloudlist);
		Computepointspose Cp;
		//设置通过多少个点算法线
		Cp.SetFindNum(999);
		//计算点云质心
		Cp.computePickPoints(cloudlist,center);
		Cp.computePointNormal(cloudlist,center,N);
		//法线超过20°被滤掉
		Cp.Setfilternormalangle(cloudlist,20.0f);
		computeangle Ca;
		//设置旋转角度
		Ca.SetRotate(angle_);
		Ca.ComputeDirection(N,Normal_X,Normal_Y);
		
		//计算RX,RY,RZ
		Ca.computeRPY(N,Normal_X,Normal_Y,RPYList);
		vector<float> z(center.size());
		for (size_t i = 0; i < center.size(); i++)
		{
			z[i] = center[i].z;
		}
		Collision.resize(2);
		Cp.computePickpointsbyWH(cloudlist,Width,Height,center);
		for (size_t i = 0; i < z.size(); i++)
		{
			center[i].z = z[i];
		}
		Ca.Setoffset_x_value(offx);
		Ca.Setoffset_y_value(offy);
		Ca.offsetX(center,Normal_X);
		Ca.offsetY(center,Normal_Y);
		//cout << Width[0] * 1000 <<endl;
		//cout << Height[0] * 1000 <<endl;
		consleXYZRPY(center,RPYList,cloudlist,Collision,Width,Height);
		Pointviewer Pv;
		Pv.simpleVisN(cloud1,center,N,cloudlist,Normal_X,Normal_Y,RPYList);  

	}
	if (argv[2] == bag)
	{
		if (pcl::io::loadPCDFile(pathbag, *cloud) < 0)
		{
			cout << "load fail . . . ." << endl;
			return 0;
		}
		PointProcess pp;
		// 点云过滤
		pp.Removepoint(cloud);
		pp.DownSimple(cloud);
		//提取平面点云
		Planefitting plane;
		plane.SetDistanceThreshold(0.01);
		plane.SetMaxIterations(100);
		plane.extractbynormal(cloud,cloudlist);
		Computepointspose Cp;
		//设置通过多少个点算法线
		Cp.SetFindNum(100);
		//计算点云质心
		Cp.computePickPoints(cloudlist,center);
		Cp.computePointNormal(cloudlist,center,N);
		//法线超过40°被滤掉
		//Cp.Setfilternormalangle(cloudlist,40.0f);
		computeangle Ca;
		//设置旋转角度
		Ca.SetRotate(angle_);
		Ca.ComputeDirection(N,Normal_X,Normal_Y);
		//计算RX,RY,RZ
		Ca.computeRPY(N,Normal_X,Normal_Y,RPYList);
		Collision.resize(2);
		Cp.computePickpointsbyWH(cloudlist,Width,Height,center1);
		Ca.Setoffset_x_value(offx);
		Ca.Setoffset_y_value(offy);
		Ca.offsetX(center,Normal_X);
		Ca.offsetY(center,Normal_Y);
		consleXYZRPY(center,RPYList,cloudlist,Collision,Width,Height);
		Pointviewer Pv;
		Pv.simpleVisN(cloud1,center,N,cloudlist,Normal_X,Normal_Y,RPYList); 
	}	
}
