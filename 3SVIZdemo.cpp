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
	float Width;
	float Height;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_dst(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	vector<pcl::PointXYZ> center;
	vector<pcl::PointXYZ> contours;
	pcl::PointXYZ center_mean;
	pcl::PointXYZ maxpoint;
	pcl::PointXYZ minpoint;
	vector<bool> Collision;
	string path1 = argv[1];
    string f = "false";
	//string path = "/home/td/Code/PCD/" + path1;
	//string pathwhite = "/home/td/Code/PCD/"+path1;
	string path = "/home/slishy/Code/PCD/box/" + path1;
	if (pcl::io::loadPCDFile(path, *cloud) < 0)
		{
			cout << "load fail . . . ." << endl;
			return 0;
		}
	    *cloud1 = *cloud;
		PointProcess pp;
		// 点云过滤
        
		pp.Removepoint(cloud);
        pcl::io::savePCDFile( "/home/slishy/Code/PCD/box/test1.pcd",*cloud);
		//提取平面点云
		Planefitting plane;
		plane.SetDistanceThreshold(0.05);
		plane.SetMaxIterations(100);
		plane.extract(cloud,cloud_dst,f);
        pcl::io::savePCDFile( "/home/slishy/Code/PCD/box/test2.pcd",*cloud_dst);
        Computepointspose Cp;
		Cp.GetWHWithCorner(cloud_dst,Width,Height,contours);
        for (size_t i = 0; i < contours.size(); i++)
        {
            cout << contours[i].x <<";" << contours[i].y << ";" << contours[i].z <<endl;
        }
		Pointviewer Pv;
		Pv.simpleVisN(cloud_dst);  
	
}
