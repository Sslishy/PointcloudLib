#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <ctime>
#include "Planefitting.h"
#include"Computepointspose.h"
#include"computeangle.h"
#include"CollisionDetection.h"
#include"Pointviewer.h"
#include"cylinderfitting.h"
#include"PointProcess.h"
#include"PointCloudAligment.h"
using namespace std;
using namespace cv;
void filternormalZ(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,pcl::Normal normal,pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out)
{
    float distance;
    for (size_t i = 0; i < cloud_in->points.size(); i++)
    {
        distance = cloud_in->points[i].x * normal.normal_x + cloud_in->points[i].y * normal.normal_y
		+cloud_in->points[i].z * normal.normal_z;
		if (distance < 0.004 && distance > -0.004)
		{
			cloud_out->points.push_back(cloud_in->points[i]);
		}
		
    }
    
}
int main(int argc, char** argv) {
	Eigen::Matrix4f transformation;
	Eigen::Matrix4f transformation1;
	pcl::PointCloud<pcl::PointXYZ>::Ptr GoldSimple(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr GoldSimpleorigin(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inorigin(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr center_(new pcl::PointCloud<pcl::PointXYZ>);
	vector<cv::Vec3f> RPY;
	vector<cv::Mat> RPY_;
	RPY_.resize(20);
	double v = 0.001;
	computeangle ca;
	string path;
	if (argc == 2)
	{
		path = argv[1];
	}
	else
	{
		path = "4.pcd";
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in1(new pcl::PointCloud<pcl::PointXYZ>);
	string pcdpath =  "/home/slishy/Code/PCD/hanjie/" + path;
    pcl::io::loadPCDFile("/home/slishy/Code/PCD/hanjie/gold.pcd", *GoldSimple);
	if(pcl::io::loadPCDFile(pcdpath, *cloud_in)<0)
	{
		pcl::console::print_error("load fail ...\n");
		return 0;
	}

   pcl::PointXYZ minpoint,maxpoint;
   pcl::getMinMax3D(*cloud_in,minpoint,maxpoint);
   pcl::Normal normalz;
   PointProcess pp;
   pp.MinValues(maxpoint.y - 0.04); pp.MaxValues(maxpoint.y);
   pp.limitY(cloud_in);
   pp.SetK(50);
   pp.SetStddevMulThresh(1);
   pp.Removepoint(cloud_in);
   
    pcl::getMinMax3D(*GoldSimple,minpoint,maxpoint);
    pp.MinValues(maxpoint.y - 0.04); pp.MaxValues(maxpoint.y);
   pp.limitY(GoldSimple);
   pp.SetK(50);
   pp.SetStddevMulThresh(1);
   pp.Removepoint(GoldSimple);
	pcl::copyPointCloud(*GoldSimple,*GoldSimpleorigin);
	pcl::copyPointCloud(*cloud_in,*cloud_inorigin);	
	pp.SetLeafSize(0.005);
	pp.DownSimple(GoldSimple);
	pp.DownSimple(cloud_in);
	PointCloudAligment Pa;
	Pa.Aligment(GoldSimple,cloud_in,cloud_out);
	Pa.Gettransformation(transformation);
	pcl::transformPointCloud(*GoldSimpleorigin,*GoldSimpleorigin,transformation);
	Pointviewer pv1;
	pv1.simpleVisN(GoldSimpleorigin,cloud_inorigin);
    //*** input point***//
	center_->points.resize(20);
	RPY.resize(20);
	//*** input angle***//
	//cout << euler <<endl;
	cv::Mat_<double> trans = (cv::Mat_<double>(3, 3) <<
	transformation(0, 0), transformation(0, 1), transformation(0, 2), 
	transformation(1, 0), transformation(1, 1), transformation(1, 2), 
	transformation(2, 0), transformation(2, 1), transformation(2, 2)
	);;
	pcl::transformPointCloud(*center_,*center_,transformation);
	for (size_t i = 0; i < center_->points.size(); i++)
	{
		RPY_[i] =  ca.eulerAnglesToRotationMatrix(RPY[i]);
		RPY_[i] =  trans*RPY_[i];
	}
	cout << transformation<<endl;
	/*Mat R = (Mat_<double>(3, 3) <<
		transformation(0,0), transformation(0,1), transformation(0,2),
		transformation(1,0), transformation(1,1), transformation(1,2),
		transformation(2,0), transformation(2,1), transformation(2,2)
		);
	cv::Vec3f angle;
	angle = ca.rotationMatrixToEulerAngles(R);
	cout << angle[0] << " " <<angle[1] << " " <<angle[2] <<endl;*/
	
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(GoldSimpleorigin);
	icp.setInputTarget(cloud_inorigin);
	icp.setMaximumIterations(50);
	icp.setMaxCorrespondenceDistance(0.001);
	icp.setRANSACOutlierRejectionThreshold(0.05);
	icp.setTransformationEpsilon(1e-6);
	//icp.setEuclideanFitnessEpsilon(0.0001);
	pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
              icp.getFitnessScore() << std::endl;
    transformation1 = icp.getFinalTransformation();
    std::cout << transformation1 << std::endl;
	pcl::transformPointCloud(*center_,*center_,transformation1);
	pcl::transformPointCloud(*GoldSimpleorigin,*GoldSimpleorigin,transformation1);
	cv::Mat_<double> trans1 = (cv::Mat_<double>(3, 3) <<
	transformation1(0, 0), transformation1(0, 1), transformation1(0, 2), 
	transformation1(1, 0), transformation1(1, 1), transformation1(1, 2), 
	transformation1(2, 0), transformation1(2, 1), transformation1(2, 2)
	);;
	vector<cv::Vec3f> angle;
	angle.resize(20);
	for (size_t i = 0; i < center_->points.size(); i++)
	{
		RPY_[i] = trans1*RPY_[i] ;
		angle[i] = ca.rotationMatrixToEulerAngles(RPY_[i]);
	}
	for (size_t i = 0; i < 20; i++)
	{
		cout << center_->points[i].x << ";" << center_->points[i].y << ";" <<center_->points[i].z << ";" << angle[i][0] <<";"<< angle[i][1]<<";"<<angle[i][2]<<endl;
	}

	pcl::io::savePCDFile("cloud1.pcd",*cloud_inorigin);
	pcl::io::savePCDFile("cloud2.pcd",*GoldSimpleorigin);
	Pointviewer pv;
	pv.simpleVisN(GoldSimpleorigin,cloud_inorigin);
}

