/*
 * @Author: Slishy 
 * @Date: 2021-07-14 13:37:34 
 * @Last Modified by: mikey.zhaopeng
 * @Last Modified time: 2021-07-16 15:48:09
 */
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/sac_model_parallel_line.h>
#include <ctime>

#include<algorithm>
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
int main(int argc, char** argv) {
	Eigen::Matrix4f transformation;
	pcl::PointCloud<pcl::PointXYZ>::Ptr GoldSimple(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr GoldSimple1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudtest_;
	pcl::PointXYZ center;
	pcl::Normal normal;
	vector<bool> Collision;
	vector<float> w;
	vector<float> h;
	vector<pcl::PointXYZ> orgin;
	vector<pcl::PointCloud<pcl::PointXYZ>> cloudlist;
	vector<pcl::PointCloud<pcl::PointXYZ>> cloudlist1;
	vector<pcl::PointXYZ> planecenter;
	vector<pcl::Normal> Normal_z;
	vector<pcl::Normal> pathNormal;
	vector<pcl::Normal> pathNormal_x;
	vector<pcl::Normal> pathNormal_y;
	vector<Vec3f> RPY;
	//string path = argv[1];
	//string path1 = "/home/slishy/Code/PCD/weld/" + path;
    pcl::io::loadPCDFile("/home/slishy/Code/PCD/weld/1.pcd", *GoldSimple);
	PointProcess pp;
	pp.SetK(6);
	pp.SetStddevMulThresh(1);
	pp.Removepoint(GoldSimple);
	
	//pp.DownSimple(GoldSimple);
	//pp.Removepoint(GoldSimple);
	pcl::copyPointCloud(*GoldSimple,*GoldSimple1);
	Planefitting pf;
	pf.SetRatio(0.2);
	pf.SetDistanceThreshold(0.003);
	pf.extract(GoldSimple,cloudlist);
	pp.SetK(50);
	pp.SetStddevMulThresh(3);
	pp.Removepoint(cloudlist);
	cloudlist1.resize(1);
	pcl::io::savePCDFile("plane.pcd",*cloudlist[0].makeShared());
	pcl::io::savePCDFile("plane1.pcd",*cloudlist[1].makeShared());
	Computepointspose cp;
	cp.computePickPoints(cloudlist,planecenter);
	cp.computePointNormal(cloudlist,planecenter,Normal_z);
	pathNormal.resize(1);	
	if (cloudlist[0].points.size() > cloudlist[1].points.size())
	{
		vector<float> angleout;
		cp.Getangle(angleout);
		float angle;
		cout << angleout[0] <<endl;
		angle = angleout[0] - 20;
		float w_,h_;
		pathNormal[0] = Normal_z[0];
		cout << pathNormal[0] <<endl;
		cp.SetfixedAngle(angle,pathNormal);
		Computepointspose cp1;
		cp1.GetWHWithCorner(cloudlist[0].makeShared(),w_,h_,orgin);		
		/*for (size_t i = 0; i < cloudlist[0].points.size(); i++)
		{
			cloudlist1[0].push_back(cloudlist[0].points[i]);
		}*/
	}
	else{
		vector<float> angleout;

		cp.Getangle(angleout);
		float angle;
		angle = angleout[1] - 20;
		float w_,h_;
		pathNormal[0] = Normal_z[0];
		cp.SetfixedAngle(angle,pathNormal);
		Computepointspose cp1;
		cp1.GetWHWithCorner(cloudlist[1].makeShared(),w_,h_,orgin);
		//pathNormal[0] = Normal_z[1];
		
		//cp.Setlimitangle(20.0f,pathNormal);
		/*for (size_t i = 0; i < cloudlist[1].points.size(); i++)
		{
			cloudlist1[0].push_back(cloudlist[1].points[i]);
		}*/
	}

	if (cloudlist[0].points.size() > cloudlist[1].points.size())
	{
		flipNormalTowardsViewpoint(planecenter[0],0.0f,0.0f,999.0f,pathNormal[0].normal_x,pathNormal[0].normal_y,pathNormal[0].normal_z);
	}
	else{
		flipNormalTowardsViewpoint(planecenter[1],0.0f,0.0f,999.0f,pathNormal[0].normal_x,pathNormal[0].normal_y,pathNormal[0].normal_z);
	}
	
	computeangle ca;
	ca.SetRotate(0.0f);
	ca.ComputeDirection(pathNormal,pathNormal_x,pathNormal_y);
	ca.computeRPY(pathNormal,pathNormal_x,pathNormal_y,RPY);
	cout << RPY[0][0] << " "<<RPY[0][1] << " " << RPY[0][2]<<endl;
	vector<float> d(2);
	for (size_t i = 0; i <	2; i++)
	{
		d[i] = -(Normal_z[i].normal_x * planecenter[i].x + Normal_z[i].normal_y * planecenter[i].y + Normal_z[i].normal_z * planecenter[i].z);
		
	}
	
	//computePickpointsbyWH(cloudlist1,w,h,orgin);
	double a1,b1,c1,d1,a2,b2,c2,d2,tempz,tempy,lineorigin;
	a1 = Normal_z[0].normal_x;
	a2 = Normal_z[1].normal_x;
	b1 = Normal_z[0].normal_y;
	b2 = Normal_z[1].normal_y;
	c1 = Normal_z[0].normal_z;
	c2 = Normal_z[1].normal_z;
	d1 = d[0];
	d2 = d[1];
	float minz = orgin[0].z;
	int minzindex = 0; 
	int minzindex_;
	for (size_t i = 0; i < orgin.size(); i++)
	{
		
		if (orgin[i].z < minz)
		{
			minz = orgin[i].z;
			minzindex = i;

		}
		
	}
	if ((minzindex + 2) > 3)
	{
		minzindex_ = minzindex - 2;
	}
	else{
		minzindex_ = minzindex +2;
	}
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	if (cloudlist[0].points.size() > cloudlist[1].points.size())
	{
		kdtree.setInputCloud(cloudlist[0].makeShared());
		int k = 1;
		std::vector<int> pointIdxNKNSearch(k);
    	std::vector<float> pointNKNSquaredDistance(k);
		kdtree.nearestKSearch(orgin[minzindex],k,pointIdxNKNSearch,pointNKNSquaredDistance);
		orgin[minzindex] = cloudlist[0].points[pointIdxNKNSearch[0]];
	}
	else
	{
		kdtree.setInputCloud(cloudlist[1].makeShared());
		int k = 1;
		std::vector<int> pointIdxNKNSearch(k);
    	std::vector<float> pointNKNSquaredDistance(k);
		kdtree.nearestKSearch(orgin[minzindex],k,pointIdxNKNSearch,pointNKNSquaredDistance);
		orgin[minzindex] = cloudlist[1].points[pointIdxNKNSearch[0]];
	}
	lineorigin = orgin[minzindex].x;
	tempz= ((d2+ a2 * lineorigin)/b2 - (d1 + a1 * lineorigin)/b1)/(c1 / b1 - c2 / b2);
    tempy= -(c1 * tempz + d1 + a1 * lineorigin)/b1;
	float tempy1 = orgin[minzindex].y;
	float tempz1 = orgin[minzindex].z;
	double line_direction_x , line_direction_y , line_direction_z;
	line_direction_x = b1*c2 - c1*b2;
    line_direction_y = c1*a2 - a1*c2;
    line_direction_z = a1*b2 - b1*a2;
	pcl::Normal linenormal;
	linenormal.normal_x = line_direction_x;
	linenormal.normal_y = line_direction_y;
	linenormal.normal_z = line_direction_z;
	
	flipNormalTowardsViewpoint(orgin[minzindex], orgin[minzindex_].x, orgin[minzindex_].y, orgin[minzindex_].z, linenormal.normal_x, linenormal.normal_y, linenormal.normal_z);
	pcl::ModelCoefficients coeffs;
	coeffs.values.push_back(lineorigin);
	coeffs.values.push_back(tempy1);
	coeffs.values.push_back(tempz1);
	coeffs.values.push_back(linenormal.normal_x * 0.28);
	coeffs.values.push_back(linenormal.normal_y * 0.28);
	coeffs.values.push_back(linenormal.normal_z * 0.28);
	vector<pcl::PointXYZ> weldpath;
	for (size_t i = 1; i < 10; i++)
	{
		if (i == 1)
		{
			cout << lineorigin  << ";" <<tempy1 <<";" <<tempz1<< endl;
		}
		if (i == 9)
		{
			i = 10;
		}
		
		cout << lineorigin + (linenormal.normal_x * 0.28) * 0.1 * i << ";" <<tempy1 + (linenormal.normal_y * 0.28) * 0.1 * i<<";" <<tempz1 + (linenormal.normal_z * 0.28) * 0.1 * i << endl;
		//weldpath[0].x =
	}
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D"));
	viewer->addPointCloud(GoldSimple1,"cloud");
	viewer->addLine(coeffs,"line");
	viewer->addCoordinateSystem(0.2);
	viewer->addSphere(orgin[minzindex],0.009 ,"shap");
	viewer->addSphere(orgin[minzindex_],0.005,"shap2");
	viewer->addSphere(orgin[0],0.005,"shap3");
	viewer->addSphere(orgin[1],0.005,"shap4");
	viewer->addSphere(orgin[2],0.005,"shap5");
	viewer->addSphere(orgin[3],0.005,"shap6");
	for (size_t i = 1; i < 10; i++)
	{
		if (i == 1)
		{
			viewer->addCoordinateSystem(0.03,lineorigin ,tempy1,tempz1 ,to_string(i));
		}
		if (i == 9)
		{
			i = 10;
		}
		viewer->addCoordinateSystem(0.03,lineorigin + (linenormal.normal_x * 0.28) * 0.1 * i,tempy1+(linenormal.normal_y * 0.28) * 0.1 * i,tempz1 + (linenormal.normal_z * 0.28) * 0.1 * i,to_string(i));
	}
	pcl::ModelCoefficients coeffs1;
	coeffs1.values.push_back(planecenter[0].x);
	coeffs1.values.push_back(planecenter[0].y);
	coeffs1.values.push_back(planecenter[0].z);
	coeffs1.values.push_back(Normal_z[0].normal_x);
	coeffs1.values.push_back(Normal_z[0].normal_y);
	coeffs1.values.push_back(Normal_z[0].normal_z);
	viewer->addLine(coeffs1,"line12");
	//viewer->addPlane(coeffsplane,"plane");
	//viewer->addPlane(coeffsplane1,"plane1");
	while (!viewer->wasStopped())
	{ 
	  viewer->spinOnce();
	}

}

