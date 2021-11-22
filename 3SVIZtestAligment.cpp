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
void consleXYZRPY(vector<pcl::PointXYZ> center, vector<cv::Vec3f> RPYList, vector<pcl::PointCloud<pcl::PointXYZ>> cloudlist, vector<bool> Collision)
{
	for (size_t i = 0; i < center.size(); i++)
	{
		cout << center[i].x << ";" << center[i].y << ";" << center[i].z << ";" << RPYList[i][0] << ";" << RPYList[i][1] << ";" << RPYList[i][2] << ";" << cloudlist[i].size() << ";" << boolalpha << Collision[i] << ";" << endl;
	}
}
void Aligment1(pcl::PointCloud<pcl::PointXYZ>::Ptr goldsimple,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr AligmentPointCloud_)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr goldsimple_(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in_(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_out(new pcl::PointCloud<pcl::PointNormal>);
    pcl::copyPointCloud(*goldsimple,*goldsimple_);
    pcl::copyPointCloud(*cloud_in,*cloud_in_);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr goldsimple_features(new pcl::PointCloud<pcl::FPFHSignature33>);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloudin_features(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::NormalEstimationOMP<pcl::PointNormal, pcl::PointNormal> nest;
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
	nest.setNumberOfThreads(8);
	nest.setRadiusSearch(0.01);
	nest.setSearchMethod(tree);
	//nest.setInputCloud(goldsimple_);
	//nest.compute(*goldsimple_);
	nest.setInputCloud(cloud_in_);
	nest.compute(*cloud_in_);
    pcl::FPFHEstimationOMP<pcl::PointNormal,pcl::PointNormal,pcl::FPFHSignature33> fest;
    fest.setNumberOfThreads(8);
	fest.setRadiusSearch(0.055);
	//fest.setInputCloud(goldsimple_);
	//fest.setSearchMethod(tree);
	//fest.setInputNormals(goldsimple_);
	//fest.compute(*goldsimple_features);
	//pcl::io::savePCDFile("goldsimple_features.pcd",*goldsimple_features);
	pcl::io::loadPCDFile("goldsimple_features.pcd",*goldsimple_features);
	fest.setInputCloud(cloud_in_);
	fest.setInputNormals(cloud_in_);
	fest.setSearchMethod(tree);
	fest.compute(*cloudin_features);
	pcl::SampleConsensusPrerejective<pcl::PointNormal,pcl::PointNormal,pcl::FPFHSignature33> align_;
	align_.setInputSource(goldsimple_);
	align_.setSourceFeatures(goldsimple_features);
	align_.setInputTarget(cloud_in_);
	align_.setTargetFeatures(cloudin_features);
	align_.setMaximumIterations(50000); // Number of RANSAC iterations
	align_.setNumberOfSamples(3); // 在对象和场景之间进行采样的点对应数，至少需要N个点才能进行计算
	align_.setCorrespondenceRandomness(5); // 在N个最佳匹配之间进行性随机选择
	align_.setSimilarityThreshold(0.8f); // 根据采样之间的距离位置不变的几何一致性，尽早消除不良影响
	align_.setMaxCorrespondenceDistance(2.5f * 0.005f); // 欧几里德距离阈值，用于确定变换后的点云是否正确对齐
	align_.setInlierFraction(0.80f); // Required inlier fraction for accepting a pose hypothesis
	{
		pcl::ScopeTime t("Alignment");
		align_.align(*cloud_out);
        pcl::copyPointCloud(*cloud_out,*AligmentPointCloud_);
	}
	if (align_.hasConverged())
	{	
		//m_transformation = align_.getFinalTransformation();
		/*pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(0, 0), transformation(0, 1), transformation(0, 2));
		pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", transformation(1, 0), transformation(1, 1), transformation(1, 2));
		pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(2, 0), transformation(2, 1), transformation(2, 2));
		pcl::console::print_info("\n");
		pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n", transformation(0, 3), transformation(1, 3), transformation(2, 3));
		pcl::console::print_info("\n");
		pcl::console::print_info("Inliers: %i/%i\n", align.getInliers().size(), object->size());*/
	}
	else
	{
		pcl::console::print_error("Alignment failed!\n");
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
	/*for (size_t i = 0; i < cloud_in->points.size(); i++)
	{
		cloud_in->points[i].x = cloud_in->points[i].x * 0.001;
        cloud_in->points[i].y = cloud_in->points[i].y * 0.001;
        cloud_in->points[i].z = cloud_in->points[i].z * 0.001;
	}
	for (size_t i = 0; i < GoldSimple->points.size(); i++)
	{
		GoldSimple->points[i].x = GoldSimple->points[i].x * 0.001;
        GoldSimple->points[i].y = GoldSimple->points[i].y * 0.001;
        GoldSimple->points[i].z = GoldSimple->points[i].z * 0.001;
	}*/
	//Planefitting pf;
	//pf.SetDistanceThreshold(0.08);
	//pf.extract(cloud_in1,cloud_in,"false");
	PointProcess pp;
	//pp.SetK(100);
	//pp.SetStddevMulThresh(1);
	//pp.Removepoint(cloud_in);
	//pp.Removepoint(cloud_in);
//	pp.smoothxyz(cloud_in);
	//pp.smoothxyz(GoldSimple);
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
	center_->points[0] = pcl::PointXYZ(1093.1 * v,-168.1 * v,494.5 * v);
	center_->points[1] = pcl::PointXYZ(1162.2 * v, -164.0 * v,486.4 * v);
	center_->points[2] = pcl::PointXYZ(1006.001 * v, -474.136 * v,225.678 * v);
	center_->points[3] = pcl::PointXYZ(1008.837 * v, -472.487 * v,176.974 * v);
	center_->points[4] = pcl::PointXYZ(1010.208 * v, -463.066 * v,171.763 * v);
	center_->points[5] = pcl::PointXYZ(1013.453 * v, -473.073 * v,170.252 * v);
	center_->points[6] = pcl::PointXYZ(1031.581 * v, -462.581 * v,197.897 * v);
	center_->points[7] = pcl::PointXYZ(1031.581 * v, -462.581 * v,326.436 * v);
	center_->points[8] = pcl::PointXYZ(1031.581 * v, -462.581 * v,326.436 * v);
	center_->points[9] = pcl::PointXYZ(1148.688 * v, -472.908 * v,222.566 * v);
	center_->points[10] = pcl::PointXYZ(1146.694 * v, -472.534 * v,179.625 * v);
	center_->points[11] = pcl::PointXYZ(1145.043 * v, -472.539 * v,172.673 * v);
	center_->points[12] = pcl::PointXYZ(1139.435 * v, -472.526 * v,169.698 * v);
	center_->points[13] = pcl::PointXYZ(1137.275 * v, -471.892 * v,169.701 * v);
	center_->points[14] = pcl::PointXYZ(1074.157 * v, -471.895 * v,169.695 * v);
	center_->points[15] = pcl::PointXYZ(1030.891 * v, -471.897 * v,169.703 * v);
	center_->points[16] = pcl::PointXYZ(1015.212 * v, -471.895 * v,169.706 * v);
	center_->points[17] = pcl::PointXYZ(1024.170 * v, -469.470 * v,173.976 * v);
	center_->points[18] = pcl::PointXYZ(1052.178 * v, -458.959 * v,191.007 * v);
	center_->points[19] = pcl::PointXYZ(1052.178 * v, -458.959 * v,470.465 * v);
	RPY.resize(20);
	//*** input angle***//
	RPY[0] = Vec3f(-176.502 , -40.805 , -136.725);
	RPY[1] = Vec3f(-176.501 , -40.804 , -136.725);
	RPY[2] = Vec3f(-176.501 , -40.804 , -152.365);
	RPY[3] = Vec3f(-176.501 , -40.804 , -152.365);
	RPY[4] = Vec3f(-176.501 , -40.804 , -152.365);
	RPY[5] = Vec3f(-176.501 , -40.804 , -152.365);
	RPY[6] = Vec3f(-176.501 , -40.804 , -152.365);
	RPY[7] = Vec3f(-176.501 , -40.804 , -152.365);
	RPY[8] = Vec3f(-159.016 , -35.708 , -74.220);
	RPY[9] = Vec3f(-159.016 , -35.708 , -74.220);
	RPY[10] = Vec3f(-159.016 , -35.708 , -74.220);
	RPY[11] = Vec3f(-159.016 , -35.708 , -74.220);
	RPY[12] = Vec3f(-159.016 , -35.708 , -74.220);
	RPY[13] = Vec3f(-159.016 , -35.708 , -98.022);
	RPY[14] = Vec3f(-159.016 , -35.708 , -120.475);
	RPY[15] = Vec3f(-159.016 , -35.708 , -124.594);
	RPY[16] = Vec3f(-159.016 , -35.708 , -146.308);
	RPY[17] = Vec3f(-159.016 , -35.708 , -146.308);
	RPY[18] = Vec3f(-159.016 , -35.708 , -146.308);
	RPY[19] = Vec3f(-159.016 , -35.708 , -146.308);
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

