#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
 #include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/boundary.h>
#include <math.h>
#include <boost/make_shared.hpp>
#include <pcl/common/common.h>
#include<pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/features/normal_3d.h>
#include"Planefitting.h"
#include <pcl/filters/covariance_sampling.h>
#include <pcl/filters/normal_space.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
using namespace cv;
void consleXYZRPY(vector<pcl::PointXYZ> center, vector<cv::Vec3f> RPYList, vector<pcl::PointCloud<pcl::PointXYZ>> cloudlist, vector<bool> Collision)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Vector4f pcacentroid;
	pcl::compute3DCentroid(*cloud_in, pcacentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloud_in, pcacentroid, covariance);
	//计算矩阵的特征值和特征向量
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
	Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
	projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
	projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pcacentroid.head<3>());
	pcl::transformPointCloud(*cloud_in, *cloud1, projectionTransform);
	pcl::PointXYZ minPoint, maxPoint;
	pcl::getMinMax3D(*cloud1, minPoint, maxPoint);
	vector<int> index;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_dst(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_dst1(new pcl::PointCloud<pcl::PointXYZ>);
	 pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
     kdtree.setInputCloud(cloud1);
	 for (size_t i = 0; i < cloud1->points.size(); i++)
	 {
		pcl::PointXYZ searchPoint = cloud1->points[i];
		float radius = 0.005;
		std::vector<int> pointIdxRadiusSearch;
    	std::vector<float> pointRadiusSquaredDistance;
		kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
		if (pointIdxRadiusSearch.size() < 70)
		{
			for (size_t j = 0; j < pointIdxRadiusSearch.size(); j++)
			{
				cloud_dst->push_back(cloud1->points[pointIdxRadiusSearch[j]]);
			}
		}
		
	 }
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree1;
	kdtree1.setInputCloud(cloud_dst);
	for (size_t i = 0; i < cloud_dst->points.size(); i++)
	{
	pcl::PointXYZ searchPoint = cloud_dst->points[i];
	float radius = 0.005;
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	kdtree1.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
	if (pointIdxRadiusSearch.size() < 60)
	{
		for (size_t j = 0; j < pointIdxRadiusSearch.size(); j++)
		{
			cloud_dst1->push_back(cloud1->points[pointIdxRadiusSearch[j]]);
		}
	}
	}                                                
	pcl::io::savePCDFile("/home/slishy/Code/PCD/hanjie/test.pcd",*cloud_dst);
    pcl::io::savePCDFile("/home/slishy/Code/PCD/hanjie/test1.pcd",*cloud_dst1);
	//pcl::io::savePCDFile("/home/slishy/Code/PCD/hanjie/test.pcd",*cloud);
}
 void Pointcloud_2_2DGetcontours(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointXYZ& center)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Vector4f pcacentroid;
	pcl::compute3DCentroid(*cloud_in, pcacentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloud_in, pcacentroid, covariance);
	//计算矩阵的特征值和特征向量
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
	Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
	projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
	projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pcacentroid.head<3>());
	pcl::transformPointCloud(*cloud_in, *cloud1, projectionTransform);
	pcl::PointXYZ minPoint, maxPoint;
	pcl::getMinMax3D(*cloud1, minPoint, maxPoint);
	vector<cv::Point2f> point22d;
	point22d.resize(cloud1->points.size());
	//pcl::io::savePCDFileASCII("cloud",*cloud1);
	Mat src1 = Mat::zeros(Size(900, 900), CV_8UC1);
	Mat src2 = Mat::zeros(Size(900, 900), CV_8UC1);
	for (size_t i = 0; i < cloud1->points.size(); i++)
	{
		int x,y;
		x = cloud1->points[i].z * 1000 +100;
		y = cloud1->points[i].y * 1000 +100;
		src1.at<uchar>(1,1) = 255;
		src1.at<uchar>(x,y) = 255;
	}
	vector<vector<Point>> contours; vector<Vec4i> hierarchy;
	findContours(src1, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
	int max = 0;
	int index;
	vector<int> num;
	num.resize(contours.size());
	for (size_t i = 0; i < num.size(); i++)
	{
		num[i] =contours[i].size();
	}
}
int main(int argc, char** argv) {
	Eigen::Matrix4f transformation;
	pcl::PointCloud<pcl::PointXYZ>::Ptr GoldSimple(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr GoldSimple1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointXYZ center;
	pcl::Normal normal;
	vector<bool> Collision;
	vector<pcl::PointCloud<pcl::PointXYZ>> cloudlist;
	vector<pcl::PointXYZ> planecenter;
	vector<pcl::Normal> Normal_z;
    pcl::io::loadPCDFile("/home/slishy/Code/PCD/weld/removehanjietest1.pcd", *GoldSimple);
	if(pcl::io::loadPCDFile("/home/slishy/Code/PCD/weld/test4.pcd", *cloud_in)<0)
	{
		pcl::console::print_error("load fail ...\n");
		return 0;
	}
	pcl::copyPointCloud(*GoldSimple,*GoldSimple1);
    center.x =0;
    center.y = 0;
    center.z = 0;
	Planefitting pf;
	pf.SetRatio(0.2);
	pf.SetDistanceThreshold(0.001);
	pf.extract(GoldSimple,cloudlist);
	Computepointspose cp;
	cp.computePickPoints(cloudlist,planecenter);
	cp.computePointNormal(cloudlist,planecenter,Normal_z);
	cout<<Normal_z[0] <<endl;
	cout<<Normal_z[1] <<endl;
	vector<float> d(2);
	for (size_t i = 0; i <	2; i++)
	{
		d[i] = -(Normal_z[i].normal_x * planecenter[i].x + Normal_z[i].normal_y * planecenter[i].y + Normal_z[i].normal_z * planecenter[i].z);
		cout <<d[i] <<endl;
	}
	double a1,b1,c1,d1,a2,b2,c2,d2,tempz,tempy,lineorigin;
	a1 = Normal_z[0].normal_x;
	a2 = Normal_z[1].normal_x;
	b1 = Normal_z[0].normal_y;
	b2 = Normal_z[1].normal_y;
	c1 = Normal_z[0].normal_z;
	c2 = Normal_z[1].normal_z;
	d1 = d[0];
	d2 = d[1];
	lineorigin = 1.357316;
	tempz= ((d2+ a2 * lineorigin)/b2 - (d1 + a1 * lineorigin)/b1)/(c1 / b1 - c2 / b2);
    tempy= -(c1 * tempz + d1 + a1 * lineorigin)/b1;
	double line_direction_x , line_direction_y , line_direction_z;
	line_direction_x = b1*c2 - c1*b2;
    line_direction_y = c1*a2 - a1*c2;
    line_direction_z = a1*b2 - b1*a2;
	pcl::ModelCoefficients coeffs;
	coeffs.values.push_back(lineorigin);
	coeffs.values.push_back(tempy);
	coeffs.values.push_back(tempz);
	coeffs.values.push_back(line_direction_x);
	coeffs.values.push_back(line_direction_y);
	coeffs.values.push_back(line_direction_z);
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D"));
	viewer->addPointCloud(GoldSimple1,"cloud");
	viewer->addLine(coeffs,"line");
	viewer->addCoordinateSystem(0.2);
	//viewer->addPlane(coeffsplane,"plane");
	//viewer->addPlane(coeffsplane1,"plane1");
	while (!viewer->wasStopped())
		{ 
	{
		{ 
	{
		{ 
	{
		{ 
		viewer->spinOnce();
	}

	//可视化
	boost::shared_ptr<pcl::visualization::PCLVisualizer> MView (new pcl::visualization::PCLVisualizer ("点云库PCL从入门到精通案例"));
	
	int v1(0); 
	MView->createViewPort (0.0, 0.0, 0.5, 1.0, v1); 
	MView->setBackgroundColor (0.3, 0.3, 0.3, v1); 
	MView->addText ("Raw point clouds", 10, 10, "v1_text", v1); 
	int v2(0); 
	MView->createViewPort (0.5, 0.0, 1, 1.0, v2); 
	MView->setBackgroundColor (0.5, 0.5, 0.5, v2); 
	MView->addText ("Boudary point clouds", 10, 10, "v2_text", v2); 
 
	MView->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud",v1);
	MView->addPointCloud<pcl::PointXYZ> (cloud_boundary, "cloud_boundary",v2);
	MView->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,0, "sample cloud",v1);
	MView->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0,1,0, "cloud_boundary",v2);
	MView->addCoordinateSystem (0.5);
	MView->initCameraParameters ();
 
	MView->spin();
 
	return 0; 
} 
int
	main(int argc, char** argv)
{
	//srand(time(NULL));
 
	//Computepointspose cp;
	//cp.computePickPoints(GoldSimple,center);
	//cp.computePointNormal(GoldSimple,center,normal);
	/*Planefitting pf;
	pf.SetDistanceThreshold(0.005);
	pf.extract(cloud_in,cloud_in_);*/
	//PointProcess pp;
	//pp.DownSimple(cloud_in);
    //pcl::io::savePCDFile("/home/slishy/Code/PCD/weld/removehanjietest.pcd",*GoldSimple);
	//pp.DownSimple(cloud_in);
	//pp.DownSimple(GoldSimple);
	//PointCloudAligment Pa;
	//Pa.Aligment(GoldSimple,cloud_in,cloud_out);
	//Pa.Gettransformation(transformation);
	//cout << transformation<<endl;
	/*Eigen::Vector3f T;
	pcl::PointCloud<pcl::PointXYZ>::Ptr center_(new pcl::PointCloud<pcl::PointXYZ>);
	center_->points.resize(1);
	center_->points[0].x = center.x;
	center_->points[0].y = center.y;
	center_->points[0].z = center.z;
	pcl::transformPointCloud(*center_,*center_,transformation);
	center.x = center_->points[0].x;
	center.y = center_->points[0].y;
	center.z = center_->points[0].z;
	cv::Vec3f RPY;
	RPY[0] = 0;
	RPY[1] = 0;
	RPY[2] = 170;
	computeangle ca;
	cv::Mat RPY_;
	RPY_ =  ca.eulerAnglesToRotationMatrix(RPY);
	cv::Mat_<double> trans = (cv::Mat_<double>(3, 3) <<
	transformation(0, 0), transformation(0, 1), transformation(0, 2), 
	transformation(1, 0), transformation(1, 1), transformation(1, 2), 
	transformation(2, 0), transformation(2, 1), transformation(2, 2)
	);;
	cv::Vec3f RPY1 = ca.rotationMatrixToEulerAngles(trans); 
	cout << RPY1 <<endl;
	RPY_ = RPY_ * trans ;
	cv::Vec3f RPY2 = ca.rotationMatrixToEulerAngles(RPY_);   
	cout << RPY2 <<endl;*/
	//Pointviewer pv;
	//pv.simpleVisN(cloud_in,cloud_out,center);
    //Planefitting pf;
}