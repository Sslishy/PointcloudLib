#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <ctime>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d_omp.h>
#include "Planefitting.h"
#include"Computepointspose.h"
#include"computeangle.h"
#include"CollisionDetection.h"
#include"Pointviewer.h"
//#include"Pointviewer.h"
#include"cylinderfitting.h"
using namespace std;
using namespace cv;
void removepoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{ 
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    // 设置平均距离估计的最近邻居的数量K
    sor.setMeanK (50);
    // 设置标准差阈值系数
    sor.setStddevMulThresh (0.05);
    // 执行过滤
    sor.filter (*cloud);

}
void removepoint(vector<pcl::PointCloud<pcl::PointXYZ>> &cloudlist)
{
	for (size_t i = 0; i < cloudlist.size(); i++)
	{
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloudlist[i].makeShared());
    // 设置平均距离估计的最近邻居的数量K
    sor.setMeanK (10);
    // 设置标准差阈值系数
    sor.setStddevMulThresh (0.05);
    // 执行过滤
    sor.filter (*cloudlist[i].makeShared());
	}
}
void removepoint1(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{ 
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    // 设置平均距离估计的最近邻居的数量K
    sor.setMeanK (50);
    // 设置标准差阈值系数
    sor.setStddevMulThresh (0.05);
    // 执行过滤
    sor.filter (*cloud);
}
void consleXYZRPY(vector<pcl::PointXYZ> center, vector<cv::Vec3f> RPYList, vector<pcl::PointCloud<pcl::PointXYZ>> cloudlist, vector<bool> Collision)
{
	for (size_t i = 0; i < center.size(); i++)
	{
		cout << center[i].x << ";" << center[i].y << ";" << center[i].z << ";" << RPYList[i][0] << ";" << RPYList[i][1] << ";" << RPYList[i][2] << ";" << cloudlist[i].size() << ";" << boolalpha << Collision[i] << ";" << endl;
	}

}
void downsimple(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
	pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud);
}
void computePickpointsbyWH(const vector<pcl::PointCloud<pcl::PointXYZ>> cloudlist,vector<pcl::PointXYZ>& center)
{
	center.resize(cloudlist.size());
	for (size_t i = 0; i < cloudlist.size(); i++)
	{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Vector4f pcacentroid;
	pcl::compute3DCentroid(*cloudlist[0].makeShared(), pcacentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloudlist[0].makeShared(), pcacentroid, covariance);
	//计算矩阵的特征值和特征向量
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
	Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
	projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
	projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pcacentroid.head<3>());
	pcl::transformPointCloud(*cloudlist[i].makeShared(), *cloud1, projectionTransform);
	pcl::PointXYZ minPoint, maxPoint;
	pcl::getMinMax3D(*cloud1, minPoint, maxPoint);
	const Eigen::Vector3f meanDiagonal = 0.5 * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());
	const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA);
	const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcacentroid.head<3>();
	vector<cv::Point2f> point22d;
	point22d.resize(cloud1->points.size());
	for (size_t i = 0; i < cloud1->points.size(); i++)
	{
		point22d[i].x = cloud1->points[i].z+100;
		point22d[i].y = cloud1->points[i].y+100;
	}
	cv::RotatedRect pointbox = minAreaRect(point22d);
	Point2f rect[4];
	pointbox.points(rect);
	pcl::PointXYZ point1,point2,point3,point4,point5;
	point1.x = 0;
	point1.y = pointbox.center.y - 100;
	point1.z = pointbox.center.x - 100;
	/*point2.x = 0;
	point2.y = rect[0].y - 100;
	point2.z = rect[0].x - 100;
	point3.x = 0;
	point3.y = rect[1].y - 100;
	point3.z = rect[1].x - 100;
	point4.x = 0;
	point4.y = rect[2].y - 100;
	point4.z = rect[2].x - 100;
	point5.x = 0;
	point5.y = rect[3].y - 100;
	point5.z = rect[3].x - 100;*/
	Eigen::Affine3f transform(Eigen::Affine3d::Identity());
    transform.translate(bboxTransform);
	//将center转到真实物件上
	point1 = pcl::transformPoint(point1,transform);
	center[i] = point1;
	}
}
int main(int argc, char** argv) {
	const clock_t begin_time = clock( );
	vector<pcl::PointCloud<pcl::PointXYZ>> cloudlist;
	vector<pcl::Normal> N;
	vector<pcl::Normal> Normal_X;
	vector<pcl::Normal> Normal_Y;
	vector<cv::Vec3f> RPYList;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	vector<pcl::PointXYZ> center;
	vector<bool> Collision;
	string path1 = argv[1];
	string path = "/home/slishy/Code/PCD/test/" + path1;
	if (pcl::io::loadPCDFile(path, *cloud) < 0)
	{
		cout << "load fail . . . ." << endl;
		return 0;
	}
	*cloud1 = *cloud;
	//downsimple(cloud);
	
	/*Planefitting plane;
	plane.SetDistanceThreshold(0.001);
	plane.SetMaxIterations(100);
	plane.extract(cloud1,cloudlist);*/
	string A = "1";
	string B = "0";
	string offx = argv[4];
	string angle = argv[3];
	float angle_ = atof(angle.c_str());
	float offx_ = atof(offx.c_str());
	if (argv[2] == A)
	{	
		removepoint(cloud);
		removepoint1(cloud);
		cylinderfitting cf;
		cf.SetDistanceThreshold(0.04);
		cf.SetMaxIterations(400);
		cf.SetMaxRadiusLimits(0.009);
		cf.SetMinRadiusLimits(0.003);
		cf.SetNormalDistanceWeight(0.1);
		cf.extract(cloud,cloudlist);
		for (size_t i = 0; i < cloudlist.size(); i++)
		{
			if (cloudlist[i].points.size() == 0)
			{
				Planefitting plane;
				plane.SetDistanceThreshold(0.004);
				plane.SetMaxIterations(100);
				plane.extract(cloud,cloudlist);
			}
		}
		Computepointspose Cp;
		Cp.SetFindNum(999);
		Cp.computePickPoints(cloudlist,center);
		Cp.computePointNormal(cloudlist,center,N);
		vector<float> angle(cloudlist.size());
		Cp.Getangle(angle);
		//Cp.Setfilternormalangle(cloudlist,40.0f);
		//cout << angle[0] << endl;
		computeangle Ca;
		Ca.SetRotate(angle_);
		Ca.ComputeDirection(N,Normal_X,Normal_Y);
		Ca.computeRPY(N,Normal_X,Normal_Y,RPYList);
		CollisionDetection Cd;
		//Cd.CollisionDetection_(center,N,Collision);
		Collision.resize(2);
		consleXYZRPY(center,RPYList,cloudlist,Collision);
		Pointviewer Pv;
		Pv.simpleVisN(cloud1,center,N,cloudlist,Normal_X,Normal_Y,RPYList);
	}
	if (argv[2] == B)
	{
	removepoint(cloud);
	removepoint(cloud);
	cylinderfitting cf;
	cf.SetDistanceThreshold(0.005);
	cf.SetMaxIterations(400);
	cf.SetMaxRadiusLimits(0.005);
	cf.SetMinRadiusLimits(0.002);
	cf.SetNormalDistanceWeight(0.01);
	cf.extract(cloud,cloudlist);
	for (size_t i = 0; i < cloudlist.size(); i++)
		{
			if (cloudlist[i].points.size() == 0)
			{
				Planefitting plane;
				plane.SetRatio(0.8);
				plane.SetDistanceThreshold(0.001);
				plane.SetMaxIterations(100);
				plane.extract(cloud,cloudlist);
			}
		}
	
	Computepointspose Cp;
	Cp.SetFindNum(999);
	Cp.computePickPoints(cloudlist,center);
	Cp.computePointNormal(cloudlist,center,N);
	vector<float> angle(cloudlist.size());
	Cp.Getangle(angle);
	//Cp.Setfilternormalangle(cloudlist,40.0f);
	Cp.Setlimitangle(0,N);
	//Cp.Get2DAngle(cloudlist,angle);
	
	/*Eigen::Vector4f pcacentroid;
	pcl::compute3DCentroid(*cloudlist[0].makeShared(), pcacentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloudlist[0].makeShared(), pcacentroid, covariance);
	//计算矩阵的特征值和特征向量
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	cv::Mat_<double> trans1 = (cv::Mat_<double>(3, 3) <<
	eigenVectorsPCA(0, 0), eigenVectorsPCA(0, 1), eigenVectorsPCA(0, 2), 
	eigenVectorsPCA(1, 0), eigenVectorsPCA(1, 1), eigenVectorsPCA(1, 2), 
	eigenVectorsPCA(2, 0), eigenVectorsPCA(2, 1), eigenVectorsPCA(2, 2)
	);;
	*/
	 pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud (cloudlist[0].makeShared());
    feature_extractor.compute ();
    std::vector <float> moment_of_inertia;
    std::vector <float> eccentricity;
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;
	feature_extractor.getMomentOfInertia (moment_of_inertia);
    feature_extractor.getEccentricity (eccentricity);
    // 获取OBB盒子
    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_extractor.getEigenValues (major_value, middle_value, minor_value);
    // 获取主轴major_vector，中轴middle_vector，辅助轴minor_vector
    feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
	
	cv::Mat_<double> trans1 = (cv::Mat_<double>(3, 3) <<
	major_vector(0), major_vector(1), major_vector(2), 
	middle_vector(0), middle_vector(1), middle_vector(2), 
	minor_vector(0), minor_vector(1), minor_vector(2)
	);;
	/*pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	  Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
    Eigen::Quaternionf quat (rotational_matrix_OBB);
    // position：中心位置
    // quat：旋转矩阵
    // max_point_OBB.x - min_point_OBB.x  宽度
    // max_point_OBB.y - min_point_OBB.y  高度
    // max_point_OBB.z - min_point_OBB.z  深度
	viewer->addPointCloud(cloudlist[0].makeShared(),"1");
    viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");

    pcl::PointXYZ center_ (center[0].x, center[0].y, center[0].z);
    pcl::PointXYZ x_axis (major_vector (0) + center_.x, major_vector (1) + center_.y, major_vector (2) + center_.z);
    pcl::PointXYZ y_axis (middle_vector (0) + center_.x, middle_vector (1) + center_.y, middle_vector (2) + center_.z);
    pcl::PointXYZ z_axis (minor_vector (0) + center_.x, minor_vector (1) + center_.y, minor_vector (2) + center_.z);
    viewer->addLine (center_, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
    viewer->addLine (center_, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
    viewer->addLine (center_, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");
    while(!viewer->wasStopped())
    {
        viewer->spinOnce (100);
        
    }*/
	computeangle Ca;
	Vec3f RPY;
	RPY = Ca.rotationMatrixToEulerAngles(trans1);
	cout << RPY <<endl;
	float a = -RPY[2];
	cout << a <<endl;
	if (angle_> a - 90 && angle_ < a + 90)
	{
		Ca.SetRotate(a);
	}
	else
	{ 
		Ca.SetRotate(a + 180);
		
	}
	Ca.ComputeDirection(N,Normal_X,Normal_Y);
	Ca.computeRPY(N,Normal_X,Normal_Y,RPYList);
	Ca.Setoffset_x_value(offx_);
	Ca.offsetX(center,Normal_X);
	Ca.Setoffset_y_value(-0.004);
	Ca.offsetY(center,Normal_Y);
	CollisionDetection Cd;
	//Cd.CollisionDetection_(center,N,Collision);
	Collision.resize(2);
	consleXYZRPY(center,RPYList,cloudlist,Collision);

	Pointviewer Pv;
	Pv.simpleVisN(cloud1,center,N,cloudlist,Normal_X,Normal_Y,RPYList);
	float seconds = float(clock( ) - begin_time)/CLOCKS_PER_SEC;
	int seconds_ = int(seconds)%60;
	//cout << seconds <<endl;
	
	return 0;
}
}