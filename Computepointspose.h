#pragma once
#include<iostream>
#include<pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/boundary.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/common/common.h>
#include<pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <Eigen/StdVector>
#include <Eigen/Geometry> 
#include <pcl/surface/mls.h>
#include <Eigen/Dense>
#include <pcl/segmentation/sac_segmentation.h>
#include<pcl/features/principal_curvatures.h>
using namespace std;
using namespace cv;
class Computepointspose
{
public:
 	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	inline void SetFindNum(int K)
	{
		m_K = K;
	}
	const inline vector<float> GetWidth(){
		return m_Width;
	}
	const inline vector<float> GetLength(){
		return m_Length;
	}
	inline void Getangle(vector<float>& outangle)
	{
		vector<float> x;
		x.resize(m_N.size());
		outangle.resize(m_N.size());
		for (size_t i = 0; i < m_N.size(); i++)
		{
			x[i] = acos(abs(m_N[i].normal_z)); 
			outangle[i] = (180/CV_PI)*x[i];
		}
	}
	
	Computepointspose();
	void computePickPoints(const vector<pcl::PointCloud<pcl::PointXYZ>> cloudlist, vector<pcl::PointXYZ>& center);
	void computePointNormal(const vector<pcl::PointCloud<pcl::PointXYZ>> cloudlist, const vector<pcl::PointXYZ> center,  vector<pcl::Normal>& N);
	void computePickpointsbyWH(const vector<pcl::PointCloud<pcl::PointXYZ>> cloudlist,vector<float>& width,vector<float>&Height,vector<pcl::PointXYZ> &center);
	void Setfilternormalangle(vector<pcl::PointCloud<pcl::PointXYZ>> &cloudlist,const float angle);
	void computePickPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointXYZ& center_out);
	void computePointNormal(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,const pcl::PointXYZ center, pcl::Normal &N);
	void Setlimitangle(const float angle,vector<pcl::Normal> &Normal_Z);
	void GetWHWithCorner(const vector<pcl::PointCloud<pcl::PointXYZ>> cloudlist,vector<float>& width,vector<float>&Height,vector<pcl::PointXYZ> &lineorgin);
	void GetWHWithCorner(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,float &width,float &Height,vector<pcl::PointXYZ> &lineorgin);
	void SetfixedAngle(const float angle , vector<pcl::Normal> &Normal_Z);
	void SetfixedAngle(const float angle , pcl::Normal &Normal_Z);
	void GetCenter(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointXYZ& center);
	void PointCloudto2D(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,cv::Mat& image);
	void Get2DAngle(const vector<pcl::PointCloud<pcl::PointXYZ>> cloudlist,vector<float> &angle);
	void PointCloudGetCrossPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,int &PointCloudindex);
	void GetCircleCenter(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointXYZ& center);
	~Computepointspose();
private:
	int m_K;
	vector<float> m_Width,m_Length;
	vector<pcl::Normal> m_N;
	
};
struct LinePara
{
	float k;
	float b;
 
};
// 获取直线参数  
void getLinePara(float& x1, float& y1, float& x2, float& y2, LinePara & LP)
{
	double m = 0;
	// 计算分子  
	m = x2 - x1;
 
	if (0 == m)
	{
		LP.k = 10000.0;
		LP.b = y1 - LP.k * x1;
	}
	else
	{
		LP.k = (y2 - y1) / (x2 - x1);
		LP.b = y1 - LP.k * x1;
	}
}
bool getCross(float & x1, float &y1, float & x2, float & y2, float & x3, float &y3, float & x4, float & y4,  Point2f & pt){
	LinePara para1, para2;
	getLinePara(x1, y1, x2, y2, para1);
	getLinePara(x3, y3, x4, y4, para2);
 
	// 判断是否平行  
	if (abs(para1.k - para2.k) > 0.5)
	{
		pt.x = (para2.b - para1.b) / (para1.k - para2.k);
		pt.y = para1.k * pt.x + para1.b;
 
		return true;
 
	}
	else
	{
		return false;
	}
}
void Computepointspose::GetWHWithCorner(const vector<pcl::PointCloud<pcl::PointXYZ>> cloudlist,vector<float>& width,vector<float>&Height,vector<pcl::PointXYZ> &lineorgin)
{
	width.resize(cloudlist.size());
	Height.resize(cloudlist.size());
	lineorgin.resize(4);
	for (size_t i = 0; i < cloudlist.size(); i++)
	{
	if (cloudlist[i].points.size() == 0)
	{
		break;
	}
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Vector4f pcacentroid;
	pcl::compute3DCentroid(*cloudlist[i].makeShared(), pcacentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloudlist[i].makeShared(), pcacentroid, covariance);
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
	vector<cv::Point2f> point22d;
	point22d.resize(cloud1->points.size());
	//pcl::io::savePCDFileASCII("cloud",*cloud1);
	for (size_t i = 0; i < cloud1->points.size(); i++)
	{
		point22d[i].x = cloud1->points[i].z+100;
		point22d[i].y = cloud1->points[i].y+100;
	}
	cv::RotatedRect pointbox = minAreaRect(point22d);
	width[i] = pointbox.size.width;
	Height[i] = pointbox.size.height;
	Point2f rect[4];
	pointbox.points(rect);
	pcl::PointXYZ point2,point3,point4,point5,pointcenter_;
	pcl::visualization::PCLVisualizer viewer("viewer");
	//getCross(rect[0].x,rect[0].y,rect[2].x,rect[2].y,rect[1].x,rect[1].y,rect[3].x,rect[3].y,centerxy);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudmid(new pcl::PointCloud<pcl::PointXYZ>);
	cloudmid->points.resize(4);
	viewer.addPointCloud(cloud1,"cloud");
	viewer.addCoordinateSystem();	
	point2.x = 0;
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
	point5.z = rect[3].x - 100;
	cloudmid->points[0] = point2;
	cloudmid->points[1] = point3;
	cloudmid->points[2] = point4;
	cloudmid->points[3] = point5;
	pcl::transformPointCloud(*cloudmid,*cloudmid,projectionTransform.inverse());
	lineorgin[0] = cloudmid->points[0];
	lineorgin[1] = cloudmid->points[1];
	lineorgin[2] = cloudmid->points[2];
	lineorgin[3] = cloudmid->points[3];
	point3.x = 0;
	point3.y = rect[1].y - 100;
	point3.z = rect[1].x - 100;
	point4.x = 0;
	point4.y = rect[2].y - 100;
	point4.z = rect[2].x - 100;
	point5.x = 0;
	point5.y = rect[3].y - 100;
	point5.z = rect[3].x - 100;
	viewer.addSphere(point2,0.001,"1");
	viewer.addSphere(point3,0.001,"2");
	viewer.addSphere(point4,0.001,"3");
	viewer.addSphere(point5,0.001,"4");
	viewer.addLine(point2,point4,"l1");
	viewer.addLine(point3,point5,"l2");
	//viewer.addSphere(pointcenter_,0.02,"5");
	}
}
void Computepointspose::GetWHWithCorner(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, float &width,float &Height,vector<pcl::PointXYZ> &lineorgin)
{
	lineorgin.resize(4);
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
	for (size_t i = 0; i < cloud1->points.size(); i++)
	{
		point22d[i].x = cloud1->points[i].z+100;
		point22d[i].y = cloud1->points[i].y+100;
	}
	cv::RotatedRect pointbox = minAreaRect(point22d);
	width = pointbox.size.width;
	Height = pointbox.size.height;
	Point2f rect[4];
	pointbox.points(rect);
	pcl::PointXYZ point2,point3,point4,point5,pointcenter_;
	pcl::visualization::PCLVisualizer viewer("viewer");
	//getCross(rect[0].x,rect[0].y,rect[2].x,rect[2].y,rect[1].x,rect[1].y,rect[3].x,rect[3].y,centerxy);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudmid(new pcl::PointCloud<pcl::PointXYZ>);
	cloudmid->points.resize(4);
	viewer.addPointCloud(cloud1,"cloud");
	viewer.addCoordinateSystem();	
	point2.x = 0;
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
	point5.z = rect[3].x - 100;
	cloudmid->points[0] = point2;
	cloudmid->points[1] = point3;
	cloudmid->points[2] = point4;
	cloudmid->points[3] = point5;
	pcl::transformPointCloud(*cloudmid,*cloudmid,projectionTransform.inverse());
	lineorgin[0] = cloudmid->points[0];
	lineorgin[1] = cloudmid->points[1];
	lineorgin[2] = cloudmid->points[2];
	lineorgin[3] = cloudmid->points[3];
	/*point3.x = 0;
	point3.y = rect[1].y - 100;
	point3.z = rect[1].x - 100;
	point4.x = 0;
	point4.y = rect[2].y - 100;
	point4.z = rect[2].x - 100;
	point5.x = 0;
	point5.y = rect[3].y - 100;
	point5.z = rect[3].x - 100;*/
	viewer.addSphere(point2,0.02,"1");
	viewer.addSphere(point3,0.02,"2");
	viewer.addSphere(point4,0.02,"3");
	viewer.addSphere(point5,0.02,"4");
	//viewer.addLine(point2,point4,"l1");
	//viewer.addLine(point3,point5,"l2");
	//viewer.addSphere(pointcenter_,0.02,"5");
	
}
void Computepointspose::computePickpointsbyWH(const vector<pcl::PointCloud<pcl::PointXYZ>> cloudlist,vector<float>& width,vector<float>&Height,vector<pcl::PointXYZ> &center)
{
	width.resize(cloudlist.size());
	Height.resize(cloudlist.size());
	center.resize(cloudlist.size());
	for (size_t i = 0; i < cloudlist.size(); i++)
	{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Vector4f pcacentroid;
	pcl::compute3DCentroid(*cloudlist[i].makeShared(), pcacentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloudlist[i].makeShared(), pcacentroid, covariance);
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
	vector<cv::Point2f> point22d;
	point22d.resize(cloud1->points.size());
	//pcl::io::savePCDFileASCII("cloud",*cloud1);
	for (size_t i = 0; i < cloud1->points.size(); i++)
	{
		point22d[i].x = cloud1->points[i].z+100;
		point22d[i].y = cloud1->points[i].y+100;
	}
	cv::RotatedRect pointbox = minAreaRect(point22d);
	width[i] = pointbox.size.width;
	Height[i] = pointbox.size.height;
	Point2f rect[4];
	Point2f centerxy;
	pointbox.points(rect);
	pcl::PointXYZ point2,point3,point4,point5,pointcenter_;
	//pcl::visualization::PCLVisualizer viewer("viewer");
	getCross(rect[0].x,rect[0].y,rect[2].x,rect[2].y,rect[1].x,rect[1].y,rect[3].x,rect[3].y,centerxy);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudmid(new pcl::PointCloud<pcl::PointXYZ>);
	cloudmid->points.resize(0);
	//viewer.addPointCloud(cloud1,"cloud");
	//viewer.addCoordinateSystem();	
	pointcenter_.x = 0;
	pointcenter_.y = centerxy.y - 100;
	pointcenter_.z = centerxy.x - 100;
	/*point2.x = 0;
	point2.y = rect[0].y - 100;
	point2.z = rect[0].x - 100;*/
	cloudmid->push_back(pointcenter_);
	//pcl::io::savePCDFile("cloudmid",*cloudmid);
	pcl::transformPointCloud(*cloudmid,*cloudmid,projectionTransform.inverse());
	//pcl::io::savePCDFile("cloudmid1",*cloudmid);
	/*Eigen::Affine3f transform(Eigen::Affine3d::Identity());
    transform.translate(bboxTransform);
	//将center转到真实物件上
	pointcenter_ = pcl::transformPoint(pointcenter_,transform);*/
	center[i] = cloudmid->points[0];
	/*cout << center[i].x <<" " <<center[i].y<<endl;
	
	point3.x = 0;
	point3.y = rect[1].y - 100;
	point3.z = rect[1].x - 100;
	point4.x = 0;
	point4.y = rect[2].y - 100;
	point4.z = rect[2].x - 100;
	point5.x = 0;
	point5.y = rect[3].y - 100;
	point5.z = rect[3].x - 100;
	viewer.addSphere(point2,0.02,"1");
	//viewer.addSphere(point3,0.02,"2");
	viewer.addSphere(point4,0.02,"3");
	//viewer.addSphere(point5,0.02,"4");
	viewer.addLine(point2,point4,"l1");
	viewer.addLine(point3,point5,"l2");
	viewer.addSphere(pointcenter_,0.02,"5");*/
	}
}
void Computepointspose::GetCenter(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointXYZ& center)
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
	for (size_t i = 0; i < cloud1->points.size(); i++)
	{
		point22d[i].x = cloud1->points[i].z+100;
		point22d[i].y = cloud1->points[i].y+100;
	}
	cv::RotatedRect pointbox = minAreaRect(point22d);
	Point2f rect[4];
	pointbox.points(rect);
	pcl::PointXYZ point2,point3,point4,point5,pointcenter_;
	pcl::visualization::PCLVisualizer viewer("viewer");
	//getCross(rect[0].x,rect[0].y,rect[2].x,rect[2].y,rect[1].x,rect[1].y,rect[3].x,rect[3].y,centerxy);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudmid(new pcl::PointCloud<pcl::PointXYZ>);
	cloudmid->points.resize(1);
	Point2f c = pointbox.center;
	//viewer.addPointCloud(cloud1,"cloud");
	//viewer.addCoordinateSystem();	
	pointcenter_.x = 0;
	pointcenter_.y = c.y - 100;
	pointcenter_.z = c.x - 100;
	cloudmid->points[0].x = pointcenter_.x;
	cloudmid->points[0].y = pointcenter_.y;
	cloudmid->points[0].z = pointcenter_.z;
	viewer.addPointCloud(cloud1,"cloud");
	viewer.addCoordinateSystem();	
	point2.x = 0;
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
	point5.z = rect[3].x - 100;
	pcl::transformPointCloud(*cloudmid,*cloudmid,projectionTransform.inverse());
	
	center.x = cloudmid->points[0].x;
	center.y = cloudmid->points[0].y;
	center.z = cloudmid->points[0].z;
	/*point3.x = 0;
	point3.y = rect[1].y - 100;
	point3.z = rect[1].x - 100;
	point4.x = 0;
	point4.y = rect[2].y - 100;
	point4.z = rect[2].x - 100;
	point5.x = 0;
	point5.y = rect[3].y - 100;
	point5.z = rect[3].x - 100;*/
	viewer.addSphere(point2,0.005,"1");
	viewer.addSphere(point3,0.005,"2");
	viewer.addSphere(point4,0.005,"3");
	viewer.addSphere(point5,0.005,"4");
	viewer.addLine(point2,point3,"line");
	viewer.addLine(point3,point4,"line1");
	viewer.addLine(point4,point5,"line2");
	viewer.addLine(point5,point2,"line3");
	//viewer.addLine(point2,point4,"l1");
	//viewer.addLine(point3,point5,"l2");
	//viewer.addSphere(pointcenter_,0.02,"5");
}
void Computepointspose::PointCloudto2D(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,cv::Mat& image)
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
	vector<cv::Point2f> point22d;
	point22d.resize(cloud1->points.size());
	//pcl::io::savePCDFileASCII("cloud",*cloud1);
	for (size_t i = 0; i < cloud1->points.size(); i++)
	{
		point22d[i].x = cloud1->points[i].z * 1000 + 500;
		point22d[i].y = cloud1->points[i].y * 1000 + 500;
		image.at<uchar>(point22d[i].x , point22d[i].y) = 255;
	}
}
void Computepointspose::PointCloudGetCrossPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, int &PointCloudindex)
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
	vector<cv::Point2f> point22d;
	point22d.resize(cloud1->points.size());
	//pcl::io::savePCDFileASCII("cloud",*cloud1);
	for (size_t i = 0; i < cloud1->points.size(); i++)
	{
		cloud1->points[i].x = 0.01;
	}
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers表示误差能容忍的点 记录的是点云的序号
    pcl::SACSegmentation<pcl::PointXYZ> seg;     // 创建一个分割器
    seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
    seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-设置目标几何形状
    seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
    seg.setDistanceThreshold(0.001);         //设置误差容忍范围，也就是阈值
    seg.setInputCloud(cloud1);               //输入点云
    seg.segment(*inliers, *coefficients); 
	pcl::ExtractIndices<pcl::PointXYZ> extract;  //分割点云，获得平面和法向量
	pcl::PointCloud<pcl::PointXYZ>::Ptr c_line(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr c_line1(new pcl::PointCloud<pcl::PointXYZ>);  
    extract.setInputCloud(cloud1);    //设置输入点云
    extract.setIndices(inliers);     //设置分割后的内点为需要提取的点集
    extract.setNegative(false);      //false提取内点, true提取外点
    extract.filter(*c_line);        //提取输出存储到c_plane2
	extract.setNegative(true);
	extract.filter(*c_line1);
	/**two**/
	seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
    seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-设置目标几何形状
    seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
    seg.setDistanceThreshold(0.001);         //设置误差容忍范围，也就是阈值
    seg.setInputCloud(c_line1);               //输入点云
    seg.segment(*inliers, *coefficients);
	extract.setInputCloud(c_line1);    //设置输入点云
    extract.setIndices(inliers);     //设置分割后的内点为需要提取的点集
    extract.setNegative(false);      //false提取内点, true提取外点
    pcl::PointCloud<pcl::PointXYZ>::Ptr c_line2(new pcl::PointCloud<pcl::PointXYZ>);  //存储直线点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr c_line3(new pcl::PointCloud<pcl::PointXYZ>);  
	extract.filter(*c_line2);        //提取输出存储到c_plane2
	extract.setNegative(true);
	extract.filter(*c_line3);
	/**three*/
	seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
    seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-设置目标几何形状
    seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
    seg.setDistanceThreshold(0.001);         //设置误差容忍范围，也就是阈值
    seg.setInputCloud(c_line3);               //输入点云
    seg.segment(*inliers, *coefficients);
	extract.setInputCloud(c_line3);    //设置输入点云
    extract.setIndices(inliers);     //设置分割后的内点为需要提取的点集
    extract.setNegative(false);      //false提取内点, true提取外点
    pcl::PointCloud<pcl::PointXYZ>::Ptr c_line4(new pcl::PointCloud<pcl::PointXYZ>);  //存储直线点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr c_line5(new pcl::PointCloud<pcl::PointXYZ>);  
	extract.filter(*c_line4);        //提取输出存储到c_plane2
	extract.setNegative(true);
	extract.filter(*c_line5);
	/*vector<pcl::PointCloud<pcl::PointXYZ>>& cloudlist;
	int i = 0, nr_points = (int)cloud1->points.size();
	while (cloud1->points.size() > 0.3 * nr_points)
	{
		cloudlist.resize(i + 1);
		seg.setInputCloud(cloud1);
		seg.segment(*inliers, *coefficients);
		
		if (inliers->indices.size() == 0)
		{
			break;
		}
		extract.setInputCloud(cloud1);
		extract.setIndices(inliers);
		switch (i)
		{
		case 0:
			extract.setNegative(false);
			extract.filter(*c_line);
			for (size_t i = 0; i < c_line->points.size(); i++)
			{
				cloudlist[0].push_back(pcl::PointXYZ(c_line->points[i].x, c_line->points[i].y, c_line->points[i].z));
			}
			//pcl::io::savePCDFile("cloud_mid.pcd", *cloud_mid);
			break;
		case 1:
			extract.setNegative(false);
			extract.filter(*cloud_mid1);
			for (size_t i = 0; i < cloud_mid1->points.size(); i++)
			{
				cloudlist[1].push_back(pcl::PointXYZ(cloud_mid1->points[i].x, cloud_mid1->points[i].y, cloud_mid1->points[i].z));
			}
			//pcl::io::savePCDFile("cloud_mid1.pcd", *cloud_mid1);
			break;
		default:
			break;
		}
		extract.setNegative(true);
		extract.filter(*cloud_out);
		*cloud1 = *cloud_out;
		i++;
		if (i == 4)
		{
			break;
		}
	}*/
	
	 pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.addCoordinateSystem(0.1);
    viewer.setBackgroundColor(0.0, 0.0, 0.5);
	//viewer.addPointCloud(cloud1,"cloud1");
	for (size_t i = 0; i < coefficients->values.size(); i++)
	{
		cout << coefficients->values[i] <<endl;
	}
	pcl::PointXYZ shap , shap1 , shap2;
	shap.x = coefficients->values[0];
	shap.y = coefficients->values[1];
	shap.z = coefficients->values[2];
	shap1.x = coefficients->values[3];
	shap1.y = coefficients->values[4];
	shap1.z = coefficients->values[5];
	double a,b,c;
	a = shap1.x + shap.x;
	b = shap1.y + shap.y;
	c = shap1.z + shap.z;
	shap2.x = a;
	shap2.y = b;
	shap2.z = c;

	viewer.addSphere(shap,0.004,"sphere");
	viewer.addSphere(shap1,0.004,"sphere1");
	viewer.addSphere(shap2,0.004,"sphere2");
	//viewer.addPointCloud<pcl::PointXYZ>(c_line5, "cloud3");
	//viewer.addPointCloud<pcl::PointXYZ>(c_line2, "cloud2");
    viewer.addPointCloud<pcl::PointXYZ>(c_line, "cloud");
	viewer.addLine(*coefficients,"1");
	PointCloudindex = 0;
/* 	/*pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud1);
	cout << "cloud size"<<cloud1->points.size()<<endl;
	//normalEstimation.setKSearch(3);
    normalEstimation.setRadiusSearch(0.004);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimation.setSearchMethod(kdtree);
    normalEstimation.compute(*normals);
	pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures>pc;
	pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloud_curvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>);
	//pcl::io::savePCDFile("1.pcd",*cloud1);
	pc.setInputCloud(cloud1);
	pc.setInputNormals(normals);
	
	pc.setSearchMethod(kdtree);
	pc.setKSearch(10);
	//pc.setRadiusSearch(0.01);
	pc.compute(*cloud_curvatures);
	cout <<"cloud_curvatures->points.size:" << cloud_curvatures->points.size() <<endl;
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.addCoordinateSystem(0.1);
    viewer.setBackgroundColor(0.0, 0.0, 0.5);
    viewer.addPointCloud<pcl::PointXYZ>(cloud1, "cloud");
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud1, normals, 2, 0.01, "normals");
    double curvature = ((*cloud_curvatures)[0].pc1 * (*cloud_curvatures)[0].pc2);
    PointCloudindex =0;
    for (size_t i = 0; i < cloud_curvatures->points.size(); i++)
    {
		
		/* cout << ((*cloud_curvatures)[i].pc1 * (*cloud_curvatures)[i].pc2) <<endl;
		/*cout << cloud_curvatures->points[i].principal_curvature_x <<endl;
		cout << cloud_curvatures->points[i].principal_curvature_y <<endl;
		cout << cloud_curvatures->points[i].principal_curvature_z <<endl;*/
      /*   if(curvature < ((*cloud_curvatures)[i].pc1 * (*cloud_curvatures)[i].pc2))
        {
            curvature = ((*cloud_curvatures)[i].pc1 * (*cloud_curvatures)[i].pc2);
			cout << "max:"<<((*cloud_curvatures)[i].pc1 * (*cloud_curvatures)[i].pc2) <<endl;
            PointCloudindex = i; 
			viewer.addSphere(cloud1->points[PointCloudindex],0.004,to_string(i));
        }  
    }
   viewer.addSphere(cloud1->points[PointCloudindex],0.02,"sphere"); */ 
}
void Computepointspose::Get2DAngle(const vector<pcl::PointCloud<pcl::PointXYZ>> cloudlist,vector<float> &angle)
{
	angle.resize(cloudlist.size());
	for (size_t i = 0; i < cloudlist.size(); i++)
	{
	if (cloudlist[i].points.size() == 0)
	{
		break;
	}
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Vector4f pcacentroid;
	pcl::compute3DCentroid(*cloudlist[i].makeShared(), pcacentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloudlist[i].makeShared(), pcacentroid, covariance);
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
	vector<cv::Point2f> point22d;
	point22d.resize(cloud1->points.size());
	//pcl::io::savePCDFileASCII("cloud",*cloud1);
	for (size_t i = 0; i < cloud1->points.size(); i++)
	{
		point22d[i].x = cloud1->points[i].z+100;
		point22d[i].y = cloud1->points[i].y+100;
	}
	cv::RotatedRect pointbox = minAreaRect(point22d);
	angle[i] = pointbox.angle;
	Point2f rect[4];
	pointbox.points(rect);
	pcl::PointXYZ point2,point3,point4,point5,pointcenter_;
	pcl::visualization::PCLVisualizer viewer("viewer");
	//getCross(rect[0].x,rect[0].y,rect[2].x,rect[2].y,rect[1].x,rect[1].y,rect[3].x,rect[3].y,centerxy);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudmid(new pcl::PointCloud<pcl::PointXYZ>);
	cloudmid->points.resize(4);
	viewer.addPointCloud(cloud1,"cloud");
	viewer.addCoordinateSystem();	
	point2.x = 0;
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
	point5.z = rect[3].x - 100;
	cloudmid->points[0] = point2;
	cloudmid->points[1] = point3;
	cloudmid->points[2] = point4;
	cloudmid->points[3] = point5;
	point3.x = 0;
	point3.y = rect[1].y - 100;
	point3.z = rect[1].x - 100;
	point4.x = 0;
	point4.y = rect[2].y - 100;
	point4.z = rect[2].x - 100;
	point5.x = 0;
	point5.y = rect[3].y - 100;
	point5.z = rect[3].x - 100;
	viewer.addSphere(point2,0.001,"1");
	viewer.addSphere(point3,0.001,"2");
	viewer.addSphere(point4,0.001,"3");
	viewer.addSphere(point5,0.001,"4");
	viewer.addLine(point2,point4,"l1");
	viewer.addLine(point3,point5,"l2");
	//viewer.addSphere(pointcenter_,0.02,"5");
	}
	
}
void Computepointspose::GetCircleCenter(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointXYZ& center)
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
	for (size_t i = 0; i < cloud1->points.size(); i++)
	{
		point22d[i].x = cloud1->points[i].z+100;
		point22d[i].y = cloud1->points[i].y+100;
	}
	cv::Point2f CricleCenter;
	float radius;
	//cv::RotatedRect pointbox = minAreaRect(point22d);
	cv::minEnclosingCircle(point22d, CricleCenter, radius);
	pcl::PointXYZ CricleCenter3D;
	CricleCenter3D.x = 0;
	CricleCenter3D.y = CricleCenter.y - 100;
	CricleCenter3D.z = CricleCenter.x - 100;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudmid(new pcl::PointCloud<pcl::PointXYZ>);
	cloudmid->resize(1);
	cloudmid->points[0].x = CricleCenter3D.x;
	cloudmid->points[0].y = CricleCenter3D.y;
	cloudmid->points[0].z = CricleCenter3D.z;
	pcl::transformPointCloud(*cloudmid,*cloudmid,projectionTransform.inverse());
	center.x = cloudmid->points[0].x;
	center.y = cloudmid->points[0].y;
	center.z = cloudmid->points[0].z;
}