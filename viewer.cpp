#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include<iostream>
#include <opencv2/opencv.hpp>
#include<Computepointspose.h>
using namespace std;
void PointCloudto2D(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,cv::Mat& image)
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
int main(int argc, char **argv)
{
	cv::Mat image(900,900,CV_8UC1);
    string path = argv[1];
    string path1 = argv[2];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	
	
    pcl::io::loadPCDFile(path,*cloud);
    pcl::io::loadPCDFile(path1,*cloud1);
	PointCloudto2D(cloud,image);
	cv::imwrite("image.png",image);
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->addCoordinateSystem(0.5);
	viewer->addPointCloud(cloud,pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud,0.0,0.0,255.0),"cloud");
	viewer->addPointCloud(cloud1,pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud1,0.0,255.0,0.0),"cloud1");
	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}
	
}