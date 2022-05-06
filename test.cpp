#include <iostream>
#include <thread>
#include <pcl/features/boundary.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>       // ndt配准文件头
#include <pcl/filters/approximate_voxel_grid.h>// 滤波文件头
#include <pcl/visualization/pcl_visualizer.h>
#include<pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>          //滤波相关类头文件
#include <pcl/segmentation/sac_segmentation.h>
#include"Planefitting.h"
#include"Computepointspose.h"
#include"PointProcess.h"
#include"PointCloudAligment.h"
#include"computeangle.h"
#include"Pointviewer.h"
using namespace std::chrono_literals;
void GetWHWithCorner(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
{
	PointProcess pp;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
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
    Pointviewer pv;
    pv.simpleVisN(cloud1);
}
int *rand_rgb(){//随机产生颜色
	int *rgb = new int[3];	
	rgb[0] = rand() % 255;
	rgb[1] = rand() % 255;
	rgb[2] = rand() % 255;
	return rgb;
}
int main(int argc, char **argv) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud4(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud5(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud6(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud7(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3boundary(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud4boundary(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud5boundary(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud6boundary(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud7boundary(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudall(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/home/slishy/Code/PCD/hanjie/pointcloud1.pcd",*cloud1);
	for (size_t i = 0; i < cloud1->points.size(); i++)
	{
		cloud1->points[i].x *= cloud1->points[i].x * 1000;
		cloud1->points[i].y *= cloud1->points[i].y * 1000;
		cloud1->points[i].z *= cloud1->points[i].z * 1000;
	}
	pcl::io::savePLYFile("p1.ply",*cloud1);
    Planefitting pf;
	Pointviewer pv;
	PointProcess pp;
    pf.SetDistanceThreshold(0.001);
    pf.extract(cloud1,cloud3,"segmentation");
	pp.extractionBoundary(cloud3,cloud3boundary);
	pf.extract(cloud1,cloud4,"segmentation");
	pp.extractionBoundary(cloud4,cloud4boundary);
	pf.extract(cloud1,cloud5,"segmentation");

	pp.extractionBoundary(cloud5,cloud5boundary);
	pf.extract(cloud1,cloud6,"segmentation");
	pp.extractionBoundary(cloud6,cloud6boundary);
	pf.extract(cloud1,cloud7,"segmentation");
	pp.extractionBoundary(cloud7,cloud7boundary);
	*cloudall = *cloud3boundary + *cloud4boundary + *cloud5boundary+ *cloud6boundary+ *cloud7boundary;
	pp.SetLeafSize(0.003);
	pp.DownSimple(cloudall);	
	vector<pcl::PointIndices>ece_inlier;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
	ece.setInputCloud(cloudall);
	ece.setClusterTolerance(0.02);
	ece.setMinClusterSize(10);
	ece.setMaxClusterSize(100);
	ece.setSearchMethod(tree);
	ece.extract(ece_inlier);
	pcl::ExtractIndices<pcl::PointXYZ> ext;
	//聚类结果展示***************************************************
	ext.setInputCloud(cloudall);
	pcl::visualization::PCLVisualizer::Ptr viewer2(new pcl::visualization::PCLVisualizer("Result of EuclideanCluster"));
	srand((unsigned)time(NULL));
	cout << ece_inlier.size()<<endl;
	for (int i = 0; i < ece_inlier.size();i++)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_copy(new pcl::PointCloud<pcl::PointXYZ>);
		vector<int> ece_inlier_ext = ece_inlier[i].indices;
		pcl::copyPointCloud(*cloudall, ece_inlier_ext, *cloud_copy);//按照索引提取点云数据
		int *rgb1 = rand_rgb();
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>rgb2(cloud1, rgb1[0], rgb1[1], rgb1[2]);
		delete[]rgb1;
		viewer2->addPointCloud(cloud_copy, to_string(i));
	}
	//viewer2->spin();
	//pv.simpleVisN(cloudall);
	/*pcl::io::savePCDFile("p1.pcd",*cloud3);
    pcl::io::savePCDFile("p2.pcd",*cloud4);  
	pcl::io::savePCDFile("p3.pcd",*cloud5);
    pcl::io::savePCDFile("p4.pcd",*cloud6);
	pcl::io::savePCDFile("p5.pcd",*cloud7);*/
   GetWHWithCorner(cloudall);
   
}