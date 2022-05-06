#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include<math.h>
#include <ctime>
#include "Planefitting.h"
#include"Computepointspose.h"
#include"computeangle.h" 
#include"CollisionDetection.h"
#include"Pointviewer.h"
#include"cylinderfitting.h"
#include"PointProcess.h"
#include"PointCloudAligment.h"
#include <pcl/filters/passthrough.h>
string golden_pcd;
string target_pcd;
float X,Y,Z,RX,RY,RZ;
int display;
void readconfig()
{
  cv::FileStorage fs_read("config.yaml", cv::FileStorage::READ);
  string path;
  fs_read["golden_pcd"] >> golden_pcd;
  fs_read["target_pcd"] >> target_pcd;  
  fs_read["X"] >> X;  
  fs_read["Y"] >> Y;
  fs_read["Z"] >> Z;  
  fs_read["RX"] >> RX;  
  fs_read["RY"] >> RY;  
  fs_read["RZ"] >> RZ;    
  fs_read["display"] >> display;  
  fs_read.release();
}
int main(int argc, char** argv) {
    readconfig();
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr golden(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_backup(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr golden_backup(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligentarget(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr point(new pcl::PointCloud<pcl::PointXYZ>);
    point->points.resize(1);
    point->points[0].x = X;
    point->points[0].y = Y;
    point->points[0].z = Z;
    Eigen::Vector3f rpy;
    rpy = Eigen::Vector3f(RX,RY,RZ);
    Eigen::Matrix3f rpymatrix;
    
    pcl::io::loadPCDFile(golden_pcd,*golden);
    pcl::io::loadPCDFile(target_pcd,*target);
    pcl::copyPointCloud(*golden,*golden_backup);
    pcl::copyPointCloud(*target,*target_backup);
    PointProcess pp;
    pp.SetLeafSize(0.01);
    pp.DownSimple(golden);
    pp.DownSimple(target);
    pp.SetK(300);
    pp.SetStddevMulThresh(1);
    pp.Removepoint(target);
    PointCloudAligment pa;
    pa.Aligment(golden,target,aligentarget);
    Eigen::Matrix4f transform;
    Eigen::Matrix3f transform3x3;
    pa.Gettransformation(transform);
    transform3x3.block<3,3>(0,0) = transform.block<3,3>(0,0);
    pcl::transformPointCloud(*point,*point,transform);
    pcl::transformPointCloud(*golden_backup,*golden_backup,transform);
    computeangle ca;
    rpymatrix = ca.eulerAnglesToRotationMatrix(rpy);
    rpymatrix = transform3x3 * rpymatrix;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(golden_backup);
	icp.setInputTarget(target_backup);
	icp.setMaximumIterations(50);
	icp.setMaxCorrespondenceDistance(0.001);
	icp.setRANSACOutlierRejectionThreshold(0.05);
	icp.setTransformationEpsilon(1e-6);
	//icp.setEuclideanFitnessEpsilon(0.0001);
	pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    transform = icp.getFinalTransformation();
    
    pcl::transformPointCloud(*golden_backup,*golden_backup,transform);
    pcl::transformPointCloud(*point,*point,transform);
    transform3x3.block<3,3>(0,0) = transform.block<3,3>(0,0);
    rpymatrix = ca.eulerAnglesToRotationMatrix(rpy);
    rpymatrix = transform3x3 * rpymatrix;
    rpy = ca.rotationMatrixToEulerAngles(rpymatrix);
    cout << point->points[0].x << ";" << point->points[0].y << ";" <<point->points[0].z << ";" << rpy[0] <<";"<< rpy[1]<<";"<<rpy[2]<<endl;
    
    
    if(display == 1)
    {
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
        viewer->addCoordinateSystem(0.5);
        viewer->addPointCloud(golden_backup,pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(golden_backup,0.0,0.0,255.0),"cloud");
        viewer->addPointCloud(target_backup,pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(target_backup,0.0,255.0,0.0),"cloud1");
        while (!viewer->wasStopped())
        {
            viewer->spinOnce();
        }
    }
    
}