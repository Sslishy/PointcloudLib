
#include <iostream>
#include<python3.8/Python.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/features/boundary.h>
#include <pcl/common/io.h>
#include <pcl/filters/voxel_grid.h>
#include"PointProcess.h"
#include"Pointviewer.h"
#include"Computepointspose.h"
#include"computeangle.h"
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/ransac.h>
string mid_cut_pcd;
string no_end_board_pcd;
string with_end_board_pcd;
void readconfig()
{
  cv::FileStorage fs_read("config.yaml", cv::FileStorage::READ);
  string path;
  fs_read["mid_cut_pcd"] >> mid_cut_pcd;
  fs_read["no_end_board_pcd"] >> no_end_board_pcd;  
  fs_read["with_end_board_pcd"] >> with_end_board_pcd; 
  fs_read.release();
}
int main(int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr c_plane(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PCDReader reader;
    reader.read<pcl::PointXYZ>("/home/slishy/Code/PCD/circle/pointcloud.pcd",*cloud); 
    pcl::copyPointCloud(*cloud,*cloud1);
    Pointviewer pv;
    //创建一个模型参数对象，用于记录结果
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices); 
    pcl::SACSegmentation<pcl::PointXYZ> seg;     
    seg.setOptimizeCoefficients(true);      
    seg.setModelType(pcl::SACMODEL_SPHERE);  
    seg.setMethodType(pcl::SAC_RANSAC);   
    seg.setMaxIterations(500000);       
    seg.setDistanceThreshold(0.0020);    
    seg.setProbability(0.99900); 
    seg.setRadiusLimits(0.02, 0.04);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients); 
    cout << coefficients->values.size() <<endl; 
    cout<< coefficients->values[0] <<endl;
    cout<< coefficients->values[1] <<endl;
    cout<< coefficients->values[2] <<endl;
    cout<< coefficients->values[3] <<endl;
    pcl::PointXYZ center;
    center.x = coefficients->values[0];
    center.y = coefficients->values[1];
    center.z = coefficients->values[2];
    pcl::ExtractIndices<pcl::PointXYZ> extract; 
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);//false is extract inliers 
    extract.filter(*c_plane);
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->addCoordinateSystem(0.5);
	viewer->addPointCloud(cloud1,pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud1,0.0,0.0,255.0),"cloud");
	viewer->addSphere(center,coefficients->values[3],"sphere");
	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}
    }


