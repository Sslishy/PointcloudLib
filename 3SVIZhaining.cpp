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
    pcl::PointCloud<pcl::PointXYZ>::Ptr c_line(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr c_line1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr c_line2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr c_line3(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_backup(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(target_pcd,*target);


    Planefitting pf;
    pf.SetDistanceThreshold(0.001);
    pf.extract(target,target_backup,"false");
    pf.extract(target,target1,"true");
    pf.extract(target1,target2,"false");
    //pf.extract(target1,target2,"false");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>); 

    if (target_backup->points.size() < target2->points.size()){
        pcl::PointCloud<pcl::Boundary> boundaries;
        pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst; //定义一个进行边界特征估计的对象
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst; //定义一个法线估计的对象
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); //保存法线估计的结果
        normEst.setInputCloud(target_backup); 
        normEst.setRadiusSearch(0.008); //设置法线估计的半径
        normEst.compute(*normals); //将法线估计结果保存至normals
        boundEst.setInputCloud(target_backup); //设置输入的点云
        boundEst.setInputNormals(normals); //设置边界估计的法线，因为边界估计依赖于法线
        boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>)); //设置搜索方式KdTree
        boundEst.setKSearch(150);
        boundEst.compute(boundaries); //将边界估计结果保存在boundaries
        //存储估计为边界的点云数据，将边界结果保存为pcl::PointXYZ类型
        for(int i = 0; i < target_backup->points.size(); i++) 
        { 
            if(boundaries[i].boundary_point > 0) 
            { 
                cloud_boundary->push_back(target_backup->points[i]); 
            } 
        }
    }
    if(target_backup->points.size() > target2->points.size())
    {
        pcl::PointCloud<pcl::Boundary> boundaries;
        pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst; //定义一个进行边界特征估计的对象
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst; //定义一个法线估计的对象
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); //保存法线估计的结果
        normEst.setInputCloud(target2); 
        normEst.setRadiusSearch(0.008); //设置法线估计的半径
        normEst.compute(*normals); //将法线估计结果保存至normals
        boundEst.setInputCloud(target2); //设置输入的点云
        boundEst.setInputNormals(normals); //设置边界估计的法线，因为边界估计依赖于法线
        boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>)); //设置搜索方式KdTree
        boundEst.setKSearch(150);
        boundEst.compute(boundaries); //将边界估计结果保存在boundaries
        //存储估计为边界的点云数据，将边界结果保存为pcl::PointXYZ类型
        for(int i = 0; i < target2->points.size(); i++) 
        { 
            if(boundaries[i].boundary_point > 0) 
            { 
                cloud_boundary->push_back(target2->points[i]); 
            } 
        }
    }
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers表示误差能容忍的点 记录的是点云的序号
    pcl::SACSegmentation<pcl::PointXYZ> seg;     // 创建一个分割器
    seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
    seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-设置目标几何形状
    seg.setMethodType(pcl::SAC_RANSAC); 
    seg.setProbability(0.99900); 
    seg.setNumberOfThreads(100);    //分割方法：随机采样法
    seg.setDistanceThreshold(0.002);         //设置误差容忍范围，也就是阈值
    seg.setInputCloud(cloud_boundary);               //输入点云
    seg.segment(*inliers, *coefficients);   //分割点云，获得平面和法向量

    pcl::ExtractIndices<pcl::PointXYZ> extract; 
    extract.setInputCloud(cloud_boundary);
    extract.setIndices(inliers);
    extract.setNegative(false);//false is extract inliers 
    extract.filter(*c_line);
    pcl::io::savePCDFile("c_line.pcd",*c_line);
    extract.setInputCloud(cloud_boundary);
    extract.setIndices(inliers);
    extract.setNegative(true);//false is extract inliers 
    extract.filter(*cloud_boundary);

    seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
    seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-设置目标几何形状
    seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
    seg.setProbability(0.99900); 
   seg.setNumberOfThreads(100);  
    seg.setDistanceThreshold(0.002);         //设置误差容忍范围，也就是阈值
    seg.setInputCloud(cloud_boundary);               //输入点云
    seg.segment(*inliers, *coefficients);   //分割点云，获得平面和法向量
    
     extract.setInputCloud(cloud_boundary);
    extract.setIndices(inliers);
    extract.setNegative(false);//false is extract inliers 
    extract.filter(*c_line1);
    pcl::io::savePCDFile("c_line1.pcd",*c_line1);
    extract.setInputCloud(cloud_boundary);
    extract.setIndices(inliers);
    extract.setNegative(true);//false is extract inliers 
    extract.filter(*cloud_boundary);


    seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
    seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-设置目标几何形状
    seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
    seg.setProbability(0.99900); 
    seg.setDistanceThreshold(0.002);         //设置误差容忍范围，也就是阈值
    seg.setInputCloud(cloud_boundary);               //输入点云
    seg.setNumberOfThreads(100);  
    seg.segment(*inliers, *coefficients);   //分割点云，获得平面和法向量
    
     extract.setInputCloud(cloud_boundary);
    extract.setIndices(inliers);
    extract.setNegative(false);//false is extract inliers 
    extract.filter(*c_line2);
    pcl::io::savePCDFile("c_line2.pcd",*c_line2);
    extract.setInputCloud(cloud_boundary);
    extract.setIndices(inliers);
    extract.setNegative(true);//false is extract inliers 
    extract.filter(*cloud_boundary);

    seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
    seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-设置目标几何形状
    seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
    seg.setDistanceThreshold(0.002);
    seg.setProbability(0.99900);
    seg.setNumberOfThreads(100);          //设置误差容忍范围，也就是阈值
    seg.setInputCloud(cloud_boundary);               //输入点云
    seg.segment(*inliers, *coefficients);   //分割点云，获得平面和法向量
    
     extract.setInputCloud(cloud_boundary);
    extract.setIndices(inliers);
    extract.setNegative(false);//false is extract inliers 
    extract.filter(*c_line3);
    pcl::io::savePCDFile("c_line3.pcd",*c_line3);
    extract.setInputCloud(cloud_boundary);
    extract.setIndices(inliers);
    extract.setNegative(true);//false is extract inliers 
    extract.filter(*cloud_boundary);
    pcl::io::savePCDFile("boun.pcd",*cloud_boundary);

    if(display == 1)
    {
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
        viewer->addCoordinateSystem(0.5);
        viewer->addPointCloud(c_line,pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(target_backup,0.0,255.0,0.0),"cloud1");
          //viewer->addPointCloud(target1,pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(target1,255.0,255.0,0.0),"cloud2");
        //viewer->addPointCloud(target2,pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(target2,255.0,0.0,0.0),"cloud3");
        while (!viewer->wasStopped())
        {
            viewer->spinOnce();
        }
    }
    
}